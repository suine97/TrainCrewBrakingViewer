using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using TrainCrew;

namespace TrainCrewBrakingViewer
{
    /// <summary>
    /// ブレーキ曲線データクラス
    /// </summary>
    /// <remarks>
    /// Bin\(駅名).bin から読み込んだ、勾配地形情報を織り込み済みのブレーキ曲線。
    /// ファイルの構造は MessagePack で以下の通り(バージョン2)。
    ///   ルート    array: [バージョン(=2), レイアウト, 距離の刻み幅[m], 速度の上限[km/h], 曲線]
    ///   曲線      map  : キーは方向(0=下り, 1=上り)
    ///   第2階層   map  : レイアウト2 … キーは編成(string、BrakingCurveFormation 参照)
    ///                    レイアウト0 … キーは車両形式(TASC.TrainModel の値)
    ///   第3階層   map  : キーはブレーキノッチ(TrainState.Bnotch の値)
    ///   第4階層   array: 速度[km/h](停止位置から手前へ、刻み幅ごと)
    /// 距離は 添字×刻み幅 で求まる。速度の上限に達した曲線はその点で打ち切られているので、
    /// 配列の末尾より手前の距離は末尾の値(=上限)で頭打ちにする。
    /// ブレーキが効かないノッチは曲線そのものが入っていない。
    ///
    /// レイアウト2で同じ編成が無い場合は、先頭車の形式の単独編成のうち
    /// 両数が最も近いもの(同じ近さなら少ない方)を代わりに使う。
    ///
    /// ルートが map のファイルは旧形式(方向→形式→ノッチ→[距離, 速度] の組)として読む。
    /// </remarks>
    public sealed class BrakingCurveSet
    {
        /// <summary>
        /// ブレーキノッチの最大数
        /// </summary>
        public const int NotchCount = 9;

        /// <summary>
        /// 駅名
        /// </summary>
        public string StationName { get; }

        /// <summary>
        /// 方向(0=下り, 1=上り)
        /// </summary>
        public int DirectionKey { get; }

        /// <summary>
        /// 車両形式
        /// </summary>
        public int TrainModelIndex { get; }

        /// <summary>
        /// 要求した編成(BrakingCurveFormation.GetKey の値、判定できなければnull)
        /// </summary>
        public string FormationKey { get; }

        /// <summary>
        /// 実際に使った編成(代わりの編成を使ったときは FormationKey と異なる。レイアウト0・旧形式はnull)
        /// </summary>
        public string UsedFormationKey { get; }

        /// <summary>
        /// レイアウト(方向→車両形式→ノッチ)
        /// </summary>
        private const int LayoutByModel = 0;

        /// <summary>
        /// レイアウト(方向→編成→ノッチ)
        /// </summary>
        private const int LayoutByFormation = 2;

        /// <summary>
        /// 対応するファイルのバージョン
        /// </summary>
        private const int FileVersion = 2;

        /// <summary>
        /// ノッチ別の速度[km/h](距離の昇順、該当データが無いノッチはnull)
        /// </summary>
        /// <remarks>ノッチごとに長さが異なる(速度の上限に達した点で打ち切られている)。</remarks>
        private readonly float[][] speeds;

        /// <summary>
        /// 距離の下限[m]
        /// </summary>
        private readonly float minDistance;

        /// <summary>
        /// 距離の刻み幅[m]
        /// </summary>
        private readonly float step;

        private BrakingCurveSet(CurveRequest request, string usedFormationKey,
                               float[][] speeds, float minDistance, float step)
        {
            StationName = request.StationName;
            DirectionKey = request.DirectionKey;
            TrainModelIndex = request.TrainModelIndex;
            FormationKey = request.FormationKey;
            UsedFormationKey = usedFormationKey;
            this.speeds = speeds;
            this.minDistance = minDistance;
            this.step = step;
        }

        /// <summary>
        /// 指定した条件のデータかどうかを判定するメソッド
        /// </summary>
        public bool Matches(string stationName, int directionKey, int trainModelIndex, string formationKey)
        {
            return DirectionKey == directionKey
                && TrainModelIndex == trainModelIndex
                && string.Equals(StationName, stationName, StringComparison.Ordinal)
                && string.Equals(FormationKey, formationKey, StringComparison.Ordinal);
        }

        /// <summary>
        /// 速度取得メソッド
        /// </summary>
        /// <param name="bnotch">ブレーキノッチ(TrainState.Bnotch と同じ値)</param>
        /// <param name="distance">停止位置までの距離[m]</param>
        /// <param name="speed">その地点の速度[km/h]</param>
        /// <returns>該当するデータがあればtrue</returns>
        /// <remarks>データの範囲外の距離は端の値で頭打ちにする。</remarks>
        public bool TryGetSpeed(int bnotch, float distance, out float speed)
        {
            speed = 0.0f;
            if (bnotch < 0 || bnotch >= NotchCount) return false;

            float[] values = speeds[bnotch];
            if (values == null || values.Length == 0) return false;

            //曲線の長さはノッチごとに異なるので、上限は添字で頭打ちにする
            float position = (distance - minDistance) / step;
            int index;
            if (!(position > 0.0f)) index = 0;
            else if (position >= values.Length - 1) index = values.Length - 1;
            else index = (int)MathF.Round(position);

            speed = values[index];
            return true;
        }

        /// <summary>
        /// 読み込みメソッド
        /// </summary>
        /// <param name="filePath">ファイルパス</param>
        /// <param name="request">読み込む条件</param>
        /// <returns>読み込んだ曲線(該当データが無ければnull)</returns>
        /// <remarks>
        /// ファイルは40MB前後あるが、必要なのは1方向×1編成の分だけなので、
        /// 該当しない方向・編成はデコードせず読み飛ばす。
        /// </remarks>
        public static BrakingCurveSet Load(string filePath, CurveRequest request)
        {
            var reader = new MessagePackReader(File.ReadAllBytes(filePath));

            // ルートが array ならバージョン2、map なら旧形式
            bool isLegacy = !reader.IsNextArray();
            float step = 0.0f;
            int layout = LayoutByModel;
            if (!isLegacy)
            {
                int headerCount = reader.ReadArrayHeader();
                if (headerCount < 5) throw new FormatException("ヘッダの要素数が足りません");

                int version = reader.ReadInt();
                if (version != FileVersion) throw new FormatException($"未対応のバージョンです ({version})");

                layout = reader.ReadInt();
                if (layout != LayoutByModel && layout != LayoutByFormation)
                    throw new FormatException($"未対応のレイアウトです ({layout})");

                step = reader.ReadSingle();
                reader.Skip();  //速度の上限(曲線の末尾の値として織り込み済み)
                if (!(step > 0.0f)) throw new FormatException("距離の刻み幅が不正です");
            }

            int directionCount = reader.ReadMapHeader();
            for (var d = 0; d < directionCount; d++)
            {
                if (reader.ReadInt() != request.DirectionKey)
                {
                    reader.Skip();
                    continue;
                }

                if (layout == LayoutByFormation) return FindFormation(reader, request, step);

                int modelCount = reader.ReadMapHeader();
                for (var m = 0; m < modelCount; m++)
                {
                    if (reader.ReadInt() != request.TrainModelIndex)
                    {
                        reader.Skip();
                        continue;
                    }

                    return isLegacy
                        ? ReadLegacyNotches(reader, request)
                        : ReadNotches(reader, request, null, step);
                }
                return null;
            }
            return null;
        }

        /// <summary>
        /// 編成の検索メソッド(レイアウト2)
        /// </summary>
        /// <remarks>
        /// 同じ編成があればそれを読む。無ければ先頭車の形式の単独編成のうち、
        /// 両数が最も近いもの(同じ近さなら少ない方)を読む。
        /// </remarks>
        private static BrakingCurveSet FindFormation(MessagePackReader reader, CurveRequest request, float step)
        {
            string singlePrefix = request.TrainModelIndex + ":";
            string nearKey = null;
            int nearCarNum = 0;
            int nearPosition = -1;

            int formationCount = reader.ReadMapHeader();
            for (var f = 0; f < formationCount; f++)
            {
                string key = reader.ReadString();

                if (request.FormationKey != null && string.Equals(key, request.FormationKey, StringComparison.Ordinal))
                    return ReadNotches(reader, request, key, step);

                //先頭車の形式の単独編成なら代わりの候補として位置を覚えておく
                if (key != null && key.StartsWith(singlePrefix, StringComparison.Ordinal) && key.IndexOf('+') < 0
                    && int.TryParse(key.Substring(singlePrefix.Length), out int carNum))
                {
                    int diff = Math.Abs(carNum - request.CarCount);
                    int nearDiff = Math.Abs(nearCarNum - request.CarCount);
                    if (nearKey == null || diff < nearDiff || (diff == nearDiff && carNum < nearCarNum))
                    {
                        nearKey = key;
                        nearCarNum = carNum;
                        nearPosition = reader.Position;
                    }
                }

                reader.Skip();
            }

            if (nearKey == null) return null;

            reader.Position = nearPosition;
            return ReadNotches(reader, request, nearKey, step);
        }

        /// <summary>
        /// ノッチ別曲線の読み込みメソッド
        /// </summary>
        private static BrakingCurveSet ReadNotches(MessagePackReader reader, CurveRequest request, string usedFormationKey, float step)
        {
            var speeds = new float[NotchCount][];
            bool hasCurve = false;

            int notchCount = reader.ReadMapHeader();
            for (var n = 0; n < notchCount; n++)
            {
                int notch = reader.ReadInt();
                int pointCount = reader.ReadArrayHeader();

                var values = new float[pointCount];
                for (var i = 0; i < pointCount; i++) values[i] = reader.ReadSingle();

                if (pointCount < 2 || notch < 0 || notch >= NotchCount) continue;

                speeds[notch] = values;
                hasCurve = true;
            }

            return hasCurve
                ? new BrakingCurveSet(request, usedFormationKey, speeds, 0.0f, step)
                : null;
        }

        /// <summary>
        /// ノッチ別曲線の読み込みメソッド(旧形式)
        /// </summary>
        private static BrakingCurveSet ReadLegacyNotches(MessagePackReader reader, CurveRequest request)
        {
            var speeds = new float[NotchCount][];
            float minDistance = 0.0f;
            float maxDistance = 0.0f;
            float step = 0.0f;

            int notchCount = reader.ReadMapHeader();
            for (var n = 0; n < notchCount; n++)
            {
                int notch = reader.ReadInt();
                int pointCount = reader.ReadArrayHeader();

                var values = new float[pointCount];
                float firstDistance = 0.0f;
                float lastDistance = 0.0f;
                bool hasSpeed = false;

                for (var i = 0; i < pointCount; i++)
                {
                    int fieldCount = reader.ReadArrayHeader();
                    float distance = reader.ReadSingle();
                    float speed = reader.ReadSingle();
                    for (var f = 2; f < fieldCount; f++) reader.Skip();

                    if (i == 0) firstDistance = distance;
                    lastDistance = distance;
                    values[i] = speed;
                    if (speed > 0.0f) hasSpeed = true;
                }

                // 距離の昇順に揃える
                if (firstDistance > lastDistance) Array.Reverse(values);

                // 全区間で0のノッチはその車両で使われていないため保持しない
                if (!hasSpeed || pointCount < 2 || notch < 0 || notch >= NotchCount) continue;

                speeds[notch] = values;
                minDistance = Math.Min(firstDistance, lastDistance);
                maxDistance = Math.Max(firstDistance, lastDistance);
                step = (maxDistance - minDistance) / (pointCount - 1);
            }

            return (step > 0.0f)
                ? new BrakingCurveSet(request, null, speeds, minDistance, step)
                : null;
        }
    }

    /// <summary>
    /// ブレーキ曲線の読み込み条件
    /// </summary>
    public sealed class CurveRequest
    {
        /// <summary>
        /// 駅名
        /// </summary>
        public string StationName { get; init; }

        /// <summary>
        /// 方向(0=下り, 1=上り)
        /// </summary>
        public int DirectionKey { get; init; }

        /// <summary>
        /// 先頭車の車両形式
        /// </summary>
        public int TrainModelIndex { get; init; }

        /// <summary>
        /// 編成(BrakingCurveFormation.GetKey の値、判定できなければnull)
        /// </summary>
        public string FormationKey { get; init; }

        /// <summary>
        /// 編成の両数
        /// </summary>
        public int CarCount { get; init; }

        /// <summary>
        /// 同じ条件かどうかの判定メソッド
        /// </summary>
        public bool IsSame(CurveRequest other)
        {
            return other != null
                && DirectionKey == other.DirectionKey
                && TrainModelIndex == other.TrainModelIndex
                && CarCount == other.CarCount
                && string.Equals(StationName, other.StationName, StringComparison.Ordinal)
                && string.Equals(FormationKey, other.FormationKey, StringComparison.Ordinal);
        }

        /// <summary>
        /// 読み込み状況の表示用文字列
        /// </summary>
        public override string ToString()
        {
            return $"{StationName} 方向{DirectionKey} 編成{FormationKey ?? $"不明(形式{TrainModelIndex} {CarCount}両)"}";
        }
    }

    /// <summary>
    /// 編成判定クラス
    /// </summary>
    /// <remarks>
    /// TrainState.CarStates から編成を判定し、binの編成のキーを作る。
    ///   ・運転台のある車両(HasDriverCab)で区切った1本を単位とする
    ///     (先頭の運転台から、次に運転台がある車両までを1単位とする)
    ///   ・キーは「形式:両数」を '+' でつないだもの(例 "2:2+3:4" = 5300形2両+4300形4両)
    ///   ・単位は (形式, 両数) の昇順に並べる(binを作る側と同じ規則。並び順は曲線に影響しない)
    /// 形式が判定できない車両があればnullを返す。
    /// </remarks>
    public static class BrakingCurveFormation
    {
        /// <summary>
        /// 車両形式名(CarState.CarModel)→車両形式
        /// </summary>
        private static readonly Dictionary<string, TASC.TrainModel> modelNames = new Dictionary<string, TASC.TrainModel>
        {
            ["5320"] = TASC.TrainModel.series5320,
            ["5300"] = TASC.TrainModel.series5300,
            ["4300"] = TASC.TrainModel.series4300,
            ["4321"] = TASC.TrainModel.Car4321F,
            ["4000"] = TASC.TrainModel.series4000,
            ["4000R"] = TASC.TrainModel.Car4000R,
            ["50000"] = TASC.TrainModel.series50000,
            ["3300V"] = TASC.TrainModel.Car3300V,
            ["3000"] = TASC.TrainModel.series3000,
            ["3020"] = TASC.TrainModel.series3020,
            ["4600"] = TASC.TrainModel.series4600,
            ["5600"] = TASC.TrainModel.series5600,
        };

        /// <summary>
        /// 編成のキー取得メソッド
        /// </summary>
        /// <param name="cars">先頭からの車両の状態</param>
        /// <returns>編成のキー(判定できなければnull)</returns>
        public static string GetKey(IReadOnlyList<CarState> cars)
        {
            if (cars == null || cars.Count == 0) return null;

            var units = new List<(int Model, int CarNum)>();
            var start = 0;
            while (start < cars.Count)
            {
                //次に運転台がある車両までを1単位とする(見つからなければ最後尾まで)
                int end = start + 1;
                while (end < cars.Count && !cars[end].HasDriverCab) end++;
                if (end >= cars.Count) end = cars.Count - 1;

                if (cars[start].CarModel == null || !modelNames.TryGetValue(cars[start].CarModel, out var model))
                    return null;

                units.Add(((int)model, end - start + 1));
                start = end + 1;
            }

            units.Sort();

            var sb = new StringBuilder();
            foreach (var (model, carNum) in units)
            {
                if (sb.Length > 0) sb.Append('+');
                sb.Append(model).Append(':').Append(carNum);
            }
            return sb.ToString();
        }
    }

    /// <summary>
    /// ブレーキ曲線データ管理クラス
    /// </summary>
    /// <remarks>
    /// 駅・方向・車両形式・編成が変わったときだけ、バックグラウンドで読み込む。
    /// ファイルが40MB前後あるためUIスレッドでは読み込まない。
    /// </remarks>
    public sealed class BrakingCurveRepository
    {
        /// <summary>
        /// ブレーキ曲線データの格納フォルダ
        /// </summary>
        private const string DirectoryName = "Bin";

        private readonly object sync = new object();

        /// <summary>
        /// 読み込みを試みた条件(成否に関わらず記録し、同じ条件で読み直さない)
        /// </summary>
        private CurveRequest attempted;

        /// <summary>
        /// 読み込み済みの曲線
        /// </summary>
        private BrakingCurveSet current;

        /// <summary>
        /// 読み込み状況
        /// </summary>
        private string status = "bin未要求";

        /// <summary>
        /// 読み込み済みの曲線
        /// </summary>
        public BrakingCurveSet Current => Volatile.Read(ref current);

        /// <summary>
        /// 読み込み状況(binが使われない原因の切り分け用)
        /// </summary>
        public string Status => Volatile.Read(ref status);

        /// <summary>
        /// ブレーキ曲線データのファイルパス解決メソッド
        /// </summary>
        /// <param name="stationName">駅名</param>
        /// <returns>存在するファイルのパス(見つからなければnull)</returns>
        /// <remarks>
        /// カレントディレクトリは起動方法によって変わるため、まずexeの位置を基準に探す。
        /// </remarks>
        private static string ResolveFilePath(string stationName)
        {
            string fileName = stationName + ".bin";

            string path = Path.Combine(AppContext.BaseDirectory, DirectoryName, fileName);
            if (File.Exists(path)) return path;

            path = Path.Combine(DirectoryName, fileName);
            return File.Exists(path) ? path : null;
        }

        /// <summary>
        /// 曲線の読み込み要求メソッド
        /// </summary>
        /// <param name="stationName">駅名</param>
        /// <param name="directionKey">方向(0=下り, 1=上り)</param>
        /// <param name="trainModelIndex">先頭車の車両形式</param>
        /// <param name="cars">先頭からの車両の状態(編成の判定に使う)</param>
        public void Request(string stationName, int directionKey, int trainModelIndex, IReadOnlyList<CarState> cars)
        {
            if (string.IsNullOrEmpty(stationName)) return;

            var request = new CurveRequest
            {
                StationName = stationName,
                DirectionKey = directionKey,
                TrainModelIndex = trainModelIndex,
                FormationKey = BrakingCurveFormation.GetKey(cars),
                CarCount = cars?.Count ?? 0,
            };

            lock (sync)
            {
                if (request.IsSame(attempted)) return;
                attempted = request;
            }

            Volatile.Write(ref status, $"bin読込中: {request}");

            Task.Run(() =>
            {
                try
                {
                    string path = ResolveFilePath(stationName);
                    if (path == null)
                    {
                        Volatile.Write(ref status, $"bin無し: {Path.Combine(DirectoryName, stationName + ".bin")}");
                        return;
                    }

                    var set = BrakingCurveSet.Load(path, request);
                    if (set == null)
                    {
                        Volatile.Write(ref status, $"bin該当データ無し: {request}");
                        return;
                    }

                    Volatile.Write(ref current, set);

                    //代わりの編成を使ったときはそれも表示する
                    string substitute = (set.UsedFormationKey != null && set.UsedFormationKey != set.FormationKey)
                        ? $" (代用: {set.UsedFormationKey})"
                        : "";
                    Volatile.Write(ref status, $"bin読込済: {request}{substitute}");
                }
                catch (Exception ex)
                {
                    Volatile.Write(ref status, $"bin読込失敗: {ex.GetType().Name} {ex.Message}");
                    Debug.WriteLine($"{ex}");
                }
            });
        }
    }
}
