using System;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

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
    ///   第2階層   map  : キーは車両形式(TASC.TrainModel の値)
    ///   第3階層   map  : キーはブレーキノッチ(TrainState.Bnotch の値)
    ///   第4階層   array: 速度[km/h](停止位置から手前へ、刻み幅ごと)
    /// 距離は 添字×刻み幅 で求まる。速度の上限に達した曲線はその点で打ち切られているので、
    /// 配列の末尾より手前の距離は末尾の値(=上限)で頭打ちにする。
    /// ブレーキが効かないノッチは曲線そのものが入っていない。
    /// レイアウトは 0(方向→形式→ノッチ) のみ対応する。
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

        private BrakingCurveSet(string stationName, int directionKey, int trainModelIndex,
                               float[][] speeds, float minDistance, float step)
        {
            StationName = stationName;
            DirectionKey = directionKey;
            TrainModelIndex = trainModelIndex;
            this.speeds = speeds;
            this.minDistance = minDistance;
            this.step = step;
        }

        /// <summary>
        /// 指定した条件のデータかどうかを判定するメソッド
        /// </summary>
        public bool Matches(string stationName, int directionKey, int trainModelIndex)
        {
            return DirectionKey == directionKey
                && TrainModelIndex == trainModelIndex
                && string.Equals(StationName, stationName, StringComparison.Ordinal);
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
        /// <param name="stationName">駅名</param>
        /// <param name="directionKey">方向(0=下り, 1=上り)</param>
        /// <param name="trainModelIndex">車両形式</param>
        /// <returns>読み込んだ曲線(該当データが無ければnull)</returns>
        /// <remarks>
        /// ファイルは30MB近くあるが、必要なのは1方向×1形式の分だけ(全体の約4%)なので、
        /// 該当しない方向・形式はデコードせず読み飛ばす。
        /// </remarks>
        public static BrakingCurveSet Load(string filePath, string stationName, int directionKey, int trainModelIndex)
        {
            var reader = new MessagePackReader(File.ReadAllBytes(filePath));

            // ルートが array ならバージョン2、map なら旧形式
            bool isLegacy = !reader.IsNextArray();
            float step = 0.0f;
            if (!isLegacy)
            {
                int headerCount = reader.ReadArrayHeader();
                if (headerCount < 5) throw new FormatException("ヘッダの要素数が足りません");

                int version = reader.ReadInt();
                if (version != FileVersion) throw new FormatException($"未対応のバージョンです ({version})");

                int layout = reader.ReadInt();
                if (layout != 0) throw new FormatException($"未対応のレイアウトです ({layout})");

                step = reader.ReadSingle();
                reader.Skip();  //速度の上限(曲線の末尾の値として織り込み済み)
                if (!(step > 0.0f)) throw new FormatException("距離の刻み幅が不正です");
            }

            int directionCount = reader.ReadMapHeader();
            for (var d = 0; d < directionCount; d++)
            {
                if (reader.ReadInt() != directionKey)
                {
                    reader.Skip();
                    continue;
                }

                int modelCount = reader.ReadMapHeader();
                for (var m = 0; m < modelCount; m++)
                {
                    if (reader.ReadInt() != trainModelIndex)
                    {
                        reader.Skip();
                        continue;
                    }

                    return isLegacy
                        ? ReadLegacyNotches(reader, stationName, directionKey, trainModelIndex)
                        : ReadNotches(reader, stationName, directionKey, trainModelIndex, step);
                }
                return null;
            }
            return null;
        }

        /// <summary>
        /// ノッチ別曲線の読み込みメソッド
        /// </summary>
        private static BrakingCurveSet ReadNotches(MessagePackReader reader, string stationName, int directionKey, int trainModelIndex, float step)
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
                ? new BrakingCurveSet(stationName, directionKey, trainModelIndex, speeds, 0.0f, step)
                : null;
        }

        /// <summary>
        /// ノッチ別曲線の読み込みメソッド(旧形式)
        /// </summary>
        private static BrakingCurveSet ReadLegacyNotches(MessagePackReader reader, string stationName, int directionKey, int trainModelIndex)
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
                ? new BrakingCurveSet(stationName, directionKey, trainModelIndex, speeds, minDistance, step)
                : null;
        }
    }

    /// <summary>
    /// ブレーキ曲線データ管理クラス
    /// </summary>
    /// <remarks>
    /// 駅・方向・車両形式が変わったときだけ、バックグラウンドで読み込む。
    /// ファイルが30MB近くあるためUIスレッドでは読み込まない。
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
        private string attemptedStationName;
        private int attemptedDirectionKey = -1;
        private int attemptedTrainModelIndex = -1;

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
        /// <param name="trainModelIndex">車両形式</param>
        public void Request(string stationName, int directionKey, int trainModelIndex)
        {
            if (string.IsNullOrEmpty(stationName)) return;

            lock (sync)
            {
                if (attemptedDirectionKey == directionKey
                    && attemptedTrainModelIndex == trainModelIndex
                    && string.Equals(attemptedStationName, stationName, StringComparison.Ordinal))
                {
                    return;
                }
                attemptedStationName = stationName;
                attemptedDirectionKey = directionKey;
                attemptedTrainModelIndex = trainModelIndex;
            }

            Volatile.Write(ref status, $"bin読込中: {stationName}");

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

                    var set = BrakingCurveSet.Load(path, stationName, directionKey, trainModelIndex);
                    if (set == null)
                    {
                        Volatile.Write(ref status, $"bin該当データ無し: {stationName} 方向{directionKey} 形式{trainModelIndex}");
                        return;
                    }

                    Volatile.Write(ref current, set);
                    Volatile.Write(ref status, $"bin読込済: {stationName} 方向{directionKey} 形式{trainModelIndex}");
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
