using System;
using System.Xml.Linq;

namespace TrainCrewBrakingViewer
{
    /// <summary>
    /// 表示設定クラス
    /// </summary>
    /// <remarks>
    /// Xml\Setting.xml から読み込む。ファイルが無い場合や値が不正な場合は既定値で動作する。
    /// </remarks>
    public class ViewerSetting
    {
        /// <summary>
        /// 設定ファイルのパス
        /// </summary>
        private const string FilePath = @"Xml\Setting.xml";

        /// <summary>
        /// ウィンドウを透過するか
        /// </summary>
        /// <remarks>
        /// WPFは透過ウィンドウに対してGPU描画を無効化し、ウィンドウ全体をCPUで合成するため、
        /// 描画負荷がウィンドウ面積に比例して増える。大きなウィンドウで重い場合はfalseにする。
        /// </remarks>
        public bool Transparent { get; private set; } = true;

        /// <summary>
        /// グラフの更新間隔[ms]
        /// </summary>
        public int UpdateIntervalMs { get; private set; } = 50;

        /// <summary>
        /// 減速曲線のサンプル点数
        /// </summary>
        /// <remarks>
        /// 描画コストをウィンドウ幅から切り離すための固定点数。増やすと滑らかになるが重くなる。
        /// </remarks>
        public int CurveSampleCount { get; private set; } = 256;

        /// <summary>
        /// 設定読み込みメソッド
        /// </summary>
        /// <returns>読み込んだ設定(失敗時は既定値)</returns>
        public static ViewerSetting Load()
        {
            var setting = new ViewerSetting();
            try
            {
                var root = XElement.Load(FilePath);

                setting.Transparent = ReadBool(root, nameof(Transparent), setting.Transparent);
                setting.UpdateIntervalMs = ReadInt(root, nameof(UpdateIntervalMs), setting.UpdateIntervalMs, 10, 1000);
                setting.CurveSampleCount = ReadInt(root, nameof(CurveSampleCount), setting.CurveSampleCount, 16, 2048);
            }
            catch
            {
                return new ViewerSetting();
            }
            return setting;
        }

        /// <summary>
        /// bool値読み込みメソッド
        /// </summary>
        private static bool ReadBool(XElement root, string name, bool defaultValue)
        {
            var element = root.Element(name);
            return (element != null && bool.TryParse(element.Value.Trim(), out bool value)) ? value : defaultValue;
        }

        /// <summary>
        /// int値読み込みメソッド(範囲外は丸める)
        /// </summary>
        private static int ReadInt(XElement root, string name, int defaultValue, int min, int max)
        {
            var element = root.Element(name);
            if (element == null || !int.TryParse(element.Value.Trim(), out int value)) return defaultValue;
            return Math.Clamp(value, min, max);
        }
    }
}
