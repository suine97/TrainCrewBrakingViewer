using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Linq;
using System.Text.RegularExpressions;
using TrainCrew;

namespace TrainCrewBrakingViewer
{
    /// <summary>
    /// TASC演算クラス
    /// </summary>
    public class TASC
    {
        /// <summary>
        /// TASC 停車パターン用減速度
        /// </summary>
        private readonly float fTASCStoppingDeceleration = 3.0f;

        /// <summary>
        /// TASC 軽減パターン用減速度
        /// </summary>
        private readonly float fTASCStoppingReductionDeceleration = 2.0f;

        /// <summary>
        /// TASC 速度制限パターン用減速度
        /// </summary>
        private readonly float fTASCLimitSpeedDeceleration = 2.5f;

        /// <summary>
        /// TASC 停車パターン
        /// </summary>
        private float fTASCStoppingPattern = 0.0f;

        /// <summary>
        /// TASC 停車軽減パターン
        /// </summary>
        private float fTASCStoppingReductionPattern = 0.0f;

        /// <summary>
        /// TASC 停車パターン用オフセット距離
        /// </summary>
        private readonly float fTASCDistanceOffset = 2.00f;

        /// <summary>
        /// TASC 一時保存用目標制限速度
        /// </summary>
        private float strTargetLimitSpeed = 0.0f;

        /// <summary>
        /// TASC 一時保存用目標制限距離
        /// </summary>
        private float strTargetLimitDistance = 0.0f;

        /// <summary>
        /// TASC 駅停車判定
        /// </summary>
        private bool IsTASCStoppedStation = false;

        /// <summary>
        /// TASC 有効判定
        /// </summary>
        public bool IsTASCEnable = true;

        /// <summary>
        /// TASC 速度制御有効判定
        /// </summary>
        public bool IsTASCSpeedControlEnable = true;

        /// <summary>
        /// TASC 動作開始判定
        /// </summary>
        public bool IsTASCOperation = false;

        /// <summary>
        /// TASC ブレーキ動作判定
        /// </summary>
        public bool IsTASCBraking = false;

        /// <summary>
        /// TASC 動作フェーズ
        /// [制御待機, 停車制御, 停車制御(低減), 速度制御, 抑速制御, 停車, 解除]
        /// </summary>
        public string sTASCPhase = "解除";

        /// <summary>
        /// TASC パターンモード
        /// [平常, 高速, 低速]
        /// </summary>
        public string sTASCPatternMode = "平常";

        /// <summary>
        /// TASC 停車演算パターン(km/h)
        /// </summary>
        public float fTASCPatternSpeed = 0.0f;

        /// <summary>
        /// TASC 速度制限演算パターン(km/h)
        /// </summary>
        public float fTASCLimitPatternSpeed = 0.0f;

        /// <summary>
        /// TASC XML 制限速度(km/h)
        /// </summary>
        public float fTASCXmlLimitSpeed = 0.0f;

        /// <summary>
        /// TASC XML 制限速度までの残り距離(m)
        /// </summary>
        public float fTASCXmlLimitDistance = 0.0f;

        /// <summary>
        /// TASC 演算減速度(km/h/s)
        /// </summary>
        public float fTASCDeceleration = 0.0f;

        /// <summary>
        /// TASC ハンドル段数
        /// </summary>
        public int iTASCNotch = 0;

        /// <summary>
        /// TASC SAP圧力値(kPa)
        /// </summary>
        public float fTASCSAPPressure = 0.0f;

        /// <summary>
        /// TASC 平均勾配値(‰)
        /// </summary>
        public float fTASCGradientAverage = 0.0f;

        /// <summary>
        /// TASC 制動待機距離[m]
        /// </summary>
        public float fTASCStandbyBreakingDistance = 700.0f;

        /// <summary>
        /// ツーハンドル運転台判定
        /// </summary>
        public bool IsTwoHandle = false;

        /// <summary>
        /// 電磁直通ブレーキ車判定
        /// </summary>
        public bool IsSMEEBrake = false;

        /// <summary>
        /// SAP圧リセット判定
        /// </summary>
        public bool IsSAPReset = false;

        /// <summary>
        /// 勾配係数
        /// </summary>
        public readonly int iGradientCoefficient = 35;

        /// <summary>
        /// 停止位置範囲
        /// </summary>
        private readonly float fStopRange = 3.00f;

        /// <summary>
        /// 停止位置オフセット距離
        /// </summary>
        private float fStopPositionOffset = 0.0f;

        /// <summary>
        /// 空走時間[s]
        /// </summary>
        private readonly float[] freeRunningTime = new float[13]
        {
            0.5f, //None
            0.5f, //5320形
            0.5f, //5300形
            0.5f, //4300形
            0.5f, //4321F
            0.5f, //4000形
            0.5f, //4000形更新車
            0.5f, //50000形
            0.5f, //3300形VVVF
            0.5f, //3000形
            0.5f, //3020形
            0.5f, //4600形
            0.5f, //5600形
        };

        /// <summary>
        /// 最大減速度[km/h/s]
        /// </summary>
        public readonly float[] maxDeceleration = new float[13]
        {
            4.60f, //None
            4.60f, //5320形
            4.60f, //5300形
            4.60f, //4300形
            4.60f, //4321F
            4.60f, //4000形
            4.60f, //4000形更新車
            4.60f, //50000形
            4.60f, //3300形VVVF
            4.20f, //3000形
            4.20f, //3020形
            4.60f, //4600形
            4.60f, //5600形
        };

        /// <summary>
        /// 最大SAP圧力値[kPa]
        /// </summary>
        private readonly float[] maxPressure = new float[13]
        {
            400.00f, //None
            400.00f, //5320形
            400.00f, //5300形
            400.00f, //4300形
            400.00f, //4321F
            400.00f, //4000形
            400.00f, //4000形更新車
            400.00f, //50000形
            400.00f, //3300形VVVF
            400.00f, //3000形
            400.00f, //3020形
            400.00f, //4600形
            400.00f, //5600形
        };

        /// <summary>
        /// 減速度係数(%)
        /// </summary>
        public readonly float[][] constDeceleration = new float[13][]
        {
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //None
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //5320形
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //5300形
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //4300形
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //4321F
            new float[] { 0.11f, 0.23f, 0.36f, 0.50f, 0.64f, 0.77f, 0.91f}, //4000形
            new float[] { 0.11f, 0.23f, 0.36f, 0.50f, 0.64f, 0.77f, 0.91f}, //4000形更新車
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //50000形
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //3300形VVVF
            new float[] { 0.11f, 0.22f, 0.33f, 0.45f, 0.56f, 0.67f, 0.78f, 0.89f}, //3000形
            new float[] { 0.11f, 0.22f, 0.33f, 0.45f, 0.56f, 0.67f, 0.78f, 0.89f}, //3020形
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //4600形
            new float[] { 0.18f, 0.33f, 0.49f, 0.61f, 0.75f, 0.89f}, //5600形
        };

        /// <summary>
        /// 車両形式
        /// </summary>
        public TrainModel trainModel = TrainModel.None;

        /// <summary>
        /// 車両形式
        /// </summary>
        public enum TrainModel : int
        {
            None = 0,
            series5320 = 1,
            series5300 = 2,
            series4300 = 3,
            Car4321F = 4,
            series4000 = 5,
            Car4000R = 6,
            series50000 = 7,
            Car3300V = 8,
            series3000 = 9,
            series3020 = 10,
            series4600 = 11,
            series5600 = 12,
        }

        /// <summary>
        /// Xml 最高速度情報
        /// </summary>
        private List<MaxSpeedClass> MaxSpeedList;

        /// <summary>
        /// Xml 勾配情報
        /// </summary>
        private List<GradientClass> GradientList;

        /// <summary>
        /// Xml 速度制限情報
        /// </summary>
        private List<SpeedLimitClass> SpeedLimitList;

        /// <summary>
        /// Xml 停止位置オフセット情報
        /// </summary>
        private List<StopPositionOffsetClass> StopPositionOffsetList;

        /// <summary>
        /// データクラス
        /// </summary>
        private readonly Data data = new Data();

        /// <summary>
        /// コンストラクタ
        /// </summary>
        public TASC()
        {
            //Xmlファイル読み込み
            GradientList = LoadXmlData(@"Xml\Gradient.xml", element => new GradientClass
            {
                Direction = element.Element("Direction").Value,
                StationName = element.Element("StationName").Value,
                Distance = float.Parse(element.Element("Distance").Value),
                Gradient = float.Parse(element.Element("Gradient").Value)
            });

            MaxSpeedList = LoadXmlData(@"Xml\MaxSpeed.xml", element => new MaxSpeedClass
            {
                Direction = element.Element("Direction").Value,
                StationName = element.Element("StationName").Value,
                Class = element.Element("Class").Value,
                StartPos = float.Parse(element.Element("StartPos").Value),
                EndPos = float.Parse(element.Element("EndPos").Value),
                MaxSpeed = float.Parse(element.Element("MaxSpeed").Value)
            });

            SpeedLimitList = LoadXmlData(@"Xml\SpeedLimit.xml", element => new SpeedLimitClass
            {
                Direction = element.Element("Direction").Value,
                StartPos = float.Parse(element.Element("StartPos").Value),
                EndPos = float.Parse(element.Element("EndPos").Value),
                Limit = float.Parse(element.Element("Limit").Value),
                BackStopPosName = element.Element("BackStopPosName").Value,
                NextStopPosName = element.Element("NextStopPosName").Value
            });

            StopPositionOffsetList = LoadXmlData(@"Xml\StopPositionOffset.xml", element => new StopPositionOffsetClass
            {
                Direction = element.Element("Direction").Value,
                StationName = element.Element("StationName").Value,
                Offset = new List<float>
                {
                    float.Parse(element.Element("Offset1").Value),
                    float.Parse(element.Element("Offset2").Value),
                    float.Parse(element.Element("Offset3").Value),
                    float.Parse(element.Element("Offset4").Value),
                    float.Parse(element.Element("Offset5").Value),
                    float.Parse(element.Element("Offset6").Value)
                }
            });

            //変数初期化
            sTASCPhase = "停車";
            IsTASCEnable = true;
            IsTASCSpeedControlEnable = true;
            IsTASCOperation = false;
            IsTASCBraking = false;
            IsTASCStoppedStation = true;
            fTASCPatternSpeed = 120.0f;
            fTASCLimitPatternSpeed = 120.0f;
            fTASCDeceleration = 0.0f;
            fTASCXmlLimitSpeed = 0.0f;
            fTASCXmlLimitDistance = 0.0f;
            iTASCNotch = 0;
            fTASCSAPPressure = 0.0f;
            strTargetLimitSpeed = 0.0f;
            strTargetLimitDistance = 0.0f;
            fStopPositionOffset = 0.0f;
            trainModel = TrainModel.None;
            IsTwoHandle = false;
            IsSMEEBrake = false;
            IsSAPReset = false;
        }

        /// <summary>
        /// XmlData読み込みメソッド
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="filePath"></param>
        /// <param name="selector"></param>
        /// <returns></returns>
        public static List<T> LoadXmlData<T>(string filePath, Func<XElement, T> selector)
        {
            try
            {
                return XElement.Load(filePath).Elements().Select(selector).ToList();
            }
            catch
            {
                return new List<T>();
            }
        }

        /// <summary>
        /// TASC 演算メソッド
        /// </summary>
        /// <param name="state">列車の状態</param>
        /// <param name="signal">信号機状態</param>
        public void TASC_Update(TrainState state, string signalName)
        {
            float speed = state.Speed;
            float remainigDistance = state.nextStaDistance;
            string stopType = state.nextStopType;
            float dist = remainigDistance > 0.0f ? remainigDistance : 0.0f;

            //車両形式判定
            switch (state.CarStates[0].CarModel)
            {
                case "5320":
                    trainModel = TrainModel.series5320;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
                case "5300":
                    trainModel = TrainModel.series5300;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
                case "4300":
                    trainModel = TrainModel.series4300;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
                case "4000":
                    trainModel = TrainModel.series4000;
                    IsTwoHandle = true;
                    IsSMEEBrake = false;
                    break;
                case "50000":
                    trainModel = TrainModel.series50000;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
                case "4321":
                    trainModel = TrainModel.Car4321F;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
                case "4000R":
                    trainModel = TrainModel.Car4000R;
                    IsTwoHandle = true;
                    IsSMEEBrake = false;
                    break;
                case "3300V":
                    trainModel = TrainModel.Car3300V;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
                case "3000":
                    trainModel = TrainModel.series3000;
                    IsTwoHandle = true;
                    IsSMEEBrake = true;
                    break;
                case "3020":
                    trainModel = TrainModel.series3020;
                    IsTwoHandle = true;
                    IsSMEEBrake = true;
                    break;
                case "4600":
                    trainModel = TrainModel.series4600;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
                case "5600":
                    trainModel = TrainModel.series5600;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
                default:
                    trainModel = TrainModel.series5320;
                    IsTwoHandle = false;
                    IsSMEEBrake = false;
                    break;
            }

            //TASC有効判定
            if (!IsTASCEnable)
            {
                fTASCPatternSpeed = 120.0f;
                fTASCLimitPatternSpeed = 120.0f;
                fTASCDeceleration = 0.0f;
                fTASCXmlLimitSpeed = 0.0f;
                fTASCXmlLimitDistance = 0.0f;
                fTASCGradientAverage = 0.0f;
                fTASCStoppingPattern = 0.0f;
                fTASCStoppingReductionPattern = 0.0f;
                fTASCStandbyBreakingDistance = 700.0f;
                strTargetLimitSpeed = 0.0f;
                strTargetLimitDistance = 0.0f;
                sTASCPhase = "解除";
                IsTASCOperation = false;
                IsTASCBraking = false;
                IsTASCStoppedStation = false;
                IsSAPReset = false;
                return;
            }

            //停止位置オフセット取得
            fStopPositionOffset = GetStopPositionOffset(state);

            //TASCパターンモード設定
            float fTASCPatternOffset = (sTASCPatternMode == "高速") ? 0.4f : (sTASCPatternMode == "低速") ? -0.5f : 0.0f;

            //TASC制動待機距離演算
            fTASCStandbyBreakingDistance = 1000.0f;

            //TASC駅停車判定(停止位置範囲内かつ速度が0km/h、ドア開(乗降駅のみ)になったら停車判定)
            if (state.stationList[state.nowStaIndex].stopType == StopType.StopForPassenger && Math.Abs(remainigDistance) <= fStopRange && speed.IsZero() && !state.AllClose)
                IsTASCStoppedStation = true;
            else if (state.stationList[state.nowStaIndex].stopType == StopType.StopForOperation && Math.Abs(remainigDistance) <= fStopRange && speed.IsZero())
                IsTASCStoppedStation = true;
            else
                IsTASCStoppedStation = false;

            //TASC制限速度取得
            GetTASCLimitSpeed(state, dist, fStopPositionOffset, out float xmlLimitSpeed, out float xmlLimitDistance);
            if ((state.nextSpeedLimit >= 0.0f) && state.nextSpeedLimit <= xmlLimitSpeed)
            {
                fTASCXmlLimitSpeed = state.nextSpeedLimit;
                fTASCXmlLimitDistance = state.nextSpeedLimitDistance;
            }
            else
            {
                fTASCXmlLimitSpeed = xmlLimitSpeed;
                fTASCXmlLimitDistance = xmlLimitDistance;
            }

            //TASC残距離取得用
            int tascFixedPointDistance = 1000;
            if (dist < 50) tascFixedPointDistance = 50;
            else if (dist < 100) tascFixedPointDistance = 100;
            else if (dist < 150) tascFixedPointDistance = 150;
            else if (dist < 200) tascFixedPointDistance = 200;
            else if (dist < 250) tascFixedPointDistance = 250;
            else if (dist < 300) tascFixedPointDistance = 300;
            else if (dist < 400) tascFixedPointDistance = 400;
            else if (dist < 500) tascFixedPointDistance = 500;
            else if (dist < 600) tascFixedPointDistance = 600;
            else if (dist < 1000) tascFixedPointDistance = 1000;

            //TASC勾配平均値演算
            if (IsTASCEnable)
                fTASCGradientAverage = CalcAverageGradientToAbsolutePosition(state, tascFixedPointDistance, 0.0f, fStopPositionOffset);
            else if (state.nextSpeedLimit >= 0.0f)
                fTASCGradientAverage = CalcAverageGradientToAbsolutePosition(state, dist, dist - fTASCXmlLimitDistance, fStopPositionOffset);
            else
                fTASCGradientAverage = CalcAverageGradientToRelativePosition(state, dist, 1000.0f, fStopPositionOffset);

            //TASC速度制限パターン演算
            if (fTASCXmlLimitSpeed < state.speedLimit)
            {
                strTargetLimitSpeed = fTASCXmlLimitSpeed;

                //出発信号機以外の停止信号ならR0標識手前を目標にする
                float calcTargetLimitDistance = (fTASCXmlLimitDistance > 0.0f) ? fTASCXmlLimitDistance : 0.0f;
                if (strTargetLimitSpeed.IsZero() && !signalName.Contains("出発"))
                    strTargetLimitDistance = ((calcTargetLimitDistance - 15.0f) > 0.0f) ? (calcTargetLimitDistance - 15.0f) : 0.0f;
                //出発信号機の停止信号なら信号機建植位置を目標にする
                else if (strTargetLimitSpeed.IsZero() && signalName.Contains("出発"))
                    strTargetLimitDistance = calcTargetLimitDistance;
                //それ以外は10m手前を目標にする
                else
                    strTargetLimitDistance = calcTargetLimitDistance - 10.0f;

                //速度制限パターン演算
                float calcLimitSpeedPattern = CalcTASCLimitSpeedPattern(strTargetLimitSpeed, strTargetLimitDistance, fTASCLimitSpeedDeceleration + fTASCPatternOffset);
                //最も低い制限速度を選択
                if (calcLimitSpeedPattern > state.speedLimit)
                    fTASCLimitPatternSpeed = state.speedLimit;
                else if (calcLimitSpeedPattern > strTargetLimitSpeed)
                    fTASCLimitPatternSpeed = calcLimitSpeedPattern;
                else
                    fTASCLimitPatternSpeed = strTargetLimitSpeed;
            }
            else
            {
                strTargetLimitSpeed = state.speedLimit;
                strTargetLimitDistance = 0.0f;

                fTASCLimitPatternSpeed = strTargetLimitSpeed;
            }

            //TASC停車パターン演算
            if (stopType.Contains("停車") && remainigDistance < fTASCStandbyBreakingDistance)
            {
                //停車パターン演算
                fTASCStoppingPattern = CalcTASCStoppingPattern(dist, fTASCStoppingDeceleration + fTASCPatternOffset);
                //停車軽減パターン演算
                if (sTASCPatternMode == "高速")
                    fTASCStoppingReductionPattern = CalcTASCStoppingReductionPattern(dist, fTASCStoppingReductionDeceleration + fTASCPatternOffset);
                else
                    fTASCStoppingReductionPattern = CalcTASCStoppingReductionPattern(dist, fTASCStoppingReductionDeceleration);
                //停車軽減パターン移行判定
                if (fTASCStoppingPattern > fTASCStoppingReductionPattern)
                    fTASCPatternSpeed = fTASCStoppingPattern;
                else
                    fTASCPatternSpeed = fTASCStoppingReductionPattern;
            }
            else
            {
                fTASCPatternSpeed = 120.0f;
                fTASCDeceleration = 0.0f;
            }
        }

        /// <summary>
        /// TASC 停車パターン演算メソッド
        /// </summary>
        /// <param name="distance">停止位置までの残り距離[m]</param>
        /// <param name="deceleration">パターン減速度[km/h/s]</param>
        /// <returns></returns>
        public float CalcTASCStoppingPattern(float distance, float deceleration)
        {
            float dist = distance - fTASCDistanceOffset;
            float dec = deceleration;
            float time = freeRunningTime[(int)trainModel];
            if (dist < 0.0f) dist = 0.0f;
            if (!fTASCGradientAverage.IsZero()) dec += (fTASCGradientAverage / iGradientCoefficient);

            //停車パターン演算
            float v = (-2.0f * dec * time + (float)Math.Sqrt((float)Math.Pow(2.0f * dec * time, 2) - 4 * (-7.2 * dec * dist))) / 2;

            return v;
        }

        /// <summary>
        /// TASC 停車軽減パターン演算メソッド
        /// </summary>
        /// <param name="distance">停止位置までの残り距離[m]</param>
        /// <param name="deceleration">パターン減速度[km/h/s]</param>
        /// <returns></returns>
        public float CalcTASCStoppingReductionPattern(float distance, float deceleration)
        {
            float dist = distance;
            float dec = deceleration;
            float time = freeRunningTime[(int)trainModel];
            if (dist < 0.0f) dist = 0.0f;
            if (!fTASCGradientAverage.IsZero()) dec += (fTASCGradientAverage / iGradientCoefficient);

            //軽減パターン演算
            float v = (-2.0f * dec * time + (float)Math.Sqrt((float)Math.Pow(2.0f * dec * time, 2) - 4 * (-7.2 * dec * dist))) / 2;

            return v;
        }

        /// <summary>
        /// TASC 速度制限パターン演算メソッド
        /// </summary>
        /// <param name="limitSpeed">制限速度[km/h]</param>
        /// <param name="distance">制限速度までの残り距離[m]</param>
        /// <param name="deceleration">パターン減速度[km/h/s]</param>
        /// <returns></returns>
        private float CalcTASCLimitSpeedPattern(float limitSpeed, float distance, float deceleration)
        {
            float dist = distance - fTASCDistanceOffset;
            float dec = deceleration;
            float time = freeRunningTime[(int)trainModel];
            if (dist < 0.0f) dist = 0.0f;
            if (!fTASCGradientAverage.IsZero()) dec += (fTASCGradientAverage / iGradientCoefficient);

            //速度制限パターン演算
            float v = -time * dec + (float)Math.Sqrt((float)Math.Pow(time, 2) * (float)Math.Pow(dec, 2) + (float)Math.Pow(limitSpeed, 2) + 7.2f * dec * dist);
            
            if (v < limitSpeed) v = limitSpeed;

            return v;
        }

        /// <summary>
        /// 現在位置から相対位置までの区間における勾配平均値を計算するメソッド
        /// </summary>
        /// <param name="_state">列車の状態</param>
        /// <param name="distance">現在位置の距離[m]</param>
        /// <param name="targetDistance">目標位置までの相対距離[m]</param>
        /// <param name="offset">距離オフセット[m]</param>
        /// <returns></returns>
        private float CalcAverageGradientToRelativePosition(TrainState _state, float distance, float targetDistance, float offset)
        {
            string direction = data.IsEven(int.Parse(Regex.Replace(_state.diaName, @"[^0-9]", ""))) ? "上り" : "下り";
            float average = 0.0f;
            float startDist = Math.Max(distance, 0.0f);
            float endDist = Math.Max(distance - targetDistance, 0.0f);

            // 開始距離が終了距離を超えている場合、入れ替え
            if (startDist > endDist) (endDist, startDist) = (startDist, endDist);
            try
            {
                var gradientsInRange = GradientList
                    .Where(s => s.Direction == direction)
                    .Where(s => s.StationName == _state.nextStaName)
                    .Where(s => s.Distance - offset >= startDist && s.Distance - offset <= endDist)
                    .Select(s => s.Gradient);

                // 一致したデータがあれば平均を計算
                if (gradientsInRange.Any()) average = gradientsInRange.Average();
            }
            catch
            {
                return average;
            }
            return average;
        }

        /// <summary>
        /// 現在位置から絶対位置までの区間における勾配平均値を計算するメソッド
        /// </summary>
        /// <param name="_state">列車の状態</param>
        /// <param name="distance">現在位置の距離[m]</param>
        /// <param name="targetDistance">目標の絶対距離[m]</param>
        /// <param name="offset">距離オフセット[m]</param>
        /// <returns>指定区間の勾配平均値</returns>
        private float CalcAverageGradientToAbsolutePosition(TrainState _state, float distance, float targetDistance, float offset)
        {
            string direction = data.IsEven(int.Parse(Regex.Replace(_state.diaName, @"[^0-9]", ""))) ? "上り" : "下り";
            float average = 0.0f;
            float startDist = Math.Max(distance, 0.0f);
            float endDist = targetDistance;

            try
            {
                var gradientsInRange = GradientList
                    .Where(s => s.Direction == direction)
                    .Where(s => s.StationName == _state.nextStaName)
                    .Where(s => s.Distance - offset >= endDist && s.Distance - offset <= startDist)
                    .Select(s => s.Gradient);

                // 一致したデータがあれば平均を計算
                if (gradientsInRange.Any()) average = gradientsInRange.Average();
            }
            catch
            {
                return average;
            }
            return average;
        }

        /// <summary>
        /// TASC 制限速度取得メソッド
        /// </summary>
        /// <param name="_state">列車の状態</param>
        /// <param name="distance">距離[m]</param>
        /// <param name="offset">距離オフセット[m]</param>
        /// <param name="limitSpeed">制限速度[km/h]</param>
        /// <param name="limitDistance">制限速度までの距離[m]</param>
        public void GetTASCLimitSpeed(TrainState _state, float distance, float offset, out float limitSpeed, out float limitDistance)
        {
            string direction = data.IsEven(int.Parse(Regex.Replace(_state.diaName, @"[^0-9]", ""))) ? "上り" : "下り";
            int backStaIndex = (_state.nowStaIndex - 1 < 0) ? 0 : _state.nowStaIndex - 1;
            int nowStaIndex = _state.nowStaIndex;
            float strSystemSpeedLimit = (_state.nextSpeedLimit < 0.0f) ? _state.speedLimit : _state.nextSpeedLimit;
            float strSystemSpeedLimitDistance = (_state.nextSpeedLimit < 0.0f) ? 0.0f : _state.nextSpeedLimitDistance;
            float dist = (distance < 0.0f) ? 0.0f : distance;
            float strlimitSpeed = 120.0f;
            float strlimitDistance = 0.0f;
            try
            {
                var str = SpeedLimitList
                    .Where(s => s.Direction == direction)
                    .Where(s => s.BackStopPosName == _state.stationList[backStaIndex].StopPosName || s.NextStopPosName == _state.stationList[nowStaIndex].StopPosName)
                    .Where(s => (s.StartPos - offset) > dist && dist >= (s.EndPos - offset))
                    .FirstOrDefault();

                //一致したデータがあれば取得
                if (str != null)
                {
                    strlimitSpeed = str.Limit;
                    strlimitDistance = (str.EndPos - offset) > 0.0f ? (str.EndPos - offset) : 0.0f;
                }

                //最も低い制限速度を選択
                if (strlimitSpeed < strSystemSpeedLimit)
                {
                    limitSpeed = strlimitSpeed;
                    limitDistance = ((dist - strlimitDistance) > 0.0f) ? (dist - strlimitDistance) : 0.0f;
                }
                else
                {
                    limitSpeed = strSystemSpeedLimit;
                    limitDistance = strSystemSpeedLimitDistance;
                }
            }
            catch
            {
                limitSpeed = strSystemSpeedLimit;
                limitDistance = strSystemSpeedLimitDistance;
            }
        }

        /// <summary>
        /// 停止位置オフセット取得メソッド
        /// </summary>
        /// <param name="_state">列車の状態</param>
        /// <returns></returns>
        private float GetStopPositionOffset(TrainState _state)
        {
            string direction = data.IsEven(int.Parse(Regex.Replace(_state.diaName, @"[^0-9]", ""))) ? "上り" : "下り";
            float offset = 0.0f;
            try
            {
                var str = StopPositionOffsetList
                    .Where(s => s.Direction == direction)
                    .Where(s => s.StationName == _state.nextStaName)
                    .Select(s => s.Offset[_state.CarStates.Count - 1]);

                //一致したデータがあれば取得
                if (str != null && str.Any() && _state.nextStopType.Contains("停車"))
                    offset = str.FirstOrDefault();
            }
            catch
            {
                return offset;
            }
            return offset;
        }
    }
}

/// <summary>
/// 勾配情報クラス
/// </summary>
public class GradientClass
{
    /// <summary>
    /// 上下
    /// </summary>
    public string Direction { get; set; }

    /// <summary>
    /// 駅名
    /// </summary>
    public string StationName { get; set; }

    /// <summary>
    /// 残り距離(m)
    /// </summary>
    public float Distance { get; set; }

    /// <summary>
    /// 勾配値(‰)
    /// </summary>
    public float Gradient { get; set; }
}

/// <summary>
/// 最高速度情報クラス
/// </summary>
public class MaxSpeedClass
{
    /// <summary>
    /// 上下
    /// </summary>
    public string Direction { get; set; }

    /// <summary>
    /// 駅名
    /// </summary>
    public string StationName { get; set; }

    /// <summary>
    /// 種別名
    /// </summary>
    public string Class { get; set; }

    /// <summary>
    /// 開始位置(m)
    /// </summary>
    public float StartPos { get; set; }

    /// <summary>
    /// 終了位置(m)
    /// </summary>
    public float EndPos { get; set; }

    /// <summary>
    /// 最高速度(km/h)
    /// </summary>
    public float MaxSpeed { get; set; }
}

/// <summary>
/// 速度制限クラス
/// </summary>
public class SpeedLimitClass
{
    /// <summary>
    /// 上下
    /// </summary>
    public string Direction { get; set; }

    /// <summary>
    /// 開始位置(m)
    /// </summary>
    public float StartPos { get; set; }

    /// <summary>
    /// 終了位置(m)
    /// </summary>
    public float EndPos { get; set; }

    /// <summary>
    /// 制限速度(km/h)
    /// </summary>
    public float Limit { get; set; }

    /// <summary>
    /// 前の停止位置名
    /// </summary>
    public string BackStopPosName { get; set; }

    /// <summary>
    /// 次の停止位置名
    /// </summary>
    public string NextStopPosName { get; set; }
}

/// <summary>
/// 停止位置オフセットクラス
/// </summary>
public class StopPositionOffsetClass
{
    /// <summary>
    /// 上下
    /// </summary>
    public string Direction { get; set; }

    /// <summary>
    /// 駅名
    /// </summary>
    public string StationName { get; set; }

    /// <summary>
    /// オフセット[m]
    /// </summary>
    public List<float> Offset { get; set; }
}
