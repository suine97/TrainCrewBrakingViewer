using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Threading;
using ScottPlot;
using ScottPlot.Plottables;
using TrainCrew;
using Color = ScottPlot.Color;

namespace TrainCrewBrakingViewer;

/// <summary>
/// MainWindow.xaml の相互作用ロジック
/// </summary>
public partial class MainWindow : Window
{
    /// <summary>
    /// 減速曲線の最大本数(TASC.constDeceleration の最大要素数)
    /// </summary>
    private const int MaxNotchCount = 8;

    private readonly Plot _plot;
    private readonly TASC _tasc;
    private readonly ViewerSetting _setting = ViewerSetting.Load();
    private readonly BrakingCurveRepository _curveRepository = new BrakingCurveRepository();
    private readonly DispatcherTimer _timer;

    /// <summary>
    /// 減速曲線のX座標(全曲線で共有)
    /// </summary>
    private readonly double[] _curveXs;

    /// <summary>
    /// 減速曲線のY座標(曲線ごと)
    /// </summary>
    private readonly double[][] _curveYs = new double[MaxNotchCount][];

    /// <summary>
    /// 減速曲線のプロット
    /// </summary>
    private readonly Scatter[] _curves = new Scatter[MaxNotchCount];

    private readonly double[] _speedPointXs = { 0.0 };
    private readonly double[] _speedPointYs = { 0.0 };
    private readonly double[] _speedLineXs = { 0.0, 0.0 };
    private readonly double[] _speedLineYs = { 0.0, 0.0 };

    /// <summary>
    /// 現在の速度を示す点
    /// </summary>
    private Scatter _speedPoint;

    /// <summary>
    /// 現在位置を示す縦線
    /// </summary>
    private Scatter _speedLine;

    /// <summary>
    /// 停目線
    /// </summary>
    private VerticalLine _stopPositionLine;

    /// <summary>
    /// 減速曲線の凡例テキスト
    /// </summary>
    private readonly string[] _legendTexts = new string[MaxNotchCount];

    /// <summary>
    /// 凡例を組んだときの車両形式
    /// </summary>
    private TASC.TrainModel _legendTrainModel = TASC.TrainModel.None;

    /// <summary>
    /// 凡例を組んだときのブレーキ方式
    /// </summary>
    private bool _legendIsSMEEBrake;

    /// <summary>
    /// 凡例を一度でも組んだか
    /// </summary>
    private bool _legendInitialized;

    /// <summary>
    /// 直前の更新で空のグラフを描画済みか
    /// </summary>
    private bool _blankRendered;

    /// <summary>
    /// コンストラクタ
    /// </summary>
    public MainWindow()
    {
        InitializeComponent();

        // 透過設定(ウィンドウハンドル生成前に確定させる必要がある)
        ApplyWindowTransparency();

        Topmost = true;
        _tasc = new TASC();

        // 描画に使う配列を確保する(以降は中身を書き換えて使い回す)
        _curveXs = new double[_setting.CurveSampleCount];
        for (var i = 0; i < MaxNotchCount; i++)
        {
            _curveYs[i] = new double[_setting.CurveSampleCount];
        }

        _timer = new DispatcherTimer(DispatcherPriority.Render)
        {
            Interval = TimeSpan.FromMilliseconds(_setting.UpdateIntervalMs)
        };

        // ScottPlotの初期設定
        _plot = WpfPlot1.Plot;
        _plot.FigureBackground.Color = new Color(0, 0, 0, 0);
        _plot.Axes.Color(Color.FromHex("#FFFFFF"));
        // 凡例の位置を変更
        _plot.Legend.Alignment = Alignment.LowerLeft;

        _plot.Axes.Left.Label.FontName = "BIZ UDゴシック";
        _plot.Axes.Left.Label.ForeColor = Color.FromHex("#DCDCDC");
        _plot.Axes.Left.Label.FontSize = 12;
        _plot.Axes.Left.Label.OffsetY = -2;
        _plot.Axes.Left.Label.Text = "速度 [km/h]";

        _plot.Axes.Bottom.Label.FontName = "BIZ UDゴシック";
        _plot.Axes.Bottom.Label.ForeColor = Color.FromHex("#DCDCDC");
        _plot.Axes.Bottom.Label.FontSize = 12;
        _plot.Axes.Bottom.Label.OffsetY = -2;
        _plot.Axes.Bottom.Label.Text = "停止位置までの距離 [m]";

        // 描画要素を一度だけ生成する(毎フレームの生成・破棄を避ける)
        InitializePlottables();

        // TrainCrewInputの初期化
        TrainCrewInput.Init();

        // ウィンドウをドラッグできるようにする
        MouseLeftButtonDown += (sender, e) =>
        {
            e.Handled = true; DragMove();
        };

        // Escキーで閉じる
        KeyDown += (sender, e) =>
        {
            if (e.Key == Key.Escape)
            {
                var result = MessageBox.Show("グラフを閉じますか？", "確認", MessageBoxButton.YesNo, MessageBoxImage.Question);
                if (result == MessageBoxResult.Yes)
                {
                    _timer.Stop();
                    TrainCrewInput.Dispose();
                    Close();
                }
            }
        };

        Closed += (_, _) => _timer.Stop();

        // 一定間隔で更新する
        // CompositionTarget.Rendering はハンドラを登録している間ウィンドウ全体の再合成を
        // 毎フレーム走らせ続けるため、変化が無くても負荷がウィンドウ面積に比例して掛かる。
        _timer.Tick += (_, _) =>
        {
            try
            {
                Update();
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"{ex}");
            }
        };
        _timer.Start();
    }

    /// <summary>
    /// ウィンドウ透過設定の適用メソッド
    /// </summary>
    private void ApplyWindowTransparency()
    {
        AllowsTransparency = _setting.Transparent;

        var background = _setting.Transparent
            ? new SolidColorBrush(System.Windows.Media.Color.FromArgb(30, 255, 255, 255))
            : new SolidColorBrush(System.Windows.Media.Color.FromRgb(16, 16, 16));
        background.Freeze();

        Background = background;
    }

    /// <summary>
    /// 描画要素の生成メソッド
    /// </summary>
    /// <remarks>
    /// 追加順で凡例の配色(パレット)が決まるため、順序は変更しない。
    /// </remarks>
    private void InitializePlottables()
    {
        // 停目線
        _stopPositionLine = _plot.Add.VerticalLine(0.0);
        _stopPositionLine.IsVisible = false;

        // 減速曲線(最大本数ぶん生成し、車両形式に応じて表示・非表示を切り替える)
        for (var i = 0; i < MaxNotchCount; i++)
        {
            _curves[i] = _plot.Add.Scatter(_curveXs, _curveYs[i]);
            // マーカーを消して折れ線だけにする(Add.Function と同じ見た目にする)
            _curves[i].MarkerSize = 0;
            _curves[i].IsVisible = false;
        }

        // 現在の速度を示す点
        _speedPoint = _plot.Add.Scatter(_speedPointXs, _speedPointYs, color: Color.FromHex("#FFA500"));
        _speedPoint.IsVisible = false;

        // 現在位置を示す縦線
        _speedLine = _plot.Add.Scatter(_speedLineXs, _speedLineYs, color: Color.FromHex("#FFA500"));
        _speedLine.IsVisible = false;
    }

    /// <summary>
    /// 更新メソッド
    /// </summary>
    private void Update()
    {
        // TrainCrew情報取得
        var state = TrainCrewInput.GetTrainState();
        TrainCrewInput.RequestStaData();
        if (state == null || state.CarStates.Count == 0 || state.stationList.Count == 0) { return; }
        if (state.nowStaIndex < 0 || state.nowStaIndex >= state.stationList.Count) { return; }

        //運転画面遷移でなければ描画をクリアする
        if (TrainCrewInput.gameState.gameScreen != GameScreen.MainGame
            && TrainCrewInput.gameState.gameScreen != GameScreen.MainGame_Pause
            && TrainCrewInput.gameState.gameScreen != GameScreen.MainGame_Loading)
        {
            RenderBlank();
            return;
        }

        //信号機情報取得
        var strSignal = TrainCrewInput.signals;
        var signalName = (strSignal.Count > 0) ? strSignal[0].name : "None";

        // TASC演算
        _tasc.TASC_Update(state, signalName);

        double speed = state.Speed;
        var notch = (_tasc.IsTwoHandle) ? Math.Max(state.Bnotch, 1) : Math.Max(state.Bnotch - 1, 0);

        // プロットの描画範囲を先に決める(減速曲線のサンプリング範囲に使うため)
        double maxAxisX = state.nextStaDistance switch
        {
            <= 24 => 25.0,
            < 950 => state.nextStaDistance * 1.05,
            _ => 1000.0,
        };
        double maxAxisY = speed switch
        {
            <= 21 => 25.0,
            < 120 => speed * 1.20,
            _ => 120.0,
        };
        double minAxisX = -(maxAxisX / 5);

        // 停目線を引く
        bool isStopStation = state.nextStopType is "停車" or "運転停車";
        _stopPositionLine.IsVisible = isStopStation;
        if (isStopStation) _stopPositionLine.X = state.nextStaDistance;

        //減速度[km/h/s]に変換するための係数を取得
        float[] constDeceleration = _tasc.constDeceleration[(int)_tasc.trainModel];
        float maxDeceleration = _tasc.maxDeceleration[(int)_tasc.trainModel];
        int notchCount = Math.Min(constDeceleration.Length, MaxNotchCount);

        // 車両形式が変わったときだけ凡例と曲線本数を組み直す
        UpdateCurveLegend(notchCount);

        // 勾配を織り込み済みのブレーキ曲線を要求する(駅・方向・形式が変わったときだけ読み込まれる)
        int trainModelIndex = (int)_tasc.trainModel;
        int directionKey = _tasc.GetDirectionKey(state);
        _curveRepository.Request(state.nextStaName, directionKey, trainModelIndex);

        // 読み込めていればbinのデータを使い、無ければ従来の演算で描く
        var curveSet = _curveRepository.Current;
        bool useBinCurve = curveSet != null && curveSet.Matches(state.nextStaName, directionKey, trainModelIndex);

        // 減速曲線を引く
        // Add.Function はプロット幅のピクセル数だけ関数を評価するためウィンドウ幅に比例して重くなる。
        // 曲線は滑らかな平方根カーブなので、固定点数でサンプリングして折れ線として描く。
        int sampleCount = _curveXs.Length;
        double sampleStep = (maxAxisX - minAxisX) / (sampleCount - 1);
        for (var j = 0; j < sampleCount; j++)
        {
            _curveXs[j] = minAxisX + sampleStep * j;
        }

        for (var i = 0; i < notchCount; i++)
        {
            float dec = constDeceleration[i] * maxDeceleration;
            double[] ys = _curveYs[i];

            // binのノッチキーは TrainState.Bnotch と同じ値
            int bnotch = _tasc.IsTwoHandle ? i + 1 : i + 2;

            for (var j = 0; j < sampleCount; j++)
            {
                var x = (float)_curveXs[j];
                var distance = state.nextStaDistance - x;

                // 停車パターン
                float y1 = 0.0f;
                if (!useBinCurve || !curveSet.TryGetSpeed(bnotch, distance, out y1))
                    y1 = _tasc.CalcTASCStoppingReductionPattern(distance, dec);

                var y2 = _tasc.CalcTASCLimitSpeedPattern(_tasc.strTargetLimitSpeed, _tasc.strTargetLimitDistance - x, dec);

                // NaNは除外して小さい方を採る(Enumerable.Min と同じ扱い)
                float y;
                if (float.IsNaN(y1)) y = y2;
                else if (float.IsNaN(y2)) y = y1;
                else y = Math.Min(y1, y2);

                ys[j] = float.IsNaN(y) ? 0.0 : y;
            }

            _curves[i].LineWidth = notch == i + 1 ? 5 : 2;
        }

        // 現在の速度に点を描画
        _speedPointYs[0] = speed;
        _speedPoint.IsVisible = true;

        // 現在位置に縦線を引く
        _speedLineYs[1] = speed;
        _speedLine.IsVisible = true;

        // プロットの描画範囲を設定
        _plot.Axes.SetLimits(minAxisX, maxAxisX, 0, maxAxisY);
        WpfPlot1.Refresh();
        _blankRendered = false;
    }

    /// <summary>
    /// 凡例・曲線本数の更新メソッド
    /// </summary>
    /// <param name="notchCount">減速曲線の本数</param>
    /// <remarks>
    /// ScottPlotの凡例は描画時に各Plottableの LegendText と IsVisible を読むため、
    /// この2つは毎回設定する。文字列の生成だけを車両形式ごとにキャッシュする。
    /// </remarks>
    private void UpdateCurveLegend(int notchCount)
    {
        // 凡例テキストの生成は車両形式かブレーキ方式が変わったときだけ行う
        if (!_legendInitialized
            || _legendTrainModel != _tasc.trainModel
            || _legendIsSMEEBrake != _tasc.IsSMEEBrake)
        {
            for (var i = 0; i < MaxNotchCount; i++)
            {
                // ブレーキノッチ表示
                _legendTexts[i] = _tasc.IsSMEEBrake ? $"B-{(i + 1) * 50}kPa" : $"B{i + 1}";
            }

            _legendTrainModel = _tasc.trainModel;
            _legendIsSMEEBrake = _tasc.IsSMEEBrake;
            _legendInitialized = true;
        }

        for (var i = 0; i < MaxNotchCount; i++)
        {
            bool isUsed = i < notchCount;
            _curves[i].IsVisible = isUsed;
            _curves[i].LegendText = isUsed ? _legendTexts[i] : string.Empty;
        }
    }

    /// <summary>
    /// 空グラフ描画メソッド
    /// </summary>
    /// <remarks>
    /// 既に空を描画済みなら再描画しない(運転画面外で描き直し続けるのを防ぐ)。
    /// </remarks>
    private void RenderBlank()
    {
        if (_blankRendered) return;

        _stopPositionLine.IsVisible = false;
        _speedPoint.IsVisible = false;
        _speedLine.IsVisible = false;
        for (var i = 0; i < MaxNotchCount; i++)
        {
            _curves[i].IsVisible = false;
            _curves[i].LegendText = string.Empty;
        }

        WpfPlot1.Refresh();
        _blankRendered = true;
    }
}
