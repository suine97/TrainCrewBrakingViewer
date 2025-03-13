using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using ScottPlot;
using TrainCrew;
using Color = ScottPlot.Color;

namespace TrainCrewBrakingViewer;

/// <summary>
/// MainWindow.xaml の相互作用ロジック
/// </summary>
public partial class MainWindow : Window
{
    private Plot _plot;
    private readonly TASC _tasc;

    /// <summary>
    /// コンストラクタ
    /// </summary>
    public MainWindow()
    {
        InitializeComponent();

        Topmost = true;
        _tasc = new TASC();

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
                    TrainCrewInput.Dispose();
                    Close();
                }
            }
        };

        // レンダリングイベントで更新
        CompositionTarget.Rendering += (_, _) =>
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
    }

    /// <summary>
    /// 更新メソッド
    /// </summary>
    private void Update()
    {
        // 今ある線を全部クリア
        _plot.Clear();

        // TrainCrew情報取得
        var state = TrainCrewInput.GetTrainState();
        TrainCrewInput.RequestStaData();
        if (state == null || state.CarStates.Count == 0 || state.stationList.Count == 0) { return; }
        try { var dataCheck = state.stationList[state.nowStaIndex].Name; }
        catch { return; }

        double maxAxisX = 1000;
        double speed = state.Speed;
        var notch = (_tasc.IsTwoHandle) ? Math.Max(state.Bnotch, 1) : Math.Max(state.Bnotch - 1, 0);

        //運転画面遷移なら処理
        if (TrainCrewInput.gameState.gameScreen == GameScreen.MainGame
            || TrainCrewInput.gameState.gameScreen == GameScreen.MainGame_Pause
            || TrainCrewInput.gameState.gameScreen == GameScreen.MainGame_Loading)
        {
            //信号機情報取得
            var strSignal = TrainCrewInput.signals;
            var signalName = (strSignal.Count > 0) ? strSignal[0].name : "None";

            // TASC演算
            _tasc.TASC_Update(state, signalName);

            //勾配値算出
            float gradientDec = _tasc.fTASCGradientAverage.IsZero() ? 0.0f : (_tasc.fTASCGradientAverage / _tasc.iGradientCoefficient);

            //減速度[km/h/s]に変換した配列を生成
            float[] strCoinstDeceration = _tasc.constDeceleration[(int)_tasc.trainModel];
            List<float> notchDist = strCoinstDeceration.Select(i => i * _tasc.maxDeceleration[(int)_tasc.trainModel] + gradientDec).ToList();

            // 停目線を引く
            if (state.nextStopType is "停車" or "運転停車")
            {
                _plot.Add.VerticalLine(state.nextStaDistance);
                maxAxisX = state.nextStaDistance;
            }

            // 減速曲線を引く
            for (var i = 0; i < notchDist.Count; i++)
            {
                var dec = notchDist[i];
                var func = new Func<double, double>(x =>
                {
                    var y = _tasc.CalcTASCStoppingReductionPattern(state.nextStaDistance - (float)x, dec);
                    return double.IsNaN(y) ? 0 : y;
                });
                var functionPlot = _plot.Add.Function(func);
                functionPlot.LineWidth = notch == i + 1 ? 5 : 2;
                // ブレーキノッチ表示
                if (_tasc.IsSMEEBrake)
                    functionPlot.LegendText = $"B-{(i + 1) * 50}kPa";
                else
                    functionPlot.LegendText = $"B{i + 1}";
            }

            // 現在の速度に点を描画
            _plot.Add.Scatter(0, speed, color: Color.FromHex("#FFA500"));
            // 現在位置に縦線を引く
            _plot.Add.Scatter(new[] { 0, 0 }, new[] { 0, speed }, color: Color.FromHex("#FFA500"));

            switch (state.nextStaDistance)
            {
                case <= 24:
                    maxAxisX = 25;
                    break;
                case < 950:
                    maxAxisX = state.nextStaDistance * 1.05;
                    break;
                default:
                    maxAxisX = 1000;
                    break;
            }
            double maxAxisY;
            switch (speed)
            {
                case <= 21:
                    maxAxisY = 25;
                    break;
                case < 120:
                    maxAxisY = speed * 1.20;
                    break;
                default:
                    maxAxisY = 120;
                    break;
            }

            // プロットの描画範囲を設定
            var minAxisX = -(maxAxisX / 5);
            _plot.Axes.SetLimits(minAxisX, maxAxisX, 0, maxAxisY);
            WpfPlot1.Refresh();
        }
        else
        {
            // プロットの描画をクリア
            _plot.Clear();
            WpfPlot1.Refresh();
        }
    }
}
