# TrainCrewBrakingViewer

TrainCrewBrakingViewer、溝月レイル/Acty様製作の列車運転ゲーム「TRAIN CREW」で動作する、ブレーキ目安曲線表示ソフトです。
![1](https://github.com/user-attachments/assets/e30f7aaa-f44e-41bc-be62-577225fc9095)


# 使い方
1、TRAIN CREWの操作設定→外部デバイス入出力を「有効」に設定してください。

2、Zipファイルを解凍後、TrainCrewBrakingViewer.exeをダブルクリックして起動してください。

3、グラフの右下をクリックしながら動かす事で拡大・縮小、グラフの任意の場所をクリックしながら動かす事で移動ができます。

4、アプリを終了する場合は「Escape」キーを押してください。

# 動作が重い場合
ウィンドウを大きくすると描画が重くなる場合は、「Xml\Setting.xml」で調整できます。(ファイルが無い場合や記述が不正な場合は既定値で動作します)

|項目|既定値|説明|
|---|---|---|
|Transparent|true|ウィンドウを透過するか。WPFは透過ウィンドウのGPU描画を無効化してウィンドウ全体をCPUで合成するため、描画負荷がウィンドウ面積に比例して増えます。falseにすると背景が不透明な暗色になる代わりに大幅に軽くなります。|
|UpdateIntervalMs|50|グラフの更新間隔[ms]。大きくすると軽くなりますが、表示の追従が遅くなります。|
|CurveSampleCount|256|減速曲線のサンプル点数。減らすと軽くなりますが、曲線が粗くなります。|

# 作成情報
作成者:Suine97

# License
"TrainCrewBrakingViewer" is under MIT license. 
