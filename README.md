# wili_io

未完成

## WiLIについて
なくしもの捜索用ROS2パッケージ群です。<br>
名前はWhere is a Lost Itemの頭文字から取りました。

## 概要
WiLIのパッケージです。以下の機能を含みます。
- WiLI-外部アプリケーション間をHTTPで通信
- WiLIの諸パラメータをデータベースで管理

データベースソフトウェアにはSQLite3を利用しています。<br>
言語はPythonです。<br>
[wili_db](https://github.com/MaruKazeMaru/wili_db)から派生しました。

## 依存Pythonパッケージ
* NumPy

## ノード
### db_proxy

### http_bridge
HTTPサーバです。5000番ポートをリッスンします。現状ポート番号はハードコーディングです。db_proxyが起動していない場合は動作しません。<br>
リクエストのURLに応じて内部パラメータまたはなくしものの位置推定結果をJSON形式で返します。<br>
URLを以下に挙げます。
#### /motion_num
隠れマルコフモデルの状態数を返します。<br>
例えば状態数が5の場合、レスポンスは下記のようになります。
```json
{ "motion_num": 5 }
```

#### /tr_prob
隠れマルコフモデルの状態数と遷移確率を返します。<br>
例えば状態数が1、遷移行列が
```
[[ 0.7 , 0.3 ],
 [ 0.2 , 0.8 ]]
```
の場合、レスポンスは下記のようになります。
```json
{ "motion_num": 2, "tr_prob": [[0.7,0.3],[0.2,0.8]] }
```

#### /heatmap
隠れマルコフモデルの状態数と各状態のガウス分布の平均、共分散行列を返します。<br>
ガウス分布は2変数であり共分散行列は独立変数が3つだけなので、2次元配列ではなく$ \sigma_{x x} , \sigma_{x y} , \sigma_{y y} $のように上（下）三角成分のみを並べた1次元配列として取り扱います。<br>
例えば状態数が1、各状態の平均が
```
[  1.2 , 0.3 ]
```
共分散行列が
```
[[ 2.3 ,  0.6 ],
 [ 0.6 , -1.8 ]]
```
の場合、レスポンスは下記のようになります。
```json
{ "motion_num": 1, "avr": [1.2,0.3], "var": [2.3,0.6,-1.8] }
```

## launch
* test.launch.py
  * 本パッケージのdb_proxyノードとhttp_bridgeノード、[wili_suggesterパッケージ](https://github.com/MaruKazeMaru/wili_suggester)のsuggesterノードを起動します。

## ライセンス
MITライセンスです。<br>
[LICENSE](LICENSE)をお読みください。
