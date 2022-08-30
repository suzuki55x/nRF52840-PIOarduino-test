# nRF52840 Arduino 

## 未対応ボード(ex.ISP1807 breakoutボード)をArduino + PlatformIOで使う

1. ベースに使えそうなボードを見つける

今回は、ブートローダーが(多分)共通のため、Adafruit fetherが使えそうだった。  
書き込みにも成功したが、おそらくピン配をカスタムする必要がある。  

2. ベースボードのvariantファイルを持ってくる。

platformのリポジトリ(nordicnrf52。platformioのドキュメントから飛べる)から、ボード設定のJSONファイルを開く。  
variants欄にvariantsの名前が記載されているので、該当ボードのArduinoプラットフォームのリポジトリに移動し、variantsを取得する。  
ArduinoIDEにすでに対応されているボードについては、それを持ってくるだけでよい


3. プロジェクト内にcustom variantを格納するディレクトリを作成し、platform.iniにて参照設定を行う

本リポジトリのiniファイルを参照。

パッケージ内の`variants`フォルダを強制参照するようなので、そこに格納すればよいらしい。  
NGな場合、 `variants_dir`の設定をiniに記述すればよい

[参考](https://github.com/maxgerhardt/pio-custom-stm32duino-variants)

## Zephyr RTOSで使う場合

https://piolabs.com/blog/engineering/platformio-zephyr-custom-hardware.html#adding-a-custom-board-to-zephyr

