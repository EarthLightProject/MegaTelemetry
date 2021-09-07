# MegaTelemetry
通信用ArduinoMegaのプログラム

メッセージフォーマット
- 気温(℃),湿度(%),気圧(hPa),緯度(deg),経度(deg),高度(m)
型　int16_t,uint16_t,uint16_t,long,long,long
演算　×100,×100,×100,×10000000,×10000000,×1000
リトルエンディアン方式