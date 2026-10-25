# Laboratuvar 3 – UART, I²C ve SPI

İleri Gömülü Sistemler dersi (Dr. Murat Sürücü), Laboratuvar 3 çalışması.

- **Hazırlayan:** Gamze Şirin
- **Öğrenci numarası:** 2510140022
- **Donanım:** ESP32-S3-N16R8 geliştirme kartı (CPU 240 MHz), GY-521 ivme sensörü modülü
- **Yazılım:** ESP-IDF v6.1, Windows
- **Tarih:** 25.10.2026

Lojik analizör kullanılmadığı için her görevin çıktısı ESP-IDF monitöründen dosyaya kaydedildi (Ctrl+T, Ctrl+L). Kayıtlar `lab3/olcum/` klasöründedir.

## Bağlantılar

| Görev | Bağlantı |
|---|---|
| UART | GPIO17 (UART1 TX) ↔ GPIO18 (UART1 RX) ↔ GPIO16 (UART2 RX), aynı breadboard sütununda |
| SPI | GPIO11 (MOSI) ↔ GPIO13 (MISO); GPIO12 (SCLK) ↔ GPIO14 (SCLK gözlem girişi); CS = GPIO10 |
| I²C | 3V3 → VCC, GND → GND, GPIO8 → SDA, GPIO9 → SCL |

## Görev 1 – UART geri döngü (`lab3/lab3_uart`)

UART1 115200 baud'da numaralı paketler gönderir ve kendi RX'inden geri okur. Gönderme ile alma arasındaki süre `esp_timer` ile ölçülür. Ardından aynı hat UART2'ye (GPIO16) farklı baud hızlarında dinletilir. Alınan baytlar ve çerçeve hatası sayısı yazdırılır.

**Doğru baud:** 6 turda 60 paketin hepsi aynen geri geldi; hata, çerçeve hatası ve taşma yok. Ortalama gecikme 2,83 ms (22 baytlık paketin 115200 baud'da aktarımı yaklaşık 1,9 ms), en büyük 2,99 ms.

**Yanlış baud** (gönderilen: 23 bayt, 115200 baud). Sonuçlar 6 turun hepsinde aynı çıktı:

| Alıcı baud | Hız farkı | Alınan bayt | Çerçeve hatası |
|---|---|---|---|
| 9600 | −%91,6 | 2 | 1 |
| 57600 | −%50,0 | 11 | 8 |
| 105000 | −%8,8 | 18 | 14 |
| 230400 | +%100,0 | 36 | 20 |

Alıcı yavaşsa birden çok bit tek bit gibi örneklenir ve bayt sayısı azalır. Alıcı hızlıysa her bit iki kez örneklenir ve bayt sayısı artar. Yaklaşık %9'luk farkta bile her baytın sonundaki bitler kaydığı için metin tamamen bozuldu.

## Görev 2 – I²C sensör (`lab3/lab3_i2c`)

Program önce 0x00–0x7F adreslerini tarar, sonra sensörün WHO_AM_I registerini okur. 0x69 adresine bir yoklama ile NACK örneği gösterir. Sensörü ±4 g aralığına ayarlar ve `esp_timer` ile saniyede 10 kez ivme okur. I²C hızı 100 kHz'dir.

- **Tarama:** Yalnızca **0x68** adresinde cihaz bulundu. 0x69 yoklaması NACK aldı.
- **WHO_AM_I = 0x70.** MPU-6050'nin değeri 0x68'dir. 0x70, modülde uyumlu bir sensörün (MPU-6500) bulunduğunu gösterir. Register haritası ivme okuması için aynı olduğundan ölçüm sorunsuz çalıştı.
- **ACCEL_CONFIG** geri okundu: 0x08 (±4 g).
- **Örnekleme:** 10,00 Hz, 279 örnekte okuma hatası 0.
- **İvme:** |a| ortalaması 0,974 g (en küçük 0,965, en büyük 0,985 g). ax ≈ −0,42 g ve az ≈ +0,88 g, modülün yaklaşık 26° eğik durduğunu gösteriyor (pinler lehimsiz olduğu için modül yana bastırılarak tutuldu).

## Görev 3 – SPI geri döngü (`lab3/lab3_spi`)

Her SPI modunda (0–3) 1 MHz'de iki farklı 4 baytlık desen gönderilir: DE AD BE EF ve artan sayaç. Alınan baytlar bayt ve bit düzeyinde karşılaştırılır. SCLK'nin boştaki düzeyi GPIO14'ten okunup CPOL ile karşılaştırılır. Ardından mod 0'da 1, 8, 20 ve 40 MHz'de 100'er işlem yapılır.

**Mod testi:** 6 turun hepsinde 4 modda da gönderilen = alınan, bayt ve bit hatası 0. SCLK boşta düzeyi mod 0–1'de 0, mod 2–3'te 1 ölçüldü; CPOL ile uyumlu.

**Hız taraması:**

| İstenen | Gerçek | Hatalı işlem | İşlem başına süre |
|---|---|---|---|
| 1 MHz | 1,000 MHz | 0 / 100 | 49,0 µs |
| 8 MHz | 8,000 MHz | 0 / 100 | 18,9 µs |
| 20 MHz | 20,000 MHz | 0 / 100 | 16,3 µs |
| 40 MHz | 40,000 MHz | 0 / 100 | 15,4 µs |

32 bitlik aktarım 1 MHz'de 32 µs, 40 MHz'de 0,8 µs sürer. Ölçülen süreden çıkarınca her işlemde yaklaşık 15 µs'lik sabit bir yazılım/sürücü yükü kalıyor. Yüksek hızlarda toplam süreyi bu yük belirliyor.

## Derleme ve yükleme

```
cd lab3/lab3_uart
idf.py set-target esp32s3
idf.py -p COMx flash monitor
```

Diğer görevler için klasör adını `lab3_i2c` veya `lab3_spi` yapmak yeterli.

---

Not: Kodların yazımında yapay zekâdan yararlanılmıştır.
