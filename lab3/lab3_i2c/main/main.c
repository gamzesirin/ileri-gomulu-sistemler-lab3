/*
 * İleri Gömülü Sistemler - Laboratuvar 3, Görev 2: I2C ve MPU-6050
 * ESP32-S3, ESP-IDF v6.1 (yeni i2c_master sürücüsü)
 *
 * Bağlantı (GY-521 modülü):
 *   VCC -> 3V3   GND -> GND   SDA -> GPIO8   SCL -> GPIO9   AD0 boşta (adres 0x68)
 *
 * Program akışı
 *   1) i2cdetect benzeri tarama: 0x08-0x77 arasındaki her adres yoklanır,
 *      ACK veren adres ızgarada yazılır, vermeyen "--" olarak gösterilir.
 *   2) WHO_AM_I (0x75) okunur. Modülde olmayan 0x69 adresi yoklanarak NACK örneği gösterilir.
 *   3) Sensör uyandırılır, ivme ölçeği ±4 g ve sayısal alçak geçiren süzgeç ayarlanır.
 *   4) esp_timer 100 ms'de bir ölçüm görevine bildirim gönderir (10 Hz).
 *      Görev yalnız ivme registerlarını (0x3B'den 6 bayt) okur ve g cinsinden yazar.
 *      10 saniyede bir |a| için en küçük / en büyük değer ve hata sayısı özetlenir.
 *   Sensör bulunamazsa program çökmez; 5 saniyede bir taramayı tekrarlar.
 */
#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"

#define PIN_SDA             8
#define PIN_SCL             9
#define I2C_HIZ_HZ          100000   /* 100 kHz: lehimsiz/uzun kabloda daha güvenli */
#define ZAMAN_ASIMI_MS      50

#define MPU_ADRES           0x68
#define YOK_ADRES           0x69      /* AD0 = 1 olsaydı kullanılacak adres; modülde yok */
#define REG_CONFIG          0x1A
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

#define ORNEK_PERIYOT_US    100000    /* 10 Hz */
#define OZET_ORNEK_SAYISI   100       /* 10 s */
#define LSB_PER_G_4G        8192.0f   /* ±4 g ölçeğinde 1 g */

static const char *TAG = "i2c_lab";

static i2c_master_bus_handle_t veri_yolu;
static i2c_master_dev_handle_t mpu;
static TaskHandle_t olcum_gorevi_tutamagi;

/* ------------------------------------------------------------------ */
/* Yardımcılar                                                         */
/* ------------------------------------------------------------------ */

/* Tek registere yazma: [register, değer] tek START ... STOP içinde gider */
static esp_err_t reg_yaz(uint8_t reg, uint8_t deger)
{
    const uint8_t paket[2] = { reg, deger };
    return i2c_master_transmit(mpu, paket, sizeof(paket), ZAMAN_ASIMI_MS);
}

/* Register okuma: adres+W, register no, tekrarlı START, adres+R, n bayt */
static esp_err_t reg_oku(uint8_t reg, uint8_t *veri, size_t n)
{
    return i2c_master_transmit_receive(mpu, &reg, 1, veri, n, ZAMAN_ASIMI_MS);
}

/* i2cdetect çıktısına benzeyen ızgara; bulunan adres sayısını döndürür */
static int veri_yolunu_tara(bool *mpu_var)
{
    int bulunan = 0;
    *mpu_var = false;
    printf("\nI2C taraması (SDA=GPIO%d, SCL=GPIO%d)\n", PIN_SDA, PIN_SCL);
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    for (int satir = 0; satir < 0x80; satir += 0x10) {
        printf("%02x: ", satir);
        for (int sutun = 0; sutun < 0x10; sutun++) {
            int adres = satir + sutun;
            if (adres < 0x08 || adres > 0x77) {
                printf("   ");                 /* ayrılmış adresler yoklanmaz */
                continue;
            }
            if (i2c_master_probe(veri_yolu, (uint16_t)adres, ZAMAN_ASIMI_MS) == ESP_OK) {
                printf("%02x ", adres);
                bulunan++;
                if (adres == MPU_ADRES) {
                    *mpu_var = true;
                }
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
    printf("Bulunan cihaz: %d\n", bulunan);
    return bulunan;
}

/* ------------------------------------------------------------------ */
/* 10 Hz örnekleme                                                     */
/* ------------------------------------------------------------------ */

/* esp_timer geri çağırması: yalnız ölçüm görevini uyandırır */
static void ornek_zamani(void *arg)
{
    xTaskNotifyGive(olcum_gorevi_tutamagi);
}

static void olcum_gorevi(void *arg)
{
    uint32_t ornek = 0, hata = 0, toplam_hata = 0;
    float en_kucuk = 1e9f, en_buyuk = 0.0f;
    int64_t pencere_baslangic = esp_timer_get_time();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint8_t ham[6];
        esp_err_t sonuc = reg_oku(REG_ACCEL_XOUT_H, ham, sizeof(ham));
        if (sonuc != ESP_OK) {
            hata++;
            toplam_hata++;
            printf("okuma hatası: %s\n", esp_err_to_name(sonuc));
        } else {
            /* Registerlar büyük uçlu (önce yüksek bayt) 16 bit işaretli sayılardır */
            int16_t x = (int16_t)((ham[0] << 8) | ham[1]);
            int16_t y = (int16_t)((ham[2] << 8) | ham[3]);
            int16_t z = (int16_t)((ham[4] << 8) | ham[5]);
            float ax = x / LSB_PER_G_4G;
            float ay = y / LSB_PER_G_4G;
            float az = z / LSB_PER_G_4G;
            float a = sqrtf(ax * ax + ay * ay + az * az);
            if (a < en_kucuk) en_kucuk = a;
            if (a > en_buyuk) en_buyuk = a;
            printf("t=%7.1f s  ax=%+6.3f  ay=%+6.3f  az=%+6.3f g   |a|=%5.3f g\n",
                   esp_timer_get_time() / 1e6, ax, ay, az, a);
        }

        if (++ornek >= OZET_ORNEK_SAYISI) {
            int64_t simdi = esp_timer_get_time();
            float gercek_hz = ornek * 1e6f / (float)(simdi - pencere_baslangic);
            printf("--- 10 s özeti: %" PRIu32 " örnek (%.2f Hz), |a| en küçük %.3f g, en büyük %.3f g, "
                   "okuma hatası %" PRIu32 " (toplam %" PRIu32 ")\n",
                   ornek, gercek_hz, en_kucuk, en_buyuk, hata, toplam_hata);
            ornek = 0;
            hata = 0;
            en_kucuk = 1e9f;
            en_buyuk = 0.0f;
            pencere_baslangic = simdi;
        }
    }
}

/* ------------------------------------------------------------------ */

void app_main(void)
{
    const i2c_master_bus_config_t yol_ayari = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* GY-521'de dış direnç de var */
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&yol_ayari, &veri_yolu));

    /* 1) Tarama; sensör yoksa çökmeden beklemeye devam et */
    bool mpu_var = false;
    veri_yolunu_tara(&mpu_var);
    while (!mpu_var) {
        printf("MPU-6050 (0x%02X) yanıt vermedi. VCC, GND, SDA=GPIO%d, SCL=GPIO%d bağlantılarını "
               "kontrol edin. Tarama 5 s sonra tekrarlanacak.\n", MPU_ADRES, PIN_SDA, PIN_SCL);
        vTaskDelay(pdMS_TO_TICKS(5000));
        veri_yolunu_tara(&mpu_var);
    }

    const i2c_device_config_t cihaz_ayari = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADRES,
        .scl_speed_hz = I2C_HIZ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(veri_yolu, &cihaz_ayari, &mpu));

    /* 2) Kimlik ve NACK örneği */
    uint8_t kimlik = 0;
    esp_err_t sonuc = reg_oku(REG_WHO_AM_I, &kimlik, 1);
    if (sonuc != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I okunamadı: %s", esp_err_to_name(sonuc));
    } else {
        printf("WHO_AM_I (0x75) = 0x%02X %s\n", kimlik,
               kimlik == 0x68 ? "(MPU-6050 beklenen değer)" : "(beklenen 0x68; benzer bir sensör olabilir)");
    }
    sonuc = i2c_master_probe(veri_yolu, YOK_ADRES, ZAMAN_ASIMI_MS);
    printf("0x%02X adresine yoklama: %s -> %s\n", YOK_ADRES, esp_err_to_name(sonuc),
           sonuc == ESP_OK ? "ACK" : "NACK (adreste cihaz yok)");

    /* 3) Sensör ayarı */
    ESP_ERROR_CHECK(reg_yaz(REG_PWR_MGMT_1, 0x00));     /* uyku modundan çık, iç saat */
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(reg_yaz(REG_ACCEL_CONFIG, 0x08));   /* AFS_SEL = 1 -> ±4 g */
    ESP_ERROR_CHECK(reg_yaz(REG_CONFIG, 0x03));         /* DLPF_CFG = 3 -> ivmede ~44 Hz bant */

    uint8_t geri = 0;
    if (reg_oku(REG_ACCEL_CONFIG, &geri, 1) == ESP_OK) {
        printf("ACCEL_CONFIG geri okundu: 0x%02X (±%d g)\n", geri, 2 << ((geri >> 3) & 0x3));
    }

    /* 4) 10 Hz örnekleme */
    xTaskCreate(olcum_gorevi, "olcum", 4096, NULL, 10, &olcum_gorevi_tutamagi);

    const esp_timer_create_args_t zamanlayici_ayari = {
        .callback = ornek_zamani,
        .name = "ornek_10hz",
    };
    esp_timer_handle_t zamanlayici;
    ESP_ERROR_CHECK(esp_timer_create(&zamanlayici_ayari, &zamanlayici));
    ESP_ERROR_CHECK(esp_timer_start_periodic(zamanlayici, ORNEK_PERIYOT_US));
    printf("Örnekleme başladı: %d ms periyot (10 Hz), ölçek ±4 g\n", ORNEK_PERIYOT_US / 1000);
}
