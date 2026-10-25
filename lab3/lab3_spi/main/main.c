/*
 * İleri Gömülü Sistemler - Laboratuvar 3, Görev 3: SPI geri döngü
 * ESP32-S3, ESP-IDF v6.1, SPI2 (FSPI) ana birim
 *
 * Bağlantı:
 *   GPIO11 (MOSI) ── GPIO13 (MISO)     gönderilen veri geri okunur
 *   GPIO12 (SCLK) ── GPIO14 (giriş)    saatin boşta düzeyi (CPOL) okunur
 *   GPIO10 (CS)   bağlantısız
 *   10-13 numaralı pinler SPI2'nin IO_MUX pinleridir; sinyaller GPIO matrisinden geçmez.
 *
 * Program akışı (10 s'de bir tekrar):
 *   1) Mod 0-3: her modda iki desen (DE AD BE EF ve artan sayaç) 1 MHz'de
 *      gönderilir; alınan veri bayt bayt ve bit bit karşılaştırılır.
 *      İşlemden sonra GPIO14 okunarak SCLK'nin boşta düzeyi CPOL ile karşılaştırılır.
 *   2) Hız taraması (mod 0): 1, 8, 20, 40 MHz'de 100'er işlem yapılır;
 *      hatalı işlem sayısı ve işlem başına ortalama süre yazdırılır.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define SPI_BIRIM           SPI2_HOST
#define PIN_MOSI            11
#define PIN_MISO            13
#define PIN_SCLK            12
#define PIN_CS              10
#define PIN_SCLK_GOZLEM     14
#define BAYT_SAYISI         4
#define MOD_TESTI_HZ        1000000
#define HIZ_TESTI_ISLEM     100
#define TUR_ARASI_MS        10000

static const char *TAG = "spi_lab";

static const int TARAMA_HIZLARI_HZ[] = { 1000000, 8000000, 20000000, 40000000 };
#define TARAMA_HIZ_SAYISI (sizeof(TARAMA_HIZLARI_HZ) / sizeof(TARAMA_HIZLARI_HZ[0]))

static uint8_t sayac_degeri;      /* artan desen için sürekli ilerleyen sayaç */

/* ------------------------------------------------------------------ */

static esp_err_t cihaz_ekle(int mod, int hiz_hz, spi_device_handle_t *cihaz)
{
    const spi_device_interface_config_t ayar = {
        .mode = (uint8_t)mod,              /* bit 1 = CPOL, bit 0 = CPHA */
        .clock_speed_hz = hiz_hz,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };
    return spi_bus_add_device(SPI_BIRIM, &ayar, cihaz);
}

/* Tek işlem: gönderilen ile alınanı karşılaştırır, hatalı bit sayısını döndürür (-1: sürücü hatası) */
static int islem_yap(spi_device_handle_t cihaz, const uint8_t *giden, uint8_t *gelen, int *hatali_bayt)
{
    spi_transaction_t islem = {
        .length = BAYT_SAYISI * 8,         /* bit cinsinden */
        .tx_buffer = giden,
        .rx_buffer = gelen,
    };
    memset(gelen, 0, BAYT_SAYISI);
    if (spi_device_polling_transmit(cihaz, &islem) != ESP_OK) {
        return -1;
    }
    int bit_hatasi = 0;
    *hatali_bayt = 0;
    for (int i = 0; i < BAYT_SAYISI; i++) {
        uint8_t fark = giden[i] ^ gelen[i];
        if (fark) {
            (*hatali_bayt)++;
            bit_hatasi += __builtin_popcount(fark);
        }
    }
    return bit_hatasi;
}

static void bayt_yaz(const uint8_t *b)
{
    for (int i = 0; i < BAYT_SAYISI; i++) {
        printf("%02X%s", b[i], i < BAYT_SAYISI - 1 ? " " : "");
    }
}

/* ------------------------------------------------------------------ */
/* 1) Mod 0-3                                                          */
/* ------------------------------------------------------------------ */

static void mod_testi(void)
{
    static const uint8_t SABIT_DESEN[BAYT_SAYISI] = { 0xDE, 0xAD, 0xBE, 0xEF };

    printf("\n=== Mod testi (%d MHz, %d bayt) ===\n", MOD_TESTI_HZ / 1000000, BAYT_SAYISI);
    printf("Mod CPOL CPHA | gönderilen    alınan        | bayt/bit hatası | SCLK boşta\n");

    for (int mod = 0; mod < 4; mod++) {
        int cpol = (mod >> 1) & 1;
        int cpha = mod & 1;
        spi_device_handle_t cihaz;
        esp_err_t sonuc = cihaz_ekle(mod, MOD_TESTI_HZ, &cihaz);
        if (sonuc != ESP_OK) {
            printf(" %d  cihaz eklenemedi: %s\n", mod, esp_err_to_name(sonuc));
            continue;
        }

        uint8_t desenler[2][BAYT_SAYISI];
        memcpy(desenler[0], SABIT_DESEN, BAYT_SAYISI);
        for (int i = 0; i < BAYT_SAYISI; i++) {
            desenler[1][i] = sayac_degeri++;
        }

        for (int d = 0; d < 2; d++) {
            uint8_t gelen[BAYT_SAYISI];
            int hatali_bayt = 0;
            int bit_hatasi = islem_yap(cihaz, desenler[d], gelen, &hatali_bayt);

            /* İşlem bitti, saat hattı şimdi boşta: düzeyi CPOL'a eşit olmalı */
            int bosta = gpio_get_level(PIN_SCLK_GOZLEM);

            printf(" %d    %d    %d  | ", mod, cpol, cpha);
            bayt_yaz(desenler[d]);
            printf("   ");
            if (bit_hatasi < 0) {
                printf("(sürücü hatası)");
            } else {
                bayt_yaz(gelen);
            }
            printf("   | %d / %-2d          | %d (%s)\n",
                   hatali_bayt, bit_hatasi < 0 ? 0 : bit_hatasi, bosta,
                   bosta == cpol ? "CPOL ile uyumlu" : "CPOL ile UYUMSUZ");
        }
        spi_bus_remove_device(cihaz);
    }
}

/* ------------------------------------------------------------------ */
/* 2) Hız taraması                                                     */
/* ------------------------------------------------------------------ */

static void hiz_taramasi(void)
{
    printf("\n=== Hız taraması (mod 0, her hızda %d işlem) ===\n", HIZ_TESTI_ISLEM);
    printf("İstenen    Gerçek     | hatalı işlem | bit hatası | işlem başına süre\n");

    for (size_t h = 0; h < TARAMA_HIZ_SAYISI; h++) {
        int hiz = TARAMA_HIZLARI_HZ[h];
        spi_device_handle_t cihaz;
        esp_err_t sonuc = cihaz_ekle(0, hiz, &cihaz);
        if (sonuc != ESP_OK) {
            printf("%2d MHz     cihaz eklenemedi: %s\n", hiz / 1000000, esp_err_to_name(sonuc));
            continue;
        }
        int gercek_khz = 0;
        spi_device_get_actual_freq(cihaz, &gercek_khz);

        uint32_t hatali_islem = 0, toplam_bit_hatasi = 0, surucu_hatasi = 0;
        int64_t t0 = esp_timer_get_time();
        for (int n = 0; n < HIZ_TESTI_ISLEM; n++) {
            uint8_t giden[BAYT_SAYISI], gelen[BAYT_SAYISI];
            for (int i = 0; i < BAYT_SAYISI; i++) {
                giden[i] = sayac_degeri++;
            }
            int hatali_bayt = 0;
            int bit_hatasi = islem_yap(cihaz, giden, gelen, &hatali_bayt);
            if (bit_hatasi < 0) {
                surucu_hatasi++;
            } else if (bit_hatasi > 0) {
                hatali_islem++;
                toplam_bit_hatasi += (uint32_t)bit_hatasi;
            }
        }
        int64_t sure = esp_timer_get_time() - t0;
        spi_bus_remove_device(cihaz);

        printf("%2d MHz     %6.3f MHz | %3" PRIu32 " / %d    | %10" PRIu32 " | %5.2f us",
               hiz / 1000000, gercek_khz / 1000.0, hatali_islem, HIZ_TESTI_ISLEM,
               toplam_bit_hatasi, (double)sure / HIZ_TESTI_ISLEM);
        if (surucu_hatasi) {
            printf("  (sürücü hatası %" PRIu32 ")", surucu_hatasi);
        }
        printf("\n");
    }
}

/* ------------------------------------------------------------------ */

void app_main(void)
{
    const spi_bus_config_t yol_ayari = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    /* 4 baytlık işlemler için DMA gerekmez; veri doğrudan SPI FIFO'sundan geçer */
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_BIRIM, &yol_ayari, SPI_DMA_DISABLED));

    const gpio_config_t gozlem_pini = {
        .pin_bit_mask = 1ULL << PIN_SCLK_GOZLEM,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gozlem_pini));

    ESP_LOGI(TAG, "SPI2: MOSI=%d MISO=%d SCLK=%d CS=%d, SCLK gözlem pini=%d",
             PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, PIN_SCLK_GOZLEM);

    for (uint32_t tur = 1;; tur++) {
        printf("\n########## Tur %" PRIu32 " ##########\n", tur);
        mod_testi();
        hiz_taramasi();
        vTaskDelay(pdMS_TO_TICKS(TUR_ARASI_MS));
    }
}
