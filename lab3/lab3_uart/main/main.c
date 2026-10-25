/*
 * İleri Gömülü Sistemler - Laboratuvar 3, Görev 1: UART geri döngü
 * ESP32-S3, ESP-IDF v6.1
 *
 * Bağlantı (tek kablo ile üç pin aynı hatta):
 *   GPIO17 (UART1 TX) ──┬── GPIO18 (UART1 RX)  geri döngü
 *                       └── GPIO16 (UART2 RX)  yanlış baud gözlemi
 *
 * Program yapısı
 *   - Her iki UART'ın sürücüsü bir olay kuyruğuyla kurulur. Sürücü kesmesi
 *     gelen veriyi ve hataları bu kuyruğa "olay" olarak koyar.
 *   - Her UART için ayrı bir alıcı görevi olayları işler: veri, çerçeve
 *     hatası, eşlik hatası, FIFO / tampon taşması.
 *   - Ana görev iki aşamayı sırayla ve sürekli tekrarlar:
 *       1) Doğru baud: 500 ms'de bir numaralı paket gönderilir, UART1'in
 *          kendi RX'ine geri dönen satır karşılaştırılır ve gönderme ile
 *          alma arasındaki süre ölçülür. 10 paketin özeti yazdırılır.
 *       2) Yanlış baud: UART1 hep 115200 baud ile gönderir, UART2 ise
 *          farklı hızlarda dinler. Gelen baytlar hex olarak ve çerçeve
 *          hatası sayısıyla yazdırılır.
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "hal/uart_ll.h"      /* UART_INTR_FRAM_ERR maskesi */
#include "esp_timer.h"
#include "esp_log.h"

#define PIN_TX              17
#define PIN_RX_GERI         18
#define PIN_RX_YANLIS       16
#define BAUD_GONDEREN       115200
#define RX_TAMPON           1024
#define OLAY_KUYRUK_BOYU    20
#define PAKET_SAYISI        10
#define PAKET_ARASI_MS      500
#define SATIR_EN_FAZLA      64
#define HEX_EN_FAZLA        24

static const char *TAG = "uart_lab";

/* Yanlış baud aşamasında UART2'nin denediği hızlar */
static const uint32_t YANLIS_HIZLAR[] = { 9600, 57600, 105000, 230400 };
#define YANLIS_HIZ_SAYISI (sizeof(YANLIS_HIZLAR) / sizeof(YANLIS_HIZLAR[0]))

typedef enum { ASAMA_DOGRU, ASAMA_YANLIS } asama_t;
static volatile asama_t asama = ASAMA_DOGRU;

static QueueHandle_t olay_u1;      /* UART1 sürücü olayları */
static QueueHandle_t olay_u2;      /* UART2 sürücü olayları */

/* UART1 alıcısı tamamlanan her satırı bu kuyruğa koyar */
typedef struct {
    char    metin[SATIR_EN_FAZLA];
    int64_t zaman_us;               /* satır sonu ('\n') işlendiği an */
} satir_t;
static QueueHandle_t satir_kuyrugu;

/* Yanlış baud aşaması sayaçları (UART2 alıcı görevi doldurur) */
static portMUX_TYPE sayac_kilidi = portMUX_INITIALIZER_UNLOCKED;
static uint8_t  u2_baytlar[HEX_EN_FAZLA];
static size_t   u2_saklanan;
static uint32_t u2_toplam_bayt;
static uint32_t u2_cerceve_hatasi;
static uint32_t u2_eslik_hatasi;
static uint32_t u2_tasma;

/* UART1 tarafında görülen hatalar (normalde hep 0 kalmalı) */
static volatile uint32_t u1_cerceve_hatasi;
static volatile uint32_t u1_tasma;

/* ------------------------------------------------------------------ */
/* Alıcı görevleri                                                     */
/* ------------------------------------------------------------------ */

/* UART1: gelen baytları satırlara böler, doğru baud aşamasında satırı ana göreve iletir */
static void u1_alici_gorevi(void *arg)
{
    uart_event_t olay;
    uint8_t parca[128];
    char satir[SATIR_EN_FAZLA];
    size_t uzunluk = 0;

    for (;;) {
        if (xQueueReceive(olay_u1, &olay, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (olay.type) {
        case UART_DATA: {
            size_t kalan = olay.size;
            while (kalan > 0) {
                size_t istenen = kalan < sizeof(parca) ? kalan : sizeof(parca);
                int okunan = uart_read_bytes(UART_NUM_1, parca, istenen, 0);
                if (okunan <= 0) {
                    break;
                }
                kalan -= (size_t)okunan;
                for (int i = 0; i < okunan; i++) {
                    if (parca[i] == '\n') {
                        satir[uzunluk] = '\0';
                        if (asama == ASAMA_DOGRU) {
                            satir_t s;
                            s.zaman_us = esp_timer_get_time();
                            snprintf(s.metin, sizeof(s.metin), "%s", satir);
                            xQueueSend(satir_kuyrugu, &s, 0);
                        }
                        uzunluk = 0;
                    } else if (uzunluk < sizeof(satir) - 1) {
                        satir[uzunluk++] = (char)parca[i];
                    }
                }
            }
            break;
        }
        case UART_FRAME_ERR:
            u1_cerceve_hatasi++;
            break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            u1_tasma++;
            uart_flush_input(UART_NUM_1);
            xQueueReset(olay_u1);
            uzunluk = 0;
            break;
        default:
            break;
        }
    }
}

/* UART2: yalnız yanlış baud aşamasında gelen baytları ve hataları sayar */
static void u2_alici_gorevi(void *arg)
{
    uart_event_t olay;
    uint8_t parca[128];

    for (;;) {
        if (xQueueReceive(olay_u2, &olay, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        bool kaydet = (asama == ASAMA_YANLIS);
        switch (olay.type) {
        case UART_DATA: {
            size_t kalan = olay.size;
            while (kalan > 0) {
                size_t istenen = kalan < sizeof(parca) ? kalan : sizeof(parca);
                int okunan = uart_read_bytes(UART_NUM_2, parca, istenen, 0);
                if (okunan <= 0) {
                    break;
                }
                kalan -= (size_t)okunan;
                if (kaydet) {
                    portENTER_CRITICAL(&sayac_kilidi);
                    for (int i = 0; i < okunan && u2_saklanan < HEX_EN_FAZLA; i++) {
                        u2_baytlar[u2_saklanan++] = parca[i];
                    }
                    u2_toplam_bayt += (uint32_t)okunan;
                    portEXIT_CRITICAL(&sayac_kilidi);
                }
            }
            break;
        }
        case UART_FRAME_ERR:
            if (kaydet) {
                portENTER_CRITICAL(&sayac_kilidi);
                u2_cerceve_hatasi++;
                portEXIT_CRITICAL(&sayac_kilidi);
            }
            break;
        case UART_PARITY_ERR:
            if (kaydet) {
                portENTER_CRITICAL(&sayac_kilidi);
                u2_eslik_hatasi++;
                portEXIT_CRITICAL(&sayac_kilidi);
            }
            break;
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
            portENTER_CRITICAL(&sayac_kilidi);
            u2_tasma++;
            portEXIT_CRITICAL(&sayac_kilidi);
            uart_flush_input(UART_NUM_2);
            xQueueReset(olay_u2);
            break;
        default:
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Kurulum                                                             */
/* ------------------------------------------------------------------ */

static void uart_kur(uart_port_t port, uint32_t baud, int tx, int rx, QueueHandle_t *kuyruk)
{
    const uart_config_t ayar = {
        .baud_rate  = (int)baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* TX tamponu 0: uart_write_bytes veri FIFO'ya girene kadar bekler */
    ESP_ERROR_CHECK(uart_driver_install(port, RX_TAMPON, 0, OLAY_KUYRUK_BOYU, kuyruk, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &ayar));
    ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* Çerçeve hatası kesmesi sürücüde varsayılan olarak kapalı; olay üretmesi için açılır */
    ESP_ERROR_CHECK(uart_enable_intr_mask(port, UART_INTR_FRAM_ERR));
}

/* ------------------------------------------------------------------ */
/* Aşama 1: doğru baud                                                 */
/* ------------------------------------------------------------------ */

static uint32_t paket_no;

static void dogru_baud_asamasi(void)
{
    uint32_t gonderilen = 0, alinan = 0, hatali = 0, cevapsiz = 0;
    int64_t gecikme_toplam = 0, gecikme_en_buyuk = 0;
    char paket[SATIR_EN_FAZLA];

    asama = ASAMA_DOGRU;
    xQueueReset(satir_kuyrugu);
    printf("\n=== Doğru baud: UART1 %" PRIu32 " baud, GPIO17 -> GPIO18 ===\n", (uint32_t)BAUD_GONDEREN);

    for (int i = 0; i < PAKET_SAYISI; i++) {
        paket_no++;
        int n = snprintf(paket, sizeof(paket), "P%03" PRIu32 " merhaba ESP32-S3\n", paket_no % 1000);

        int64_t t0 = esp_timer_get_time();
        uart_write_bytes(UART_NUM_1, paket, (size_t)n);
        gonderilen++;

        satir_t gelen;
        if (xQueueReceive(satir_kuyrugu, &gelen, pdMS_TO_TICKS(200)) != pdTRUE) {
            cevapsiz++;
            printf("  %.*s  ->  (cevap yok; GPIO17-GPIO18 kablosunu kontrol edin)\n", n - 1, paket);
        } else {
            int64_t gecikme = gelen.zaman_us - t0;
            bool ayni = (strncmp(gelen.metin, paket, (size_t)(n - 1)) == 0) && gelen.metin[n - 1] == '\0';
            alinan++;
            if (!ayni) {
                hatali++;
            } else {
                gecikme_toplam += gecikme;
                if (gecikme > gecikme_en_buyuk) {
                    gecikme_en_buyuk = gecikme;
                }
            }
            printf("  %.*s  ->  \"%s\"  %s  %" PRId64 " us\n",
                   n - 1, paket, gelen.metin, ayni ? "AYNI" : "FARKLI", gecikme);
        }
        vTaskDelay(pdMS_TO_TICKS(PAKET_ARASI_MS));
    }

    uint32_t dogru = alinan - hatali;
    printf("Özet: gönderilen %" PRIu32 ", alınan %" PRIu32 ", hatalı %" PRIu32 ", cevapsız %" PRIu32,
           gonderilen, alinan, hatali, cevapsiz);
    if (dogru > 0) {
        printf(" | gecikme ort. %" PRId64 " us, en büyük %" PRId64 " us",
               gecikme_toplam / (int64_t)dogru, gecikme_en_buyuk);
    }
    printf(" | UART1 çerçeve hatası %" PRIu32 ", taşma %" PRIu32 "\n", u1_cerceve_hatasi, u1_tasma);
}

/* ------------------------------------------------------------------ */
/* Aşama 2: yanlış baud                                                */
/* ------------------------------------------------------------------ */

static void yanlis_baud_asamasi(void)
{
    static const char METIN[] = "Yanlis baud testi 0123\n";
    const size_t metin_uzunluk = sizeof(METIN) - 1;

    printf("\n=== Yanlış baud: gönderen UART1 %" PRIu32 " baud, dinleyen UART2 (GPIO16) ===\n",
           (uint32_t)BAUD_GONDEREN);
    printf("Gönderilen %u bayt: \"Yanlis baud testi 0123\\n\"\n", (unsigned)metin_uzunluk);

    for (size_t h = 0; h < YANLIS_HIZ_SAYISI; h++) {
        uint32_t hiz = YANLIS_HIZLAR[h];

        asama = ASAMA_DOGRU;                 /* ayar sırasında UART2 verisini sayma */
        uart_set_baudrate(UART_NUM_2, hiz);
        uint32_t gercek = 0;
        uart_get_baudrate(UART_NUM_2, &gercek);
        vTaskDelay(pdMS_TO_TICKS(20));
        uart_flush_input(UART_NUM_2);

        portENTER_CRITICAL(&sayac_kilidi);
        u2_saklanan = 0;
        u2_toplam_bayt = 0;
        u2_cerceve_hatasi = 0;
        u2_eslik_hatasi = 0;
        u2_tasma = 0;
        portEXIT_CRITICAL(&sayac_kilidi);

        asama = ASAMA_YANLIS;
        uart_write_bytes(UART_NUM_1, METIN, metin_uzunluk);
        uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(150));       /* UART2'nin zaman aşımıyla son baytları teslim etmesi için */
        asama = ASAMA_DOGRU;

        uint8_t kopya[HEX_EN_FAZLA];
        size_t saklanan;
        uint32_t toplam, cerceve, eslik, tasma;
        portENTER_CRITICAL(&sayac_kilidi);
        saklanan = u2_saklanan;
        memcpy(kopya, u2_baytlar, saklanan);
        toplam = u2_toplam_bayt;
        cerceve = u2_cerceve_hatasi;
        eslik = u2_eslik_hatasi;
        tasma = u2_tasma;
        portEXIT_CRITICAL(&sayac_kilidi);

        int32_t fark_binde = (int32_t)(((int64_t)hiz - BAUD_GONDEREN) * 1000 / BAUD_GONDEREN);
        printf("UART2 %6" PRIu32 " baud (gerçek %6" PRIu32 ", fark %+d,%d%%): %2" PRIu32 " bayt, çerçeve hatası %" PRIu32,
               hiz, gercek, (int)(fark_binde / 10), (int)((fark_binde < 0 ? -fark_binde : fark_binde) % 10),
               toplam, cerceve);
        if (eslik || tasma) {
            printf(", eşlik %" PRIu32 ", taşma %" PRIu32, eslik, tasma);
        }
        printf("\n    hex:");
        for (size_t i = 0; i < saklanan; i++) {
            printf(" %02X", kopya[i]);
        }
        if (toplam > saklanan) {
            printf(" ...");
        }
        printf("\n    metin: \"");
        for (size_t i = 0; i < saklanan; i++) {
            putchar((kopya[i] >= 0x20 && kopya[i] < 0x7F) ? kopya[i] : '.');
        }
        printf("\"\n");
    }
    /* UART2'yi tekrar gönderenle aynı hıza al (sonraki turda gereksiz hata üretmesin) */
    uart_set_baudrate(UART_NUM_2, BAUD_GONDEREN);
}

void app_main(void)
{
    satir_kuyrugu = xQueueCreate(4, sizeof(satir_t));
    configASSERT(satir_kuyrugu);

    uart_kur(UART_NUM_1, BAUD_GONDEREN, PIN_TX, PIN_RX_GERI, &olay_u1);
    uart_kur(UART_NUM_2, BAUD_GONDEREN, UART_PIN_NO_CHANGE, PIN_RX_YANLIS, &olay_u2);

    xTaskCreate(u1_alici_gorevi, "u1_alici", 4096, NULL, 12, NULL);
    xTaskCreate(u2_alici_gorevi, "u2_alici", 4096, NULL, 12, NULL);

    ESP_LOGI(TAG, "UART1: TX=GPIO%d RX=GPIO%d, UART2: RX=GPIO%d", PIN_TX, PIN_RX_GERI, PIN_RX_YANLIS);

    for (uint32_t tur = 1;; tur++) {
        printf("\n########## Tur %" PRIu32 " ##########\n", tur);
        dogru_baud_asamasi();
        yanlis_baud_asamasi();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
