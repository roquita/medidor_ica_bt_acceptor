
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "freertos/ringbuf.h"

#include "time.h"
#include "sys/time.h"

#define CONF_PIN (GPIO_NUM_21)
#define TXD_PIN (GPIO_NUM_17) //(GPIO_NUM_18)
#define RXD_PIN (GPIO_NUM_16) //(GPIO_NUM_19)
#define TEN_PIN (GPIO_NUM_22)
static const int RX_BUF_SIZE = 2048;

#define SPP_TAG "X-TAG" //"SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "X_SERVER"
#define EXAMPLE_DEVICE_NAME "X_SLAVE"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA /*Choose show mode: show data or speed*/

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE; // ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;
uint32_t bthandle = 0;
bool write_flag_enabled = true;
bool is_cong_needed = false;
RingbufHandle_t buf_handle;
bool read_flag_enabled = false;

char device_id[50];

void create_device_id()
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    sprintf(device_id, "16-%02X-%02X-%02X-%02X-%02X-%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void uart_tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    buf_handle = xRingbufferCreate(1028, RINGBUF_TYPE_BYTEBUF);

    while (1)
    {
        if (read_flag_enabled)
        {
            vTaskDelay(pdMS_TO_TICKS(300)); // vTaskDelay(pdMS_TO_TICKS(200));

            size_t size = 0;
            void *data = xRingbufferReceive(buf_handle, &size, pdMS_TO_TICKS(1000));

            if (data)
            {
                gpio_set_level(TEN_PIN, 1);
                uart_write_bytes(UART_NUM_2, data, size);
                uart_wait_tx_done(UART_NUM_2, portMAX_DELAY);
                gpio_set_level(TEN_PIN, 0);

                vRingbufferReturnItem(buf_handle, data);
            }
            read_flag_enabled = false;
        }
        vTaskDelay(1);
    }
}

static void uart_rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    int rxBytes = 0;
    while (1)
    {
        rxBytes += uart_read_bytes(UART_NUM_2, data + rxBytes, RX_BUF_SIZE - rxBytes, pdMS_TO_TICKS(20)); // pdMS_TO_TICKS(1)
        if (bthandle == 0)
        {
            memset(data, 0, rxBytes);
            rxBytes = 0;
            continue;
        }
        if (rxBytes > 0 && write_flag_enabled)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes'", rxBytes);
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            esp_spp_write(bthandle, rxBytes, data);
            rxBytes = 0;
            write_flag_enabled = false;
        }
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
}

static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s", time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        bthandle = 0;
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");/*
        if (gpio_get_level(CONF_PIN) == 0) // 7E1
        {
            printf("name 7e1\n");
            esp_bt_dev_set_device_name("AS1440");
        }
        else // 8N1
        {
            printf("name 8n1\n");
            esp_bt_dev_set_device_name("A1800");
        }*/
        // esp_bt_dev_set_device_name(device_id);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        // ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
        //          param->data_ind.len, param->data_ind.handle);
        // esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
        // esp_spp_write(param->cong.handle, 7, (uint8_t *)"hola bb");
        // printf("bt habdle: %d\n", param->data_ind.handle);
        // printf("bt habdle ram: %d\n", bthandle);
        /*
                gpio_set_level(TEN_PIN, 1);
                uart_write_bytes(UART_NUM_2, (void *)param->data_ind.data, param->data_ind.len);
                uart_wait_tx_done(UART_NUM_2, portMAX_DELAY);
                gpio_set_level(TEN_PIN, 0);
                */

        // Send an item
        xRingbufferSend(buf_handle, param->data_ind.data, param->data_ind.len, pdMS_TO_TICKS(1000));
        read_flag_enabled = true;

#else
        gettimeofday(&time_new, NULL);
        data_num += param->data_ind.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3)
        {
            print_speed();
        }
#endif
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");

        if (is_cong_needed && param->cong.cong == false)
        {
            is_cong_needed = false;
            write_flag_enabled = true;
        }

        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT || cong : %d", param->write.cong);
        // write_flag_enabled = true;

        if (param->write.cong == false)
        {
            write_flag_enabled = true;
        }
        else
        {
            is_cong_needed = true;
        }

        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        gettimeofday(&time_old, NULL);
        bthandle = param->data_ind.handle;
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        else
        {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default:
    {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void app_main(void)
{
    const gpio_config_t gpio_conf = {
        .pin_bit_mask = 1UL << CONF_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio_conf);

    create_device_id();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

            if (gpio_get_level(CONF_PIN) == 0) // 7E1
        {
            printf("name 7e1\n");
            esp_bt_dev_set_device_name("AS1440");
        }
        else // 8N1
        {
            printf("name 8n1\n");
            esp_bt_dev_set_device_name("A1800");
        }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    // Set default parameters for Secure Simple Pairing
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO; // ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED; // ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code = {'5', '5', '5', '5', '5'};
    esp_bt_gap_set_pin(pin_type, 5, pin_code);

    const gpio_config_t ten_conf = {
        .pin_bit_mask = 1UL << TEN_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ten_conf);
    gpio_set_level(TEN_PIN, 0);

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = gpio_get_level(CONF_PIN) == 0 ? UART_DATA_7_BITS : UART_DATA_8_BITS,
        .parity = gpio_get_level(CONF_PIN) == 0 ? UART_PARITY_EVEN : UART_PARITY_DISABLE,
        //.data_bits = UART_DATA_8_BITS,
        //.parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    char parity = uart_config.parity == UART_PARITY_DISABLE ? 'N' : (uart_config.parity == UART_PARITY_EVEN ? 'E' : 'O');
    uint8_t data_bits = uart_config.data_bits + 5;
    printf("uart data config: %d%c1\n\r", data_bits, parity);

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(uart_rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);

    printf("config done!!!!\n\r");
}
