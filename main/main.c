#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "lwip/sockets.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#define SLAVE_ADDRESS_LCD 0x27
#define I2C_MASTER_SCL_IO GPIO_NUM_17
#define I2C_MASTER_SDA_IO GPIO_NUM_18
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_NUM I2C_NUM_0

static const char *TAG = "LCD";

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)

static bool USE_WIFI = false;
static bool is_connected = false;
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASS "WIFI_PW"
#define PORT 8080

esp_err_t err;
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Disconnected. Reconnecting...");
        esp_wifi_connect();
        is_connected = false;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        is_connected = true;
    }
}
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
}
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
static esp_err_t uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    return uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}
void lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;
    data_t[1] = data_u | 0x08;
    data_t[2] = data_l | 0x0C;
    data_t[3] = data_l | 0x08;
    err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
    if (err != 0)
        ESP_LOGI(TAG, "Error in sending command");
}
void lcd_send_data(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;
    data_t[1] = data_u | 0x09;
    data_t[2] = data_l | 0x0D;
    data_t[3] = data_l | 0x09;
    err = i2c_master_write_to_device(I2C_NUM, SLAVE_ADDRESS_LCD, data_t, 4, 1000);
    if (err != 0)
        ESP_LOGI(TAG, "Error in sending data");
}
void lcd_put_cur(int row, int col)
{
    switch (row)
    {
    case 0:
        col |= 0x80;
        break;
    case 1:
        col |= 0xC0;
        break;
    }

    lcd_send_cmd(col);
}
void lcd_send_string(char *str)
{
    while (*str)
        lcd_send_data(*str++);
}

void lcd_init(void)
{
    for (size_t i = 0; i < 3; i++)
    {
        lcd_send_cmd(0x30); // 1 line, 8-bit mode
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    lcd_send_cmd(0x20); // 1 line, 4-bit mode
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_send_cmd(0x28); // 2 lines, 5x8 matrix,4-bit mode
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_send_cmd(0x01); // Clear display
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_send_cmd(0x06); // Shift cursor right
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_send_cmd(0x0C); // Display on, cursor off
    vTaskDelay(pdMS_TO_TICKS(50));
}
void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
}
void tcp_server_task(void *pvParameters)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);

    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0)
    {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) != 0)
    {
        ESP_LOGE(TAG, "Error during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", PORT);

    while (1)
    {
        struct sockaddr_in client_addr;
        char rx_buffer[128];
        uint32_t addr_len = sizeof(client_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Client connected");

        while (1)
        {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0)
            {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            else if (len == 0)
            {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            else
            {
                ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
                rx_buffer[len] = '\0';
                lcd_clear();
                lcd_put_cur(0, 0);
                lcd_send_string(rx_buffer);
            }
        }

        shutdown(sock, 0);
        close(sock);
    }

    close(listen_sock);
    vTaskDelete(NULL);
}
void serial_read_task(void *pvParameters)
{
    uint8_t data[64];
    int length = 0;
    while (1)
    {
        ESP_LOGI(TAG, "Reading from UART");
        length = uart_read_bytes(UART_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Length: %d", length);

        if (length > 0)
        {
            if (length >= sizeof(data))
            {
                length = sizeof(data) - 1;
            }

            ESP_LOGI(TAG, "Received from Serial: %s", (char *)data);
            data[length] = '\0';
            lcd_clear();
            lcd_put_cur(0, 0);
            lcd_send_string((char *)data);
        }
        else if (length < 0)
        {
            ESP_LOGE(TAG, "Error reading from UART");
        }
        else
        {
            ESP_LOGI(TAG, "No data received");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void app_main(void)
{

    if (USE_WIFI)
    {
        ESP_ERROR_CHECK(i2c_master_init());
        ESP_LOGI(TAG, "I2C initialized successfully");
        wifi_init_sta();
        while (!is_connected)
        {
            ESP_LOGI(TAG, "Waiting for connection...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        ESP_LOGI(TAG, "Connected to Wi-Fi. Continuing with main program...");
        lcd_init();
        xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    }
    else
    {
        ESP_ERROR_CHECK(i2c_master_init());
        ESP_LOGI(TAG, "I2C initialized successfully");
        ESP_ERROR_CHECK(uart_init());
        ESP_LOGI(TAG, "UART initialized successfully");
        lcd_init();
        xTaskCreate(serial_read_task, "serial_read", BUF_SIZE * 2, NULL, 5, NULL);
    }
}
