# Dự Án: Đọc Dữ Liệu Từ Cảm Biến và Gửi Qua MQTT

## Giới Thiệu

Đoạn mã dưới đây thực hiện việc đọc dữ liệu từ cảm biến DHT11 và MQ2 sau đó gửi dữ liệu này qua giao thức MQTT. Cảm biến DHT11 đo nhiệt độ và độ ẩm, cảm biến MQ2 đo giá trị nồng độ khí gas và MQTT được sử dụng để truyền dữ liệu này đến một máy chủ từ xa.

## Mã Nguồn

```c
// DHT Task
void dht_sensor_task(void *pvParameters)
{
    while(1)
    {
        if(dht_read_float_data(SENSOR_TYPE, DHT_PIN, &humidity, &temperature) == ESP_OK)
            printf("Humidity: %.1f%%, Temperature: %.1fC\n", humidity, temperature);
        else
            printf("Could not read data from DHT11\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// MQ2 Task
void mq2_sensor_task(void *pvParameter)
{
    uint32_t gpio_num;
    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            int digital_value = gpio_get_level(gpio_num);
            printf("MQ2 Input Value: %d\n", digital_value);
        }
    }
}

// MQTT Task
void mqtt_publish_task(void *pvParameter)
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameter;
    char payload[100];
    
    while (1)
    {
        snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);
        
        int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, payload, 0, 1, 0);
        printf("Publish message ID: %d\n", msg_id);

        vTaskDelay(pdMS_TO_TICKS(5000)); // Send value every 5 seconds
    }
}

void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on change
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0); // Install GPIO ISR service

    gpio_isr_handler_add(MQ2_PIN, gpio_isr_handler, (void*) MQ2_PIN); // Hook ISR handle for MQ2_PIN

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // Create queue to handle GPIO event from ISR

    xTaskCreate(mq2_sensor_task, "mq2_sensor_task", 2048, NULL, 10, NULL); // Create MQ2 Task
    xTaskCreate(dht_sensor_task, "dht_sensor_task", 2048, NULL, 10, NULL); // Create DHT Task

    //Start MQTT client
    esp_err_t ret = mqtt_app_start();
    if (ret != ESP_OK)
    {
        printf("Failed to start MQTT client\n");
        return;
    }

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, client, 5, NULL); // Create MQTT Task
}
```
## Giải thích từng đoạn code
```c
// DHT Task
void dht_sensor_task(void *pvParameters)
{
    while(1)
    {
        if(dht_read_float_data(SENSOR_TYPE, DHT_PIN, &humidity, &temperature) == ESP_OK)
            printf("Humidity: %.1f%%, Temperature: %.1fC\n", humidity, temperature);
        else
            printf("Could not read data from DHT11\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```
Đoạn code trên là Task đọc dữ liệu từ cảm biến DHT11-Cảm biến nhiệt độ, độ ẩm với hàm `dht_read_float_data()` được sử dụng để đọc giá trị humidity và temperature của cảm biến SENSOR_TYPE từ chân DHT_PIN, toán tử so sánh `== ESP_OK` để đảm bảo giá trị không bị `NULL`
Sau đó giá trị sẽ được lưu vào biến `temperature` và `humidity` đã được tạo và kèm theo là gửi giá trị qua cổng Serial.
```c
// MQ2 Task
void mq2_sensor_task(void *pvParameter)
{
    uint32_t gpio_num;
    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY))
        {
            int digital_value = gpio_get_level(gpio_num);
            printf("MQ2 Input Value: %d\n", digital_value);
        }
    }
}
```
Đoạn code trên là Task đọc dữ liệu từ cảm biến MQ2-Cảm biến đo nồng độ khí gas, hàm `xQueueReceive()` dùng để nhận dữ liệu từ một hàng đợi trong FreeRTOS, dòng `int digital_value = gpio_get_level(gpio_num);` dùng để gán giá trị logic đã đọc được vào biến `digital_value`.
Sau đó gửi giá trị qua cổng Serial.
```c
// MQTT Task
void mqtt_publish_task(void *pvParameter)
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameter;
    char payload[100];
    
    while (1)
    {
        snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);
        
        int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, payload, 0, 1, 0);
        printf("Publish message ID: %d\n", msg_id);

        vTaskDelay(pdMS_TO_TICKS(5000)); // Send value every 5 seconds
    }
}
```
Đoạn code trên là Task gửi giá trị thu được lên Broker MQTT. dòng `esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameter;` sử dụng để ép kiểu của biến pvParameters thành dạng esp_mqtt_client_handle_t để có thể truyền vào hàm ở dưới. Dòng `char payload[100];` để khai báo một mảng ký tự (string) có kích thước 100 byte để chứa dữ liệu sẽ được gửi qua MQTT.
Dòng `snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);` sẽ định dạng chuỗi thành String và sau đó lưu vào payload. Dòng `int msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC, payload, 0, 1, 0);` được dùng để gửi tin nhắn qua MQTT.
```c
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on change
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
```
Đoạn code trên để cấu hình chân GPIO cho cảm biến MQ2, với các cấu hình là `MODE_INPUT`, `PULLDOWN_DISABLE`, `PULLUP_DISABLE`, `INTR_ANYEDGE`. 
```c
    gpio_install_isr_service(0); // Install GPIO ISR service
    gpio_isr_handler_add(MQ2_PIN, gpio_isr_handler, (void*) MQ2_PIN); // Hook ISR handle for MQ2_PIN
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // Create queue to handle GPIO event from ISR
```
Đoạn code trên cấu hình các chức năng ngắt bao gồm `gpio_install_isr_service()` dùng để cài đặt chức năng ngắt, `gpio_isr_handler_add()` dùng để thêm xử lý ngắt cho chân `MQ2_PIN`, `xQueueCreate()` dùng để tạo hàng đợi chứa các ngắt.
```c
xTaskCreate(mq2_sensor_task, "mq2_sensor_task", 2048, NULL, 10, NULL); // Create MQ2 Task
    xTaskCreate(dht_sensor_task, "dht_sensor_task", 2048, NULL, 10, NULL); // Create DHT Task

    // Start MQTT client
    esp_err_t ret = mqtt_app_start();
    if (ret != ESP_OK)
    {
        printf("Failed to start MQTT client\n");
        return;
    }

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_publish_task", 4096, client, 5, NULL); // Create MQTT Task
```
Đoạn code trên dùng để kích hoạt các Task đã được tạo ở trên và kết nối tới MQTT Broker. `xTaskCreate()` dùng để tạo các Task, `mqtt_app_start()` dùng để kết nối tới MQTT Broker.
