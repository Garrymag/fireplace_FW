#include "display.h"
#include "cmsis_os.h"
#include <task.h>
#include <queue.h>

QueueHandle_t dwin_write_queue;
QueueHandle_t dwin_read_queue;
TaskHandle_t dwin_task_handle;

void dwin_task_func(void *pvParameters);

void dwin_init()
{
  DWIN.setPins(PC11, PC10);
  DWIN.begin(115200);

  delay(3000);

  // Create queues
  dwin_write_queue = xQueueCreate(10, sizeof(dwin_write_msg_t));
  dwin_read_queue = xQueueCreate(10, sizeof(dwin_read_msg_t));

  // Create task
  xTaskCreate(dwin_task_func, "DWIN_Task", 2048, NULL, 1, &dwin_task_handle);

}

void dwin_task_func(void *pvParameters) {
    for (;;) {

        // Process read
        if(DWIN.read()) {
            DisplayData data = DWIN.getDisplayData();
            dwin_read_msg_t read_msg = {data.address, data.data};
            xQueueSend(dwin_read_queue, &read_msg, 0);
        }

        // Process write queue
        dwin_write_msg_t write_msg;
        if (xQueueReceive(dwin_write_queue, &write_msg, 0) == pdPASS) {
            DWIN.send(write_msg.addr, write_msg.value);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

BaseType_t dwin_send(uint16_t addr, uint16_t value) {
    dwin_write_msg_t msg = {addr, value};
    return xQueueSend(dwin_write_queue, &msg, portMAX_DELAY);
}

BaseType_t dwin_receive(dwin_read_msg_t *msg) {
    return xQueueReceive(dwin_read_queue, msg, 0);
}

void dwin_set_page(uint16_t page) {
    DWIN.setPage(page);
}