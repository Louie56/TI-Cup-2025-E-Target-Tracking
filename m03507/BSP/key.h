#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"


// 事件类型枚举
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_CLICK,
    KEY_EVENT_DOUBLE_CLICK,
    KEY_EVENT_TRIPLE_CLICK,
    KEY_EVENT_QUAD_CLICK
} key_event_t;

extern uint32_t get_system_time_ms();

key_event_t key_detect(void);
int Count_Key1_Press(void);

#endif
/*
while (1)
{
    key_event_t result = key_detect();
    if (result == KEY_EVENT_CLICK) {
        // 处理单击
    }
    if (result == KEY_EVENT_DOUBLE_CLICK) {
        // 处理双击
    }
    if (result == KEY_EVENT_LONG_PRESS) {
        // 处理长按
    }
    // 其它任务...
}
*/