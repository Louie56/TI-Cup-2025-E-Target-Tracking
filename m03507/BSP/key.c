#include "key.h"
#include "ti_msp_dl_config.h"

//KEY1 PB8 初始化下拉 按下上拉

#define KEY_DEBOUNCE_TIME_MS      30
#define KEY_MULTI_CLICK_INTERVAL  400  // 多击最大间隔ms

int Count_Key1_Press(void)
{
 int count = 0;

    int confirmed = 0;

    while (1)
    {
        int current_state = DL_GPIO_readPins(KEYS_PORT, KEYS_KEY1_PIN_PIN);

        // 检测按下
        if (DL_GPIO_readPins(KEYS_PORT, KEYS_KEY1_PIN_PIN)==0)
        {
			delay_ms(20);
            while(DL_GPIO_readPins(KEYS_PORT, KEYS_KEY1_PIN_PIN)==0);
			delay_ms(20);
            count++;
           // 打印或提示按键次数
		//	uart0_send_char(count);
        }

       if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8)==0)
        {
			delay_ms(20);
            while(DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8)==0);
			delay_ms(20);
	//		KeyNum=2;
           // 打印或提示按键次数
			break;
        }
    }

    return count;
}
