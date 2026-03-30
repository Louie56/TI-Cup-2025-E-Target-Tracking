#include "uart0.h"
#include <stdio.h>
#include <stdint.h>
#include "time.h"

// 定义命令常量
#define CMD_PIXEL_COORD  0x30
#define CMD_TASK_ID      0x10
#define CMD_FLOAT_DATA   0x40  // 标定舵机 浮点数命令

//舵机标定
typedef struct {
    uint8_t group_id;
    float value1;
    float value2;
} FloatData;

FloatData float_data={0};  // 存储浮点数数据


/* 
   组0误差和任务ID 
*/
int x_err = 0;
int y_err = 0;
uint8_t task_id = 0;  // 新增任务ID变量




#define UART_BUFFER_SIZE 128
#define FRAME_HEAD1 0xAA
#define FRAME_HEAD2 0xAA
#define FRAME_TAIL1 0xFF
#define FRAME_TAIL2 0xFF
#define FRAME_TAIL3 0xFF

// 串口数据接收缓冲区
uint8_t uart_buffer[UART_BUFFER_SIZE];
uint8_t uart_index = 0;  // 缓冲区索引

// 当前接收到的帧数据
uint8_t addr = 0, cmd = 0, data[8];  // 存储帧地址，命令和数据

// 状态机，判断是否完整接收数据
typedef enum {
    WAIT_HEAD1,
    WAIT_HEAD2,
    WAIT_ADDR,
    WAIT_CMD,
    WAIT_DATA,
    WAIT_TAIL1,  // 新增：等待第一个帧尾
    WAIT_TAIL2,  // 新增：等待第二个帧尾
    WAIT_TAIL3   // 新增：等待第三个帧尾
} uart_state_t;

uart_state_t uart_state = WAIT_HEAD1;
uint8_t expected_data_len = 0;  // 根据命令预期的数据长度

// 解析坐标数据的函数
void process_coordinates(uint8_t group_id, uint8_t* data) {
    if(group_id == 0x01) {  // 只处理组ID=1
        x_err = ((data[0] << 8) | data[1])-10000;  // X 坐标（16 位）
        y_err = ((data[2] << 8) | data[3])-10000;  // Y 坐标（16 位）
        
		Openmv_Data();
    }
}

// 处理任务ID的函数
void process_task_id(uint8_t task) {
    task_id = task;  // 保存任务ID
    
//    // 调试输出（可选）
//    uart1_send_char('T');
//    uart1_send_char(task_id);
//    uart1_send_char('\n');
}


void process_float_data(uint8_t group_id, uint8_t* data) {
    float_data.group_id = group_id;
    
    // 解析第一个浮点数
    uint8_t int_part1 = data[0];
    uint8_t frac_part1 = data[1];
    float_data.value1 = int_part1 + (frac_part1 / 100.0f);
    
    // 解析第二个浮点数
    uint8_t int_part2 = data[2];
    uint8_t frac_part2 = data[3];
    float_data.value2 = int_part2 + (frac_part2 / 100.0f);
    
//    // 调试输出
//    uart1_send_char('F');
//    uart1_send_char('1');
//    uart1_send_float(float_data.value1);  // 发送整数形式的值
//    uart1_send_char('F');
//    uart1_send_char('2');
//    uart1_send_float(float_data.value2);
//    uart1_send_char('\n');
	
}

void uart_receive_byte(uint8_t byte) {
    switch (uart_state) {
        case WAIT_HEAD1:
            if (byte == FRAME_HEAD1) {
                uart_state = WAIT_HEAD2;
            }
            break;

        case WAIT_HEAD2:
            if (byte == FRAME_HEAD2) {
                uart_state = WAIT_ADDR;
            } else {
                uart_state = WAIT_HEAD1;  // 如果第二个字节不匹配，重新等待帧头
            }
            break;

        case WAIT_ADDR:
            addr = byte;  // 保存目标地址
            uart_state = WAIT_CMD;
            break;

        case WAIT_CMD:
            cmd = byte;  // 保存命令字
            uart_index = 0;  // 数据区索引重置
            
            // 根据命令设置预期数据长度
            if (cmd == CMD_PIXEL_COORD) {
                expected_data_len = 5;  // 组ID(1) + X(2) + Y(2)
            } else if (cmd == CMD_TASK_ID) {
                expected_data_len = 1;  // 任务ID(1)
            } else if (cmd == CMD_FLOAT_DATA) {
                expected_data_len = 5;  // 组ID(1) + 浮点数1(2) + 浮点数2(2)
						}
							else {
                // 未知命令，重置状态机
                uart_state = WAIT_HEAD1;
                break;
            }
            
            uart_state = WAIT_DATA;
            break;

        case WAIT_DATA:
            if (uart_index < sizeof(data)) {
                data[uart_index++] = byte;  // 保存数据部分
            }
            
            // 检查是否接收完预期长度的数据
            if (uart_index >= expected_data_len) {
                uart_state = WAIT_TAIL1;
            }
            break;

        case WAIT_TAIL1:
            if (byte == FRAME_TAIL1) {
                uart_state = WAIT_TAIL2;
            } else {
                uart_state = WAIT_HEAD1;  // 帧尾不匹配，重置
            }
            break;
            
        case WAIT_TAIL2:
            if (byte == FRAME_TAIL2) {
                uart_state = WAIT_TAIL3;
            } else {
                uart_state = WAIT_HEAD1;  // 帧尾不匹配，重置
            }
            break;
            
        case WAIT_TAIL3:
            if (byte == FRAME_TAIL3) {
                // 完整接收到一帧数据
                if (addr == 0x01) {  // 检查地址是否匹配本设备
                    switch (cmd) {
                        case CMD_PIXEL_COORD:
                            if (expected_data_len >= 1) {
                                uint8_t group_id = data[0];
                                process_coordinates(group_id, data + 1);
                            }
                            break;
                            
                        case CMD_TASK_ID:
                            if (expected_data_len >= 1) {
                                process_task_id(data[0]);
                            }
                            break;
												case CMD_FLOAT_DATA:  // 新增浮点数处理
														if (expected_data_len >= 1) {
																uint8_t group_id = data[0];
																process_float_data(group_id, data + 1);
														}
														break;
                    }
                }
            }
            uart_state = WAIT_HEAD1;  // 无论是否匹配都重置状态机
            break;
    }
}

void UART_0_INST_IRQHandler(void)
{
	
    static unsigned char uart0_data;
    // 如果产生了串口中断
    switch(DL_UART_getPendingInterrupt(UART_0_INST))
    {
		
        case DL_UART_IIDX_RX: // 如果是接收中断
            uart0_data = DL_UART_Main_receiveData(UART_0_INST);
            uart_receive_byte(uart0_data);
            break;

        default: // 其他的串口中断
            break;
    }
}