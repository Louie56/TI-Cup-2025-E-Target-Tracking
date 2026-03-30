#include "motor.h"
#include "time.h"
#include "ti_msp_dl_config.h"
#include "math.h"
#define M_PI 3.1415926

volatile int g_current_period_us = 7000; // 当前电机脉冲周期，初始为低速（大周期）
volatile int g_current_x_error = 0;      // X轴累积误差，用于步进插补
volatile int g_current_y_error = 0;      // Y轴累积误差，用于步进插补
// DDA 累积误差：用于实现直线插补。它们也需要是全局的，以便连续运动时能保持位置连续。
volatile int g_dda_x_accumulator = 0; // X轴DDA误差累加器
volatile int g_dda_y_accumulator = 0; // Y轴DDA误差累加器
// 全局的加减速参数，也可以作为函数参数传入
const int GLOBAL_RAMP_STEPS = 300; // 加减速步数，越大越平滑
const int GLOBAL_PERIOD_MAX = 7000; // 启动/停止时的最低速度（最大脉冲周期）
const int GLOBAL_PERIOD_MIN = 500; // 启动/停止时的最低速度（最大脉冲周期）

int targetx_steps;  // X 目标步数
int currentx_steps = 0;   // X 当前走了多少步

int targety_steps;  // Y 目标步数
int currenty_steps = 0;//Y 当前走了多少步


int step_countx;
int step_county;

int distance;



void TwoAxis_Move(int steps_x, int steps_y, int period_min, int period_max, int ramp_steps)
{
int i = 0;
    int total_steps = (steps_x > steps_y) ? steps_x : steps_y; // 总共多少步，以较大者为主

    int x_count = 0, y_count = 0;
    int x_error = 0, y_error = 0;

    // 确保 ramp_steps 不会超过总步数的一半，以免匀速段消失或出现问题
    if (ramp_steps * 2 > total_steps) {
        ramp_steps = total_steps / 2;
        if (ramp_steps == 0 && total_steps > 0) ramp_steps = 1; // 至少1步加减速
    }
    if (total_steps == 0) return; // 没有步数则直接返回

    // 计算速度范围
    int period_range = period_max - period_min;

    for (i = 0; i < total_steps; i++)
    {
        int delay_us_val;
        double progress; // 用于计算S型曲线的进度

        if (i < ramp_steps) // 加速阶段
        {
            // 加速阶段的S型近似：从period_max逐渐减小到period_min
            // 使用一个非线性（例如二次方）函数让速度变化更平滑
            // progress 从 0.0 到 1.0
            progress = (double)i / ramp_steps;
            // 调整 progress，使其在初期变化慢，后期变化快 (例如 1 - (1-x)^2)
            // 这里的 (1.0 - progress) * (1.0 - progress) 使得延时变化在初期缓慢
            delay_us_val = period_max - (int)(period_range * (1.0 - (1.0 - progress) * (1.0 - progress)));

            // 确保延时不会低于 period_min
            if (delay_us_val < period_min) delay_us_val = period_min;
        }
        else if (i < total_steps - ramp_steps) // 匀速阶段
        {
            delay_us_val = period_min;
        }
        else // 减速阶段
        {
            // 减速阶段的S型近似：从period_min逐渐增大到period_max
            int dec_i = total_steps - i; // 减速阶段的当前步数，从 ramp_steps 递减到 1
            // progress 从 0.0 到 1.0 (反向进度)
            progress = (double)dec_i / ramp_steps;
            // 调整 progress，使其在末期变化慢，初期变化快 (例如 x^2)
            // 这里的 progress * progress 使得延时变化在末期缓慢
            delay_us_val = period_max - (int)(period_range * (progress * progress));

            // 确保延时不会低于 period_min
            if (delay_us_val < period_min) delay_us_val = period_min;
        }

		// 2. 控制 X 电机
		x_error += steps_x;
		if (x_error >= total_steps)
		{
			DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);   // X STEP
			delay_1us(1);
			DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
			x_error -= total_steps;
			x_count++;
		}
		
		// 3. 控制 Y 电机
		y_error += steps_y;
		if (y_error >= total_steps)
		{
			DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6);   // Y STEP
			delay_1us(1);
			DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);
			y_error -= total_steps;
			y_count++;
		}

		delay_1us(delay_us_val); // 控制整体节奏
	}
}

void TwoAxis_Movetest_Continuous(int target_steps_x, int target_steps_y, int target_period_min)
{
    // 如果目标步数为0，直接返回，避免不必要的计算
    if (target_steps_x == 0 && target_steps_y == 0) {
        return;
    }

    // 1. 确定本次运动的总步数 (以较大者为准)
    int total_movement_steps = (target_steps_x > target_steps_y) ? target_steps_x : target_steps_y;
    
    // 确保加减速步数不超过总步数的一半
    int actual_ramp_steps = GLOBAL_RAMP_STEPS;
    if (actual_ramp_steps * 2 > total_movement_steps) {
        actual_ramp_steps = total_movement_steps / 2;
        if (actual_ramp_steps == 0 && total_movement_steps > 0) actual_ramp_steps = 1; // 至少1步加减速
    }

    // 计算本次运动的速度范围
    // 注意：这里的 period_range 应该根据全局速度和目标速度来计算
    // 简化处理：假设目标速度总是在 period_min 到 GLOBAL_PERIOD_MAX 之间
    int period_range_current_move = GLOBAL_PERIOD_MAX - target_period_min;


    // 2. 运动规划循环
    // 这个循环将负责计算每一“虚拟”步的脉冲周期，并生成脉冲
    for (int i = 0; i < total_movement_steps; i++)
    {
        unsigned int calculated_delay_us; // 当前步的计算延时

        // S型加速阶段
        if (i < actual_ramp_steps)
        {
            double progress = (double)i / actual_ramp_steps;
            // 五次多项式 S 型曲线，加速阶段延时从大到小
            double s_curve_val = 6 * pow(progress, 5) - 15 * pow(progress, 4) + 10 * pow(progress, 3);
            calculated_delay_us = GLOBAL_PERIOD_MAX - (unsigned int)(s_curve_val * period_range_current_move);

            // 确保不低于目标最小周期
            if (calculated_delay_us < target_period_min) calculated_delay_us = target_period_min;
            // 确保不超过最大周期
            if (calculated_delay_us > GLOBAL_PERIOD_MAX) calculated_delay_us = GLOBAL_PERIOD_MAX;
        }
        // 匀速阶段
        else if (i < total_movement_steps - actual_ramp_steps)
        {
            calculated_delay_us = target_period_min;
        }
        // S型减速阶段
        else
        {
            int dec_i = total_movement_steps - i; // 减速阶段的剩余步数，从 actual_ramp_steps 递减到 1
            double progress = (double)dec_i / actual_ramp_steps;
            // 五次多项式 S 型曲线，减速阶段延时从小到大
            double s_curve_val = 6 * pow(progress, 5) - 15 * pow(progress, 4) + 10 * pow(progress, 3);
            calculated_delay_us = GLOBAL_PERIOD_MAX - (unsigned int)(s_curve_val * period_range_current_move);

            // 确保不低于目标最小周期
            if (calculated_delay_us < target_period_min) calculated_delay_us = target_period_min;
            // 确保不超过最大周期
            if (calculated_delay_us > GLOBAL_PERIOD_MAX) calculated_delay_us = GLOBAL_PERIOD_MAX;
        }

        // 3. 实际生成脉冲并控制节奏
        // 关键点：这里使用 g_current_period_us 来记录当前的速度
        // 而不是直接用 calculated_delay_us，以便下一个函数调用能继承这个速度。
        g_current_period_us = calculated_delay_us; 

        // 2. 控制 X 电机
        g_current_x_error += target_steps_x;
        if (g_current_x_error >= total_movement_steps)
        {
            DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6);    // X STEP
            delay_1us(1); // 脉冲宽度
            DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);
            g_current_x_error -= total_movement_steps;
        }
        
        // 3. 控制 Y 电机
        g_current_y_error += target_steps_y;
        if (g_current_y_error >= total_movement_steps)
        {
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);    // Y STEP
            delay_1us(1); // 脉冲宽度
            DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
            g_current_y_error -= total_movement_steps;
        }

        // 每次循环等待计算出的当前脉冲周期
        delay_us(g_current_period_us);
    }

    // 运动结束后，为了下一个运动能平滑衔接，我们不直接将速度降到最低，而是保持当前速度
    // 但为了确保停止时的平稳，可能还需要一个额外的减速到停的函数
}

// 新增：一个强制电机完全停止并减速到GLOBAL_PERIOD_MAX的函数
void Motor_Stop_Smoothly() {
    // 假设从当前速度 g_current_period_us 减速到 GLOBAL_PERIOD_MAX (最低速)
    // 这里的逻辑可以简化，也可以使用一个小的 S 型曲线来减速。
    // 为了简单，我们只做几步减速
    int steps_to_stop = 50; // 用50步来减速
    int period_range = GLOBAL_PERIOD_MAX - g_current_period_us;
    if (period_range < 0) period_range = 0; // 避免负值

    for (int i = 0; i < steps_to_stop; i++) {
        unsigned int current_stop_period = g_current_period_us + (unsigned int)((double)period_range * i / steps_to_stop);
        if (current_stop_period > GLOBAL_PERIOD_MAX) current_stop_period = GLOBAL_PERIOD_MAX; // 限制不超过最大周期

        // 生成脉冲，但这里没有考虑具体轴的步数，只是为了模拟减速过程
        // 在实际应用中，您需要修改 MotorController_ProcessStep 这样细粒度的函数
        // 来处理每个轴的步进。此处为简化示例。
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6); // X STEP 示例
        delay_1us(1);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);
        
        delay_us(current_stop_period);
        g_current_period_us = current_stop_period; // 更新当前速度
    }
    g_current_period_us = GLOBAL_PERIOD_MAX; // 最终回到最低速
    g_current_x_error = 0; // 清除累积误差
    g_current_y_error = 0;
}

void TwoAxis_Move_Line(int target_x_steps, int target_y_steps, unsigned int target_speed_us)
{
    // 如果目标步数为0，直接返回，避免不必要的计算
    if (target_x_steps == 0 && target_y_steps == 0) {
        return;
    }

    // --- 1. 确定本次直线运动的总“插补”步数 ---
    // 这个“总插补步数”是DDA算法的关键，它应该是X和Y步数中绝对值最大的那个。
    // 这代表了直线路径上的最大“虚拟”步数。
    int total_interpolation_steps = ((target_x_steps) > (target_y_steps)) ? (target_x_steps) : (target_y_steps);

    // 确保加减速步数不超过总插补步数的一半
    int actual_ramp_steps = GLOBAL_RAMP_STEPS;
    if (actual_ramp_steps * 2 > total_interpolation_steps) {
        actual_ramp_steps = total_interpolation_steps / 2;
        if (actual_ramp_steps == 0 && total_interpolation_steps > 0) actual_ramp_steps = 1; // 至少1步加减速
    }

    // 记录本次运动开始时的速度，用于 S 型曲线的平滑衔接
    unsigned int start_period_for_ramp = g_current_period_us;

    // 计算从当前速度到目标速度的速度范围
    // 例如，如果从100us加速到50us，这个范围是50us
    // 如果从50us减速到80us，这个范围是-30us（取绝对值计算）
    int period_diff_for_ramp = (int)start_period_for_ramp - (int)target_speed_us;

    // --- DDA 算法的方向和误差积累初始化 ---
    // DDA的误差累加器 g_dda_x_accumulator 和 g_dda_y_accumulator 是全局的，
    // 这样在连续调用时，它们会保持累积，理论上能保证连续路径的精确性。
    // 但是，每次新的直线运动开始时，为了避免前一条路径的累积误差影响新的直线方向，
    // 通常会**在每次调用 MoveTwoAxis_Line 时将它们清零**。
    // 在这里我们选择清零，以确保每次函数调用都生成一条独立且准确的直线。
    // 如果需要更高级的路径融合（例如从一条线平滑过渡到另一条线而不仅仅是速度平滑），
    // 那么DDA累加器也需要参与融合，但那会复杂得多。
    g_dda_x_accumulator = 0;
    g_dda_y_accumulator = 0;

    // 设定X和Y轴的运动方向
    // 需要根据您的硬件连接，设置GPIO引脚为高或低来控制方向
    // 例如：
    // if (target_x_steps > 0) DL_GPIO_setPins(X_DIR_PORT, X_DIR_PIN); else DL_GPIO_clearPins(X_DIR_PORT, X_DIR_PIN);
    // if (target_y_steps > 0) DL_GPIO_setPins(Y_DIR_PORT, Y_DIR_PIN); else DL_GPIO_clearPins(Y_DIR_PORT, Y_DIR_PIN);
    // 假设您的方向引脚是 GPIOB, DL_GPIO_PIN_7 for X 和 GPIOA, DL_GPIO_PIN_9 for Y
    if (target_x_steps > 0) {
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_7); // X正方向
    } else {
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_7); // X负方向
    }
    if (target_y_steps > 0) {
        DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_9); // Y正方向
    } else {
        DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_9); // Y负方向
    }


    // --- 2. 运动规划循环：每循环一次，代表一个“通用步进机会” ---
    // 这个循环会执行 total_interpolation_steps 次，确保所有轴都按比例移动到位
    for (int i = 0; i < total_interpolation_steps; i++)
    {
        unsigned int calculated_pulse_period_us; // 当前步的计算延时（脉冲周期）

        // --- S型速度曲线计算 ---
        // 目标：让速度从 start_period_for_ramp 平滑地过渡到 target_speed_us
        // 并在减速阶段平滑过渡到 MAX_PERIOD_US
        
        if (i < actual_ramp_steps) // 加速阶段
        {
            // 进度从 0.0 到 1.0
            double progress = (double)i / actual_ramp_steps;
            // 五次多项式 S 型曲线函数 (0到1的映射)，值从0到1，前期慢，后期快
            double s_curve_factor = 6 * pow(progress, 5) - 15 * pow(progress, 4) + 10 * pow(progress, 3);
            
            // 计算当前脉冲周期：从起始周期 (慢速) 向目标周期 (高速) 变化
            // s_curve_factor 越大，表示速度越接近目标速度，脉冲周期越小
            calculated_pulse_period_us = start_period_for_ramp - (unsigned int)(period_diff_for_ramp * s_curve_factor);

            // 确保计算结果在合理范围内
            if (calculated_pulse_period_us < target_speed_us) calculated_pulse_period_us = target_speed_us;
            if (calculated_pulse_period_us > start_period_for_ramp) calculated_pulse_period_us = start_period_for_ramp; // 防止超出起始周期
        }
        else if (i < total_interpolation_steps - actual_ramp_steps) // 匀速阶段
        {
            calculated_pulse_period_us = target_speed_us;
        }
        else // 减速阶段
        {
            // 减速阶段的剩余步数，从 actual_ramp_steps 递减到 1
            int dec_i = total_interpolation_steps - i;
            double progress = (double)dec_i / actual_ramp_steps;
            // 五次多项式 S 型曲线，值从0到1，但表示的是从当前速度减速到 MAX_PERIOD_US
            // s_curve_factor 越大，表示越接近结束，速度越慢，脉冲周期越大
            double s_curve_factor = 6 * pow(progress, 5) - 15 * pow(progress, 4) + 10 * pow(progress, 3);
            
            // 计算当前脉冲周期：从目标速度 (高速) 向 MAX_PERIOD_US (最低速) 变化
            // (1.0 - s_curve_factor) 使得在减速开始时变化快，结束时变化慢
            calculated_pulse_period_us = target_speed_us + (unsigned int)((GLOBAL_PERIOD_MAX - target_speed_us) * (1.0 - s_curve_factor));

            // 确保计算结果在合理范围内
            if (calculated_pulse_period_us < target_speed_us) calculated_pulse_period_us = target_speed_us;
            if (calculated_pulse_period_us > GLOBAL_PERIOD_MAX) calculated_pulse_period_us = GLOBAL_PERIOD_MAX;
        }

        // --- 3. 更新全局当前速度 ---
        // 这是实现连续运动速度衔接的关键！
        g_current_period_us = calculated_pulse_period_us; 

        // --- 4. DDA 步进逻辑：根据误差累积判断哪个轴步进 ---
        // X轴 DDA 累加：每次循环累加X轴的目标绝对步数
        g_dda_x_accumulator += (target_x_steps);
        // 如果 X轴累积误差 >= 总插补步数，则X轴步进
        if (g_dda_x_accumulator >= total_interpolation_steps) {
            DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);    // X STEP 脉冲高
            delay_1us(1);                             // 脉冲宽度
            DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);  // X STEP 脉冲低
            g_dda_x_accumulator -= total_interpolation_steps; // 减去一个周期
        }

        // Y轴 DDA 累加：每次循环累加Y轴的目标绝对步数
        g_dda_y_accumulator += (target_y_steps);
        // 如果 Y轴累积误差 >= 总插补步数，则Y轴步进
        if (g_dda_y_accumulator >= total_interpolation_steps) {
            DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6);    // Y STEP 脉冲高
            delay_1us(1);                             // 脉冲宽度
            DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);  // Y STEP 脉冲低
            g_dda_y_accumulator -= total_interpolation_steps; // 减去一个周期
        }

        // --- 5. 控制整体节奏 ---
        // 每次循环等待当前计算出的脉冲周期。这决定了实际的速度。
        delay_us(g_current_period_us); 
    }
}



void TwoAxis_Movetest(int steps_x, int steps_y, int period_min, int period_max, int ramp_steps)
{
	int i = 0;
	int total_steps = (steps_x > steps_y) ? steps_x : steps_y; // 总共多少步，以较大者为主

	int x_count = 0, y_count = 0;
	int x_error = 0, y_error = 0;

	for (i = 0; i < total_steps; i++)
	{
		int delay_us_val;
		if (i < ramp_steps)//加速
		{
			delay_us_val = period_max - ((period_max - period_min) * i / ramp_steps);
		}
		else if (i < total_steps - ramp_steps)//匀速
		{
			delay_us_val = period_min;
		}
		else//减速
		{
			int dec_i = total_steps - i;
			delay_us_val = period_max - ((period_max - period_min) * dec_i / ramp_steps);
		}

		// 2. 控制 X 电机
		x_error += steps_x;
	
		if (x_error >= total_steps)
		{
			DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6);   // X STEP
			delay_1us(1);
			DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);
			x_error -= total_steps;
			x_count++;
		}
		
		// 3. 控制 Y 电机
		y_error += steps_y;
		if (y_error >= total_steps)
		{
			DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);   // Y STEP
			delay_1us(1);
			DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
			y_error -= total_steps;
			y_count++;
		}

		delay_us(delay_us_val); // 控制整体节奏
	}
}

void StepPulse6(void)
{
			DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6);   // X STEP
    delay_1us(1);
			DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);
}
void StepPulse8(void)
{
			DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);   // Y STEP
    delay_1us(1);
			DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
}
void TwoAxis_Movetest1(int steps_x, int steps_y, int period_min, int period_max, int ramp_steps)
{
    int total_steps = (steps_x > steps_y) ? steps_x : steps_y;
    int period_diff = period_max - period_min;

    double x_error = 0.0, y_error = 0.0;

    for (int i = 0; i < total_steps; i++)
    {
        int delay_us_val;

        if (i < ramp_steps) // S 型加速
        {
            float t = (float)i / ramp_steps;
            float s = 0.5f * (1 - cosf(M_PI * t));
            delay_us_val = period_max - (int)(period_diff * s);
        }
        else if (i < total_steps - ramp_steps) // 匀速
        {
            delay_us_val = period_min;
        }
        else // S 型减速
        {
            int dec_i = total_steps - i;
            float t = (float)dec_i / ramp_steps;
            float s = 0.5f * (1 - cosf(M_PI * t));
            delay_us_val = period_max - (int)(period_diff * s);
        }

        // 插补补偿（浮点累加器）
        x_error += (double)steps_x;
        if (x_error >= total_steps)
        {
            StepPulse6();  // X STEP
            x_error -= total_steps;
        }

        y_error += (double)steps_y;
        if (y_error >= total_steps)
        {
            StepPulse8();  // Y STEP
            y_error -= total_steps;
        }

        delay_us(delay_us_val);
    }
}



void movy(int dir)
{
	if(dir==1)
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_9); // upy
	if(dir==-1)
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_9); // downy
	if(dir!=0)
	{
		DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6);   
			delay_1us(1);
			DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);
	}
			DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);

}

void movx(int dir)
{
	if(dir==1)
			DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_3); // leftx
	if(dir==-1)
			DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_3); // leftx
	
	if(dir!=0)
	{	
		DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);   // Y STEP
		delay_1us(1);
		DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
	}
			DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);

}
