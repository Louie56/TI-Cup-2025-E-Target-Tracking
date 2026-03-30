#include "control.h"
#include "math.h"

#define Step_one 1 //单步步进值
#define PI 3.1415926535f

static float now_x_step = 0, now_y_step = 0;  //记忆当前步数

/*
 * x
 * -1对应x轴正方向
 */
void stepper_y_run(int tim,float step,float subdivide,int8_t dir)
{
  int i;
  if(step < 0.5)
    return;
  //控制方向
  if(dir == 1)
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_9); // 右转 
  else if(dir == -1)
    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_9); // 左转 
  //delay_ms(2); 有个延时
  
  for(i = 0; i < step; i++)
  {
    if(dir == -1)
      now_x_step--;
    else if(dir == 1)
      now_x_step++;
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6); // 右转 ;//翻转电平
    delay_ms((int)(tim / 2));//延时
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6); // 右转 ;//翻转电平
    delay_ms((int)(tim / 2));
  }
}
/*
 * y
 * 1对应y轴正方向
 */
void stepper_x_run(int tim, float step, float subdivide, int8_t dir)
{
  int i;
  if(step < 0.5)
    return;
  if(dir == 1)
    DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_3); // 上 B3 Y
  else if(dir == -1)
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_3); //
  delay_ms(2);
  for(i = 0; i < step; i++)
  {
    if(dir == 1)
      now_y_step++;
    else if(dir == -1)
      now_y_step--;

    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_8);
    delay_ms((int)(tim / 2));
    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_8);
    delay_ms((int)(tim / 2));
  }
}
/**
*定点函数
*/
void turn_coordinate(float x, float y)
{
  float angle_x, angle_y;
  float step_x, step_y;
  float dx, dy;
  float sqx;

  sqx = sqrt(1050 * 1050 + x * x);
  angle_x = atan(x / 1050) * 180 / PI;
  angle_y = atan(y / sqx) * 180 / PI;

  step_x = angle_x / 0.05625;//计算对应步数,与0相差
  step_y = angle_y / 0.05625;

  dx = step_x - now_x_step;
  dy = step_y - now_y_step;

  if(dx > 0)
    stepper_x_run(2, dx, 32, -1);
  else if(dx < 0)
    stepper_x_run(2, -dx, 32, 1);

  if(dy > 0)
    stepper_y_run(2, dy, 32, 1);
  else if(dy < 0)
    stepper_y_run(2, -dy, 32, -1);
}
/*
 * @brief：直线运动插补
 * @parameter：起点坐标（X0, Y0），终点坐标（Xe, Ye）
 * @return： 无
 * */
  float NXY;              //总步数
  float Fm = 0;           //偏差
  float Xm, Ym; //当前坐标
  uint8_t XOY;            //象限
void drawline(float X0, float Y0, float Xe, float Ye)
{

  Xm = X0, Ym = Y0;
  Xe = Xe - X0;
  Ye = Ye - Y0;
  NXY = (fabsf(Xe) + fabsf(Ye)) / Step_one;

  if(Xe > 0 && Ye >= 0) XOY = 1;
  else if(Xe <= 0 && Ye > 0) XOY = 2;
  else if(Xe < 0 && Ye <= 0) XOY = 3;
  else if(Xe >= 0 && Ye < 0) XOY = 4;

  while(NXY > 0)
  {
    switch (XOY)
    {
    case 1: (Fm >= 0) ? (Xm += Step_one) : (Ym += Step_one); break;
    case 2: (Fm <  0) ? (Xm -= Step_one) : (Ym += Step_one); break;
    case 3: (Fm >= 0) ? (Xm -= Step_one) : (Ym -= Step_one); break;
    case 4: (Fm <  0) ? (Xm += Step_one) : (Ym -= Step_one); break;
    default: break;
    }
    NXY -= 1;
    Fm = (Ym - Y0) * Xe - (Xm - X0) * Ye;
    turn_coordinate(Xm, Ym);
    delay_ms(2);
  }
}
/*
 * @brief：圆运动插补
 * @parameter：圆心坐标（x0, y0），半径 R, 方向 SorN 1 顺时针 2 逆时针
 * @return： 无
 * */
void drawcircle(float x0, float y0, float R, uint8_t SorN)
{
  float X0, Y0, Xe, Ye;
  float step = 0;
  float Fm = 0;
  float Xm, Ym;
  uint8_t XOY;

  X0 = x0; Y0 = y0 + R;  //开始点
  Xe = x0; Ye = y0 + R;  //结束点
  Xm = X0; Ym = Y0;

  while (pow((Xm - Xe), 2) + pow((Ym - Ye), 2) > Step_one * Step_one / 2 || (step == 0))
  {
    if ((Xm - x0) > 0 && (Ym - y0) >= 0) XOY = 1;
    else if ((Xm - x0) <= 0 && (Ym - y0) > 0) XOY = 2;
    else if ((Xm - x0) < 0 && (Ym - y0) <= 0) XOY = 3;
    else if ((Xm - x0) >= 0 && (Ym - y0) < 0) XOY = 4;

    switch (XOY)
    {
    case 1:
      if(SorN == 1)
        (Fm >= 0) ? (Ym -= Step_one) : (Xm += Step_one);
      else
        (Fm <= 0) ? (Ym += Step_one) : (Xm -= Step_one);
      break;
    case 2:
      if(SorN == 1)
        (Fm >= 0) ? (Xm += Step_one) : (Ym += Step_one);
      else
        (Fm >  0) ? (Ym -= Step_one) : (Xm -= Step_one);
      break;
    case 3:
      if(SorN == 1)
        (Fm >= 0) ? (Ym += Step_one) : (Xm -= Step_one);
      else
        (Fm >  0) ? (Xm += Step_one) : (Ym -= Step_one);
      break;
    case 4:
      if(SorN == 1)
        (Fm >= 0) ? (Xm -= Step_one) : (Ym -= Step_one);
      else
        (Fm >  0) ? (Ym += Step_one) : (Xm += Step_one);
    default: break;
    }
    step = step + 1;
    Fm = pow((Xm - x0), 2) + pow((Ym - y0), 2) - pow(R, 2);
    turn_coordinate(Xm, Ym);
    delay_ms(2);
  }
}
