#include <Arduino.h>
#include "DengFOC.h"

int Sensor_DIR=1;    //传感器方向
int Motor_PP=7;    //电机极对数

void setup() {
  Serial.begin(115200);
  DFOC_Vbus(12.6);   //设定驱动器供电电压
  DFOC_alignSensor(Motor_PP,Sensor_DIR);
  pinMode(21, OUTPUT);  // 配置启用引脚
  digitalWrite(21, HIGH);  // 启用电机
    pinMode(33, OUTPUT);  // 配置启用引脚
  digitalWrite(33, HIGH);  // 启用电机
}


void loop() 
{
  //设置速度环PID
   DFOC_M0_SET_VEL_PID(0.001,0.00,0,0);
  DFOC_M0_SET_ANGLE_PID(0.2,0.00,0,0);

  //设置速度
   DFOC_M0_setVelocity(serial_motor_target());
   //DFOC_M0_set_Force_Angle(serial_motor_target());
  //接收串口
  serialReceiveUserCommand();
  // Serial.print("当前角度：");
  // Serial.println(DFOC_M0_Angle());
}