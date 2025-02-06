

void setPwm(float Ua, float Ub, float Uc);
float setTorque(float Uq,float angle_el);
float _normalizeAngle(float angle);
void DFOC_Vbus(float power_supply);
void DFOC_alignSensor(int _PP,int _DIR);
float _electricalAngle();

float serial_motor_target();
String serialReceiveUserCommand();
//传感器读取
float DFOC_M0_Velocity();
float DFOC_M0_Angle();
//PID
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp);
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp);
float DFOC_M0_VEL_PID(float error);
float DFOC_M0_ANGLE_PID(float error);
//接口函数
void DFOC_M0_set_Velocity_Angle(float Target);
void DFOC_M0_setVelocity(float Target);
void DFOC_M0_set_Force_Angle(float Target);
void DFOC_M0_setTorque(float Target);
