
float kp_ = 0.085, ki_ = 0.08, kd_ = 0.015;

const int time_ = 50;
const int isr_pin = 4;
long counter = 0;
long time_now = 0;
long print_time = 0;

unsigned long debounce = 0;
long prevCount = 0;
float velocity = 0;
long deltaCount = 0;

double setpoint_ = 0.0, ip_, op_;
const float enc_to_wheel_ratio = 0.017543;
int f = 0;
int PID_f = 0;
float servo_ang = 0.0;
bool flag = true;
// int flag;
