#include <math.h>

#define PI_OVER_4 0.7853982
#define RADS2RPM 9.5492966
#define CAR_WIDTH 38 // cm
#define CAR_LENGTH 25 // cm
#define CAR_WHEEL_RADIUS 5.45 //CM
#define SQRT2 1.41421356

struct Mecanum_str{
    float vel_x;    // cm/s 平移速度x
    float vel_y;    // cm/s 平移速度y
    float omega;    // rad/s    自轉角速度(+Z, 從上往下看是逆時針)
    float left_front;   // rpm              
    float left_rear;    // rpm
    float right_front;  // rpm
    float right_rear;   // rpm              
};
/*
輪子旋轉軸: +y 
車頭: +x
      +     +
   1 /   x   \ 2
    - y__|    -
    +         +
   3 \       / 4
      -     - 
*/

#define MECANUM_INIT(name)\
    name.vel_x = 0;\
    name.vel_y = 0;\
    name.omega = 0;\
    name.left_front = 0;\
    name.left_rear = 0;\
    name.right_front = 0;\
    name.right_rear = 0;\

void Mecanum_calc_speed(struct Mecanum_str *M);