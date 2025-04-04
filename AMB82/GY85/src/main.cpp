#include <Arduino.h>
#include <Wire.h>
#include <GY85.h>

GY85 gy85;
hw_timer_t *timer = NULL;
volatile bool flag = false;
void ARDUINO_ISR_ATTR p(){flag = true;}

void setup() {
  Serial.begin(115200);

    
    timer = timerBegin(0, 80, true);// 80MHz/80 = 1MHz
    timerAttachInterrupt(timer, &p, true);//設定中斷函數
    timerAlarmWrite(timer, 20000, true);// 50Hz
    timerAlarmEnable(timer);// 中斷始能



  gy85.init();
}

void loop() {
    if(flag)
    {
        gy85.update();

        // printf("%.2f\t\t%.2f\n", gy85.mag_xy, gy85.xy_angle);
        // printf("x: %.4f\t\ty: %.4f\t\tmag: %.4f\t\txy: %.4f\n", gy85.mag_roll, gy85.mag_pitch, gy85.mag_yaw, gy85.mag_xy);
        // printf("x: %.4f\t\ty: %.4f\t\tmag: %.4f\t\txy: %.4f\n", gy85.acc_x_angle, gy85.acc_y_angle, gy85.mag_yaw, gy85.mag_xy);

        float X, Y, Z;
        gy85.read_HMC5883L(&X, &Y, &Z);
        // gy85.cali_HMC5883L(&X, &Y, &Z);
        printf("%f,%f,%f\n", X, Y, Z);
        // printf("%f\n", gy85.yaw_angle);

        // printf("%.4f\t\t%.4f\n", gy85.acc_x_angle, gy85.acc_y_angle);
        // float X, Y, Z;
        // gy85.cali_ADXL345(&X, &Y, &Z);
        // printf("%.4f\t\t%.4f\t\t%.4f\n", X, Y, Z);

        flag = false;
    }
}