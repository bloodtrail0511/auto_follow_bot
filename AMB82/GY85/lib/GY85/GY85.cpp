#include <GY85.h>
#include <Arduino.h>
#include <Wire.h>

void GY85::init()
{
    Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    write_register(ADXL345_ADDR, ADXL345_MODE, 0b00001000); //measure mode
    write_register(HMC5883L_ADDR, HMC5883L_MODE, 0b00000000); //measure mode
    write_register(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0b11111000);//75Hz
    write_register(HMC5883L_ADDR, HMC5883L_CONFIG_B, 0b01100000);//+- 2.5 Gauss
    write_register(ITG3205_ADDR, ITG3205_SMPLRT, 0b00001001);//1k/(9+1)=100Hz
    write_register(ITG3205_ADDR, ITG3205_DLPF, 0b00011011);//2000deg/sec LPF98Hz

    //磁力計校正
    float x, y, z, mag_x = 0, mag_y = 0, mag_z = 0;
    int sampling_times = 50;
    for(int i = 0; i < sampling_times; i++){
        cali_HMC5883L(&x, &z, &y);
        mag_x += x;
        mag_y += y;
        mag_z += z;
        delay(20);
    }
    mag_x/=sampling_times;
    mag_y/=sampling_times;
    mag_z/=sampling_times;
    mag_zero_roll = atan2(mag_y, mag_z)*180/PI;
    mag_zero_pitch = atan2(-mag_x, mag_z)*180/PI;
    mag_zero_yaw = atan2(mag_y, mag_x)*180/PI;

    //陀螺儀校正
    int16_t itgx, itgy, itgz;
    for(int i = 0; i < sampling_times; i++){
        read_ITG3205(&itgx, &itgy, &itgz);
        gyro_offset_x += float(itgx);
        gyro_offset_y += float(itgy);
        gyro_offset_z += float(itgz);
        delay(10);
    }
    gyro_offset_x /= sampling_times;
    gyro_offset_y /= sampling_times;
    gyro_offset_z /= sampling_times;

    last_time = millis();
}

void GY85::read_HMC5883L(float* X, float* Y, float* Z)
{
    Wire.beginTransmission(HMC5883L_ADDR);
    Wire.write(HMC5883L_DATA);
    Wire.endTransmission(false);

    Wire.requestFrom(HMC5883L_ADDR, (uint8_t)6);
    int16_t x, y, z;
    x = Wire.read()<<8 | Wire.read();
    y = Wire.read()<<8 | Wire.read();
    z = Wire.read()<<8 | Wire.read();
    *X = (float(x))/HMC5883L_GAIN;
    *Y = (float(y))/HMC5883L_GAIN;
    *Z = (float(z))/HMC5883L_GAIN;
}

// void GY85::cali_HMC5883L(float* X, float* Y, float* Z)
// {
//     read_HMC5883L(X, Y, Z);
//     // *X = (*X - x0)/root_A;
//     // *Y = (*Y - y0)/root_B;
//     // *Z = (*Z - z0)/root_C;
//     *X = (*X - x0);
//     *Y = (*Y - y0);
//     *Z = (*Z - z0);
//     *X = *X/root_a;
//     *Y = *Y/root_b;
//     *Z = *Z/root_c;
// }

void GY85::cali_HMC5883L(float* X, float* Y, float* Z)
{
    // 讀取原始數據
    float rawX, rawY, rawZ;
    read_HMC5883L(&rawX, &rawY, &rawZ);

    // 硬磁效應校正
    float correctedX = rawX - hardIronOffset[0];
    float correctedY = rawY - hardIronOffset[1];
    float correctedZ = rawZ - hardIronOffset[2];

    // 軟磁效應校正（矩陣變換）
    *X = softIronMatrix[0][0] * correctedX + softIronMatrix[0][1] * correctedY + softIronMatrix[0][2] * correctedZ;
    *Y = softIronMatrix[1][0] * correctedX + softIronMatrix[1][1] * correctedY + softIronMatrix[1][2] * correctedZ;
    *Z = softIronMatrix[2][0] * correctedX + softIronMatrix[2][1] * correctedY + softIronMatrix[2][2] * correctedZ;
}

void GY85::read_ADXL345(int16_t* X, int16_t* Y, int16_t* Z)
{
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(ADXL345_DATA);
    Wire.endTransmission(false);

    Wire.requestFrom(ADXL345_ADDR, (uint8_t)6);

    *X = Wire.read() | Wire.read()<<8;
    *Y = Wire.read() | Wire.read()<<8;
    *Z = Wire.read() | Wire.read()<<8;
}

void GY85::read_ITG3205(int16_t* X, int16_t* Y, int16_t* Z)
{
    Wire.beginTransmission(ITG3205_ADDR);
    Wire.write(ITG3205_DATA);
    Wire.endTransmission(false);

    Wire.requestFrom(ITG3205_ADDR, (uint8_t)6);

    *X = Wire.read()<<8 | Wire.read();
    *Y = Wire.read()<<8 | Wire.read();
    *Z = Wire.read()<<8 | Wire.read();
}

void GY85::cali_ITG3205(float* X, float* Y, float* Z)
{
    int16_t x, y, z;
    read_ITG3205(&x, &y, &z);
    *X = (float(x) - gyro_offset_x)/ITG3205_SENSITIVITY;
    *Y = (float(y) - gyro_offset_y)/ITG3205_SENSITIVITY;
    *Z = (float(z) - gyro_offset_z)/ITG3205_SENSITIVITY;
}

void GY85::update()
{
    // 時間管理
    unsigned long current_time = millis();
    float delta_t = (current_time - last_time) / 1000.; // 轉換為秒
    last_time = current_time;

    float mag_x, mag_y, mag_z;
    cali_HMC5883L(&mag_x, &mag_z, &mag_y);
    mag_roll = -(atan2(mag_y, mag_z) * 180.0 / PI - mag_zero_roll);  // 滾轉角
    mag_pitch = -(atan2(-mag_x, mag_z) * 180.0 / PI - mag_zero_pitch); // 俯仰角
    mag_yaw = atan2(mag_y, mag_x) * 180.0 / PI - mag_zero_yaw;   // 偏航角

    if (mag_roll >= 180) {mag_roll -= 360;}
    else if (mag_roll < -180) {mag_roll += 360;}
    if (mag_pitch >= 180) {mag_pitch -= 360;}
    else if (mag_pitch < -180) {mag_pitch += 360;}
    if (mag_yaw >= 180) {mag_yaw -= 360;}
    else if (mag_yaw < -180) {mag_yaw += 360;}

    // 讀取校正後的角速度數據
    float gyro_x, gyro_y, gyro_z;
    cali_ITG3205(&gyro_x, &gyro_y, &gyro_z);
    // 使用角速度進行積分計算角度
    gyro_roll -= gyro_x * delta_t;   // X 軸旋轉
    gyro_pitch += gyro_y * delta_t;  // Y 軸旋轉
    gyro_yaw -= gyro_z * delta_t;    // Z 軸旋轉

    if (gyro_roll >= 180) gyro_roll -= 360;
    else if (gyro_roll < -180) gyro_roll += 360;
    if (gyro_pitch >= 180) gyro_pitch -= 360;
    else if (gyro_pitch < -180) gyro_pitch += 360;
    if (gyro_yaw >= 180) gyro_yaw -= 360;
    else if (gyro_yaw < -180) gyro_yaw += 360;

    roll_angle = alpha * (roll_angle - gyro_x * delta_t) + (1 - alpha) * mag_roll;
    pitch_angle = alpha * (pitch_angle + gyro_y * delta_t) + (1 - alpha) * mag_pitch;
    yaw_angle = alpha * (yaw_angle - gyro_z*delta_t) + (1 - alpha) * mag_yaw;
}


void GY85::write_register(uint8_t device, uint8_t reg, uint8_t msg)
{
    Wire.beginTransmission(device);
    Wire.write(reg);
    Wire.write(msg);
    Wire.endTransmission();
}