#ifndef GY85_H
#define GY85_H

#include <Arduino.h>
#include <Wire.h>

class GY85
{
    public:
        void init();
        void read_HMC5883L(float* X, float* Y, float* Z);//磁力計
        void cali_HMC5883L(float* X, float* Y, float* Z);//磁力計橢球校正
        void read_ADXL345(int16_t* X, int16_t* Y, int16_t* Z);//加速度計
        void read_ITG3205(int16_t* X, int16_t* Y, int16_t* Z);//陀螺儀(角速度)
        void cali_ITG3205(float* X, float* Y, float* Z);//陀螺儀(角速度)offset校正
        float get_mag_yaw();
        float get_gyro_yaw();
        float get_angle();
        void update();
        void write_register(uint8_t device, uint8_t reg, uint8_t msg);

        float roll_angle = 0;   // 滾轉角（X 軸旋轉）
        float pitch_angle = 0;  // 俯仰角（Y 軸旋轉）
        float yaw_angle = 0;    // 偏航角（Z 軸旋轉）
        // 磁力計角度
        float mag_roll;
        float mag_pitch;
        float mag_yaw;
        // 陀螺儀角度
        float gyro_roll = 0;
        float gyro_pitch = 0;
        float gyro_yaw = 0;

    private:
        uint8_t I2C_MASTER_SDA_IO = 21;
        uint8_t I2C_MASTER_SCL_IO = 22;
        int I2C_MASTER_FREQ_HZ = 100000;
        uint8_t ADXL345_ADDR = 0x53;
        uint8_t ADXL345_MODE = 0x2D;
        uint8_t ADXL345_DATA = 0x32;
        uint8_t HMC5883L_ADDR = 0x1E;
        uint8_t HMC5883L_CONFIG_A = 0x00;
        uint8_t HMC5883L_CONFIG_B = 0x01;
        uint8_t HMC5883L_MODE = 0x02;
        uint8_t HMC5883L_DATA = 0x03;
        uint8_t ITG3205_ADDR = 0x68;
        uint8_t ITG3205_SMPLRT = 0x15;
        uint8_t ITG3205_DLPF = 0x16;
        uint8_t ITG3205_DATA = 0x1D;

        float HMC5883L_GAIN = 660.;
        float ITG3205_SENSITIVITY = 14.375;


        // // 硬磁效應偏移量
        float hardIronOffset[3] = {0.13481,0.12513,-0.080605}; // MATLAB 計算的偏移量

        // 軟磁效應比例矩陣
        float softIronMatrix[3][3] = {
            {0.93565,0.075208,0.0089123},
            {0.075208,1.1061,-0.014323},
            {0.0089123,-0.014323,0.97188}
        };

        ////////////////only esp32///////////////////
        // 硬磁效應偏移量
//         float hardIronOffset[3] = {-0.065309,-0.12876,-0.40245}; // MATLAB 計算的偏移量

//         // 軟磁效應比例矩陣
//         float softIronMatrix[3][3] = {
// {0.99755,0.13673,-0.013348},
// {0.13673,1.0428,0.0049955},
// {-0.013348,0.0049955,0.97917}
// // {0.99755,0,0},
// // {0,1.0428,0},
// // {0,0,0.97917}
//         };
        ////////////////only esp32///////////////////


        //陀螺儀(ITG3205)offset
        float gyro_offset_x = 0;
        float gyro_offset_y = 0;
        float gyro_offset_z = 0;
        unsigned long last_time = 0;

        //磁力計零角度
        float mag_zero_roll = 0;
        float mag_zero_pitch = 0;
        float mag_zero_yaw = 0;

        //互補濾波係數
        float alpha = 0.98;








        // //橢球擬合參數                 A            B           C           D           E           F           G
        // float HMC5883L_CALI[7] = {0.40002242,  0.53460796,  0.41470263, -0.60011751,  0.04687447,
        // 0.07448111,  0.11963459};
        // float x0 = HMC5883L_CALI[3]/(-2*HMC5883L_CALI[0]);
        // float y0 = HMC5883L_CALI[4]/(-2*HMC5883L_CALI[1]);
        // float z0 = HMC5883L_CALI[5]/(-2*HMC5883L_CALI[2]);
        // float root_a = pow(HMC5883L_CALI[1]*HMC5883L_CALI[2]/HMC5883L_CALI[0], 0.125);
        // float root_b = pow(HMC5883L_CALI[0]*HMC5883L_CALI[2]/HMC5883L_CALI[1], 0.125);
        // float root_c = pow(HMC5883L_CALI[0]*HMC5883L_CALI[1]/HMC5883L_CALI[2], 0.125);
};


#endif