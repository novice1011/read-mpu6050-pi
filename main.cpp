#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <math.h>

#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

const double PI = 3.14159265359;

void MPU6050_Init(int fd){

    wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
    wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
    wiringPiI2CWriteReg8 (fd, CONFIG, 0);		/* Write to Configuration register */
    wiringPiI2CWriteReg8 (fd, ACCEL_CONFIG, 0b00011000);	/* Write to Accel Configuration register using 16g sensitivity*/
    wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register */
    }

int16_t read_raw_data(int fd, int addr){
    int16_t high_byte,low_byte,value;
    high_byte = wiringPiI2CReadReg8(fd, addr);
    low_byte = wiringPiI2CReadReg8(fd, addr+1);
    value = (high_byte << 8) | low_byte;
    return value;
}

void ms_delay(int val){
    int i,j;
    for(i=0;i<=val;i++)
        for(j=0;j<1200;j++);
}

int main(){

    int16_t Acc_x,Acc_y,Acc_z;
    double Ax=0, Ay=0, Az=0;
    int fd = wiringPiI2CSetup(Device_Address);   /*Initializes I2C with device Address*/
    MPU6050_Init(fd);		                 /* Initializes MPU6050 */
    unsigned int old = millis();
    int timertilt=0;

    while(1)
    {
        if (millis()-old > 1000){
            /*Read raw value of Accelerometer and gyroscope from MPU6050*/
            Acc_x = read_raw_data(fd, ACCEL_XOUT_H);
            Acc_y = read_raw_data(fd, ACCEL_YOUT_H);
            Acc_z = read_raw_data(fd, ACCEL_ZOUT_H);

            /* Divide raw value by sensitivity scale factor */
            Ax = (double)Acc_x/32768*16;
            Ay = (double)Acc_y/32768*16;
            Az = (double)Acc_z/32768*16;

            double pitchA    = atan2(Ay,Az)/PI*180;
            double rollA     = atan2(Ax,Az)/PI*180;
            if (abs(pitchA)>45 || abs(rollA)>45){
                timertilt++;
                printf("danger\n");
            }
            else {
                timertilt=0;
            }
            if (timertilt>10){
                printf("calling ambulance\n");
                timertilt = 0;
            }

            printf("pitchA:%.2f \t rollA:%.2f\n",pitchA,rollA);
            old = millis();
        }
    }
    return 0;
}

//REFF:
//http://www.raspberrypirobotics.com/mpu6050-interfacing-with-raspberry-pi-using-c/
//https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
