#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
//#include "stm32f4xx_hal_def.h"
#include <stm32f4xx_hal_uart.h>

struct data{
    x;
    y;
    z;
}

float getPitch(float _pitch, data gyroscope, data accelerometer, float _pitchGyroFavoring, float _filterUpdateRate){
    float pitchFromAccel = 0;
    pitchFromAccel = atan(-accelerometer.x / sqrt(pow(accelerometer.y, 2) + pow(accelerometer.z, 2)));
    _pitch = (_pitchGyroFavoring) * (_pitch + (gyroscope.y * (1.00 / _filterUpdateRate))) + (1.00 - _pitchGyroFavoring) * (pitchFromAccel);
    return _pitch;	
}

float getRoll(float _roll, data gyroscope, data accelerometer, float _rollGyroFavoring, float _filterUpdateRate){
    float rollFromAccel  = 0;
    rollFromAccel = atan(accelerometer.y / sqrt(pow(accelerometer.x, 2) + pow(accelerometer.z, 2)));
    // rollFromAccel = atan2(accelerometer.y, accelerometer.z);
    _roll = (_rollGyroFavoring) * (_roll + (gyroscope.x * (1.00 / _filterUpdateRate))) + (1.00 - _rollGyroFavoring) * (rollFromAccel);
    return _roll;

}

void updateAccelData(data &accelerometer){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*uint8_t buf[10]= {0x0A};//IRA_REG_M returns 0x48
	  int16_t ret = HAL_I2C_Master_Transmit(&hi2c1, SAD_W_M, &buf[0], 1, 1000);

	  // should get 0100 1000 back
	  ret =  HAL_I2C_Master_Receive(&hi2c1, SAD_R_M, &buf[0], 1, 1000);*/

	  // Writes CTRL_REG1_A
	  uint8_t buf[10] = {CTRL_REG1_A, 0b10010111};
	  int32_t ret = HAL_I2C_Master_Transmit(&hi2c1, SAD_W_A, &buf[0], 2, 1000);
	  if (ret != HAL_OK) {ret = 1;}

	  // Writes CTRL_REG2_A
	  uint8_t buf2[10] = {CTRL_REG2_A, 0b00000000};
	  ret = HAL_I2C_Master_Transmit(&hi2c1, SAD_W_A, &buf2[0], 2, 1000);
	  if (ret != HAL_OK) {ret = 1;}

	  // Read OUT_X_L_A, OUT_X_H_A
	  uint8_t outx_buf_addr[10] = {OUT_X_L_A};
	  ret = HAL_I2C_Master_Transmit(&hi2c1, SAD_W_A, &outx_buf_addr[0], 1, 1000);
	  if (ret != HAL_OK) {ret = 1;}
	  uint8_t outx_buf_data[10];
	  ret =  HAL_I2C_Master_Receive(&hi2c1, SAD_R_A, &outx_buf_data[0], 2, 1000);
	  if (ret != HAL_OK) {ret = 1;}

	  // Read OUT_Y_L_A, OUT_Y_H_A
	  uint8_t outy_buf_addr[10] = {OUT_Y_L_A};
	  ret = HAL_I2C_Master_Transmit(&hi2c1, SAD_W_A, &outy_buf_addr[0], 1, 1000);
	  if (ret != HAL_OK) {ret = 1;}
	  uint8_t outy_buf_data[10];
	  ret =  HAL_I2C_Master_Receive(&hi2c1, SAD_R_A, &outy_buf_data[0], 2, 1000);
	  if (ret != HAL_OK) {ret = 1;}

	  // Read OUT_Z_L_A, OUT_Z_H_A
	  uint8_t outz_buf_addr[10] = {OUT_Z_L_A};
	  ret = HAL_I2C_Master_Transmit(&hi2c1, SAD_W_A, &outz_buf_addr[0], 1, 1000);
	  if (ret != HAL_OK) {ret = 1;}
	  uint8_t outz_buf_data[10];
	  ret =  HAL_I2C_Master_Receive(&hi2c1, SAD_R_A, &outz_buf_data[0], 2, 1000);
	  if (ret != HAL_OK) {ret = 1;}


	  int16_t accelerometer.x = (outx_buf_data[1] << 8) + outx_buf_data[0];
	  int16_t accelerometer.y = (outy_buf_data[1] << 8) + outy_buf_data[0];
	  int16_t accelerometer.z = (outz_buf_data[1] << 8) + outz_buf_data[0];

/* 	  float xgrav = (xraw*.061) / 1000.0;
	  float ygrav = (yraw*.061) / 1000.0;
	  float zgrav = (zraw*.061) / 1000.0; */


}

int main(void){
    //step 1: get raw data and make it into units that we can use
    //step 2: take data and signal processing
    data accelerometer;
    data gyroscope;

    float _pitch;
    float _roll;

    //Need to get this data from the IMU, 
    accelerometer.x = 0;
    accelerometer.y = 0;
    accelerometer.z = 0;
    gyroscope.x = 0;
    gyroscope.y = 0;
    gyroscope.z = 0;


    float _pitchGyroFavoring = /*FILL IN*/; // percentage 0-1.0
    float _rollGyroFavoring = /*FILL IN*/; // percentage 0-1.0
    float _filterUpdateRate = /*FILL IN*/;

    ASSERT(_pitchGyroFavoring + _rollGyroFavoring == 1.0);

    while(1){

        updateAccelData(accelerometer);
        //need one of these ^ for gyro data
        _pitch = getPitch(gyroscope, accelerometer, _pitchGyroFavoring, _filterUpdateRate);
        _roll = getRoll(gyroscope, accelerometer, _rollGyroFavoring,  _filterUpdateRate);

        // need to get this in normal units
    }

    // Complimentary Filter

    // The above code returns the angle in RADIANS

    //Accelerometer Output (G) = (N-512) / 1024 * (double)10.78;

        /*
        */
}