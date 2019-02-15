/*
 * vl53l0x_calibration.c
 *
 *  Created on: 8 февр. 2019 г.
 *      Author: User
 */

#include "vl53l0x_calibration.h"

/*extern variables and structures*/
extern VL53L0X_Dev_t	myDevStruct[3];
extern VL53L0X_DEV myDev;
extern UART_HandleTypeDef huart2;

//-----------------------------------------------------------------------------------------------------------------------------------------------

void vl53l0x_calibration ()
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET); //xShut pin on

  HAL_Delay(5); //wait for wake up
//-----------------------------------------------------------------------------------------------------------------------------------------------
  int k;
  uint16_t Id;
  int status;
  uint8_t str[15];

  for (k=0;k<=2;k++)
  {
  myDev=&myDevStruct[k];
  myDev->I2cDevAddr=0x52;
  myDev->Present = 0;
  myDev->I2cHandle=&hi2c2;

    	status = VL53L0X_RdWord(myDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
  		if (!status)
  		{
  		sprintf(str, "%d my id \r\n", Id);
  		HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  		}
  		else
  		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET); //LED on

  									HAL_Delay(10);
//-----------------------------------------------------------------------------------------------------------------------------------------------

  		VL53L0X_ResetDevice(myDev);
  		status = VL53L0X_SetDeviceAddress(myDev, 0x52+(k+1)*2);
                  if( status == 0 )
                  {
  							myDev->I2cDevAddr=0x52+(k+1)*2;
  							sprintf(str, "Adress ok  %x\r\n", myDev->I2cDevAddr);
  							HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
                  }
                  else
  				  {

                	  	  	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  							sprintf(str, "Adres fail %d\r\n", status);
  							HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
                  }
  									HAL_Delay(10);
//-----------------------------------------------------------------------------------------------------------------------------------------------

      status = VL53L0X_DataInit(myDev);
  				if( status == 0 )
                myDev->Present = 1;

                else
  				  {
  							sprintf(str, "Data init fail %d\r\n", status);
  							HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  							HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
                  }
  									HAL_Delay(20);
//-----------------------------------------------------------------------------------------------------------------------------------------------

    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
  	uint8_t isApertureSpads;
  	FixPoint1616_t signalLimit=(FixPoint1616_t)(0.1*65536);
  	FixPoint1616_t sigmaLimit=(FixPoint1616_t)(60*65536);
  	uint32_t timingBudget = 33000;
  	uint8_t preRangeVcselPeriod = 18;
  	uint8_t finalRangeVcselPeriod = 14;

 //-----------------------------------------------------------------------------------------------------------------------------------------------

              status=VL53L0X_StaticInit(myDev);
              if( status )	{
  							sprintf(str, "static init fail %d\r\n", status);
  							HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  					  		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  							}
              else
              	  {
  							sprintf(str, "static init Ok\r\n");
  							HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				  }

              	  	  	  	  	  HAL_Delay(1);
//-----------------------------------------------------------------------------------------------------------------------------------------------


  			status = VL53L0X_PerformRefSpadManagement(myDev, &refSpadCount, &isApertureSpads);
  			if( status ){
  						sprintf(str, "perform SPAD calibration fail %d\r\n", status);
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  						}
  			else
  				 {
  						sprintf(str, "spad calibration Ok\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				 }

  									HAL_Delay(10);
//-----------------------------------------------------------------------------------------------------------------------------------------------


              status = VL53L0X_PerformRefCalibration(myDev, &VhvSettings, &PhaseCal);
  			if( status ){
  						sprintf(str, "Ref calibration fail %d\r\n", status);
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  						}
  			else {
  						sprintf(str, "Ref calibration ok \r\n", status);
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  			     }
  									HAL_Delay(1);
 //-----------------------------------------------------------------------------------------------------------------------------------------------


            status = VL53L0X_SetLimitCheckEnable(myDev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
  			if( status ){
  						sprintf(str, "SetLimitCheckEnable fail			%d\r\n", status);
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  						}
  			else {
  						sprintf(str, "SetLimitCheckEnable Ok\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				 }

  									HAL_Delay(1);

//-----------------------------------------------------------------------------------------------------------------------------------------------

  			status = VL53L0X_SetLimitCheckEnable(myDev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signal limit
  			if( status ){
  						sprintf(str, "SetLimitCheckEnable signal fail\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  						}
  			else {
  						sprintf(str, "SetLimitCheckEnable signal Ok\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				}

  									HAL_Delay(1);

 //-----------------------------------------------------------------------------------------------------------------------------------------------

  			status = VL53L0X_SetLimitCheckValue(myDev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
  			if( status ){
  						sprintf(str, "SetLimitCheckValue signal fail\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  						}
  			else {
  						sprintf(str, "SetLimitCheckValue signal Ok\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				 }

  									HAL_Delay(1);

//-----------------------------------------------------------------------------------------------------------------------------------------------


  			status = VL53L0X_SetLimitCheckValue(myDev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
  			if( status ){
  						sprintf(str, "SetLimitCheckValue sigma fail\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  						}
  			else {
						sprintf(str, "SetLimitCheckValue sigma Ok\r\n");
						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  	  			 }

  									HAL_Delay(1);

//-----------------------------------------------------------------------------------------------------------------------------------------------

            status = VL53L0X_SetVcselPulsePeriod(myDev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
  			if( status ){
  						sprintf(str, "SetVcselPulsePeriod fail\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
  						}
			else {
						sprintf(str, "SetVcselPulsePeriod Ok\r\n");
						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				 }

  										HAL_Delay(1);

//-----------------------------------------------------------------------------------------------------------------------------------------------

            status = VL53L0X_SetVcselPulsePeriod(myDev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
  			if( status ){
  						sprintf(str, "SetVcselPulsePeriod final fail\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

  						}
  			else {
  						sprintf(str, "SetVcselPulsePeriod final Ok\r\n");
  						HAL_UART_Transmit(&huart2, str, strlen(str), 0xffff);
  				 }

  										HAL_Delay(1);

//-----------------------------------------------------------------------------------------------------------------------------------------------

	status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(myDev, timingBudget);
	VL53L0X_SetDeviceMode(myDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	VL53L0X_SetInterruptThresholds(myDev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING ,  200<<16 ,  0<<16);
    status = VL53L0X_SetGpioConfig(myDev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW, VL53L0X_INTERRUPTPOLARITY_HIGH);
//-----------------------------------------------------------------------------------------------------------------------------------------------

  	VL53L0X_StopMeasurement(myDev);

  			if(k==0)
			  {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); //xShut pin2 on
			  }
			else
			{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); //xShut pin3 on
			}
										HAL_Delay(75);

  }
}
