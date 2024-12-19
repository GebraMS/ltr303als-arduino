/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author          : Sepehr Azimi
 * ________________________________________________________________________________________________________
 */
#ifndef	__LTR303ALS_H__
#define	__LTR303ALS_H__
#include "Wire.h"
#include "arduino.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
/************************************************
 *              USER REGISTER MAP               *
 ***********************************************/ 
#define LTR303ALS_ALS_CONTR                  0x80
#define LTR303ALS_ALS_MEAS_RATE         		 0x85
#define LTR303ALS_PART_ID		  							 0x86
#define LTR303ALS_MANUFAC_ID                 0x87
#define LTR303ALS_ALS_DATA_CH1_0             0x88      ////0xA0 TO 0xAE
#define LTR303ALS_ALS_DATA_CH1_1						 0x89
#define LTR303ALS_ALS_DATA_CH0_0						 0x8A
#define LTR303ALS_ALS_DATA_CH0_1						 0x8B
#define LTR303ALS_ALS_STATUS						     0x8C				/* I2C Address */
#define LTR303ALS_INTERRUPT                  0x8F
#define LTR303ALS_ALS_THRES_UP_0             0x97
#define LTR303ALS_ALS_THRES_UP_1		         0x98
#define LTR303ALS_ALS_THRES_LOW_0 					 0x99
#define LTR303ALS_ALS_THRES_LOW_1 			     0x9A
#define LTR303ALS_INTERRUPT_PERSIST		       0x9E
#define LTR303ALS_ADDRESS 							     0x29		
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 
 /************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
 /************************************************
 *          Register Values Begin                *
 ***********************************************/ 
#define LTR303ALS_OSR_256_CONVERSION_TIME				 1
#define LTR303ALS_OSR_512_CONVERSION_TIME				 2
#define LTR303ALS_OSR_1024_CONVERSION_TIME				 3
#define LTR303ALS_OSR_2048_CONVERSION_TIME				 5
#define LTR303ALS_OSR_4096_CONVERSION_TIME				 9
#define LTR303ALS_OSR_8192_CONVERSION_TIME				 17
#define REGISTER_DATA_BUFFER_SIZE              	   4
#define PROM_DATA_BUFFER_SIZE         				 14
#define SEA_LEVEL_PRESSURE									   101325
/*----------------------------------------------*
 *           Register Values End                *
 *----------------------------------------------*/
#define LTR329_I2CADDR_DEFAULT 0x29 ///< I2C address
#define LTR329_PART_ID 0x86         ///< Part id/revision register
#define LTR329_MANU_ID 0x87         ///< Manufacturer ID register
#define LTR329_ALS_CTRL 0x80        ///< ALS control register
#define LTR329_STATUS 0x8C          ///< Status register
#define LTR329_CH1DATA 0x88         ///< Data for channel 1 (read all 4 bytes!)
#define LTR329_MEAS_RATE 0x85       ///< Integration time and data rate
// These registers on LTR-303 only!
#define LTR303_REG_INTERRUPT 0x8F ///< Register to enable/configure int output
#define LTR303_REG_THRESHHIGH_LSB 0x97 ///< ALS 'high' threshold limit
#define LTR303_REG_THRESHLOW_LSB 0x99  ///< ALS 'low' threshold limit
#define LTR303_REG_INTPERSIST 0x9E ///< Register for setting the IRQ persistance

/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	FAILED = 0     ,                      
	DONE     
}LTR303ALS_Reset_Status;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum ALS_Mode 
{  
	STANDBY = 0     ,                      
	ACTIVE     
}LTR303ALS_ALS_Mode;

/*!    @brief  Sensor gain for ALS  */
typedef enum ALS_Gain 
{
  ALS_GAIN_1X  = 0,
  ALS_GAIN_2X  = 1,
  ALS_GAIN_4X  = 2,
  ALS_GAIN_8X  = 3,
  ALS_GAIN_48X = 6,
  ALS_GAIN_96X = 7,
} LTR303ALS_ALS_Gain;

/*!    @brief Integration times, in milliseconds */
typedef enum Integration_Time
{
  ALS_INTEGTIME_100_mS,
  ALS_INTEGTIME_50_mS,
  ALS_INTEGTIME_200_mS,
  ALS_INTEGTIME_400_mS,
  ALS_INTEGTIME_150_mS,
  ALS_INTEGTIME_250_mS,
  ALS_INTEGTIME_300_mS,
  ALS_INTEGTIME_350_mS,
} LTR303ALS_Integration_Time;

/*!    @brief Measurement rates, in milliseconds */
typedef enum Measurement_Rate
{
  ALS_MEASRATE_50_mS,
  ALS_MEASRATE_100_mS,
  ALS_MEASRATE_200_mS,
  ALS_MEASRATE_500_mS,
  ALS_MEASRATE_1000_mS,
  ALS_MEASRATE_2000_mS,
} LTR303ALS_Measurement_Rate;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum Data_Status 
{  
	OLD_DATA = 0     ,                      
	NEW_DATA     
}LTR303ALS_Data_Status;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum Interrupt_Status 
{  
	INTERRUPT_INACTIVE = 0     ,                      
	INTERRUPT_ACTIVE     
}LTR303ALS_Interrupt_Status;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum Data_Valid 
{  
	DATA_IS_VALID = 0     ,                      
	DATA_IS_INVALID     
}LTR303ALS_Data_Valid;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum Interrupt_Mode 
{  
	INT_PIN_INACTIVE = 0     ,                      
	INT_PIN_TRIG_INTERRUPT     
}LTR303ALS_Interrupt_Mode;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum Interrupt_Polarity 
{  
	ACTIVE_LOW = 0     ,                      
	ACTIVE_HIGH     
}LTR303ALS_Interrupt_Polarity;
/**************************************************************
 *       						 Values For Sample Rate    						    *
 **************************************************************/ 
typedef enum Interrupt_Persist
{
  EVERY_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_2_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_3_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_4_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_5_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_6_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_7_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_8_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_9_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_10_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_11_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_12_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_13_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_14_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_15_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_16_ALS_VALUE_OUT_OF_THR_RANGE,
} LTR303ALS_Interrupt_Persist;

 /*************************************************
 *  Defining LTR303ALS Register & Data As Struct   *
 **************************************************/
typedef	struct LTR303ALS
{
	  uint8_t                       	Register_Cache;
	  uint8_t													PART_ID;
		uint8_t													MANUFACTURE_ID;
	  LTR303ALS_Reset_Status					RESET;
	  LTR303ALS_ALS_Mode              ALS_MODE;
	  LTR303ALS_ALS_Gain              ALS_GAIN;
	  uint8_t	 												ALS_GAIN_VALUE;
	  LTR303ALS_Measurement_Rate      MEASUREMENT_RATE;
	  LTR303ALS_Integration_Time      INTEGRATION_TIME;
	  float										      INTEGRATION_TIME_VALUE;
    LTR303ALS_Data_Status           DATA_STATUS;
    LTR303ALS_Interrupt_Status			INTERRRUPT_STATUS;
    LTR303ALS_Data_Valid            DATA;
	  LTR303ALS_Interrupt_Mode        INTERRUPT_MODE;
	  LTR303ALS_Interrupt_Polarity    INTERRUPT_POLARITY;
    LTR303ALS_Interrupt_Persist     INTERRUPT_PERSIST;
	  uint16_t                        INTERRUPT_UPPER_THRESHOLD;
	  uint16_t                        INTERRUPT_LOWER_THRESHOLD;
		uint8_t 												REGISTER_DATA[REGISTER_DATA_BUFFER_SIZE];
		uint16_t               					ALS_DATA_CH1;//Reference to uint16_t where IR-only data will be stored
		uint16_t               					ALS_DATA_CH0;//Reference to uint16_t where visible+IR data will be stored
		double 													RATIO;
		double 													ALS_LUX;
}GebraBit_LTR303ALS;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *  Declare Read&Write LTR303ALS Register Values Functions *
 ********************************************************/
extern void GB_LTR303ALS_Read_Reg_Data(uint8_t regAddr,  uint8_t *data)	;
extern void GB_LTR303ALS_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity);
extern void GB_LTR303ALS_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data);	
extern void GB_LTR303ALS_Write_Command( uint8_t cmd);
extern void GB_LTR303ALS_Write_Reg_Data(uint8_t regAddr,  uint8_t data)	;
extern void GB_LTR303ALS_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data);
/********************************************************
 *       Declare LTR303ALS Configuration Functions         *
 ********************************************************/
extern void GB_LTR303ALS_Soft_Reset ( GebraBit_LTR303ALS * LTR303ALS )  ;
extern void GB_LTR303ALS_ALS_Mode ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_ALS_Mode als ) ;
extern void GB_LTR303ALS_ALS_Gain ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_ALS_Gain gain )  ;
extern void GB_LTR303ALS_Measurement_Repeat_Rate ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Measurement_Rate rate )  ;
extern void GB_LTR303ALS_Integration_Time ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Integration_Time intg )  ;
extern void GB_LTR303ALS_Read_Part_ID ( GebraBit_LTR303ALS * LTR303ALS  )  ;
extern void GB_LTR303ALS_Read_Manufacture_ID ( GebraBit_LTR303ALS * LTR303ALS )  ;
extern void GB_LTR303ALS_Read_ALS_STATUS ( GebraBit_LTR303ALS * LTR303ALS )  ;
extern void GB_LTR303ALS_Interrupt_Mode ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Interrupt_Mode mode )  ;
extern void GB_LTR303ALS_Interrupt_Polarity ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Interrupt_Polarity polar )  ;
extern void GB_LTR303ALS_Interrupt_Persist ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Interrupt_Persist persist )  ;
extern void GB_LTR303ALS_Interrupt_Upper_Limitation ( GebraBit_LTR303ALS * LTR303ALS , uint16_t limit );
extern void GB_LTR303ALS_Interrupt_Lower_Limitation ( GebraBit_LTR303ALS * LTR303ALS , uint16_t limit )  ;
extern void GB_LTR303ALS_initialize( GebraBit_LTR303ALS * LTR303ALS )  ;
extern void GB_LTR303ALS_Configuration(GebraBit_LTR303ALS * LTR303ALS)  ;
extern void GB_LTR303ALS_Get_Register_Raw_Pressure_Temperature_Humidity(GebraBit_LTR303ALS * LTR303ALS);
extern void GB_LTR303ALS_Lux_Reading(GebraBit_LTR303ALS * LTR303ALS);
extern void GB_LTR303ALS_Get_Data(GebraBit_LTR303ALS * LTR303ALS);
#endif
