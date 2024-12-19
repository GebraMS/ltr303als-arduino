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
 * @Author       	: Sepehr Azimi
 * ________________________________________________________________________________________________________
 */
#include "GebraBit_LTR303ALS.h"

/*========================================================================================================================================= 
 * @brief     Read  data from  spacial register address.
 * @param     regAddr Register Address of LTR303ALS that reading data from this address
 * @param     data    Pointer to Variable that data is saved .
 * @return    None
 ========================================================================================================================================*/
void GB_LTR303ALS_Read_Reg_Data(uint8_t regAddr,  uint8_t *data)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(LTR303ALS_ADDRESS);
    Wire.write(regAddr); 
    Wire.endTransmission(false); 
    Wire.requestFrom((uint8_t)LTR303ALS_ADDRESS, (uint8_t)1);
	delay(15);
    if (Wire.available()) {
        *data = Wire.read(); 
    }
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of LTR303ALS that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    None
 ========================================================================================================================================*/
void GB_LTR303ALS_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(LTR303ALS_ADDRESS);
    Wire.write(regAddr); 
    Wire.endTransmission(false); 
    Wire.requestFrom((uint8_t)LTR303ALS_ADDRESS, (uint8_t)byteQuantity); 
	delay(15);
    for (uint16_t i = 0; i < byteQuantity; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        }
    }
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of LTR303ALS .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    None
 ========================================================================================================================================*/
void GB_LTR303ALS_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t tempData = 0;
	GB_LTR303ALS_Read_Reg_Data( regAddr, &tempData);
	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
	tempData &= mask; // zero all non-important bits in data
	tempData >>= (start_bit - len + 1); //shift data to zero position
	*data = tempData;
}
/*=========================================================================================================================================
 * @param     cmd    	Command that will be writen 
 * @return    None
 ========================================================================================================================================*/
void GB_LTR303ALS_Write_Command( uint8_t cmd)
{
	uint8_t TBuff[1];
	TBuff[0]=cmd;
	Wire.beginTransmission(LTR303ALS_ADDRESS);
	Wire.write(cmd); 
    Wire.endTransmission(); 
	//HAL_I2C_Master_Transmit(LTR303ALS_I2C,LTR303ALS_WRITE_ADDRESS,TBuff,1,100);
}
void GB_LTR303ALS_Write_Reg_Data(uint8_t regAddr,  uint8_t data)																			/*		Read Burst Data From Register			*/
{
	//HAL_I2C_Mem_Write(LTR303ALS_I2C,LTR303ALS_WRITE_ADDRESS,regAddr,1,&data,1,200);
	Wire.beginTransmission(LTR303ALS_ADDRESS);
    Wire.write(regAddr); 
    Wire.write(data); 
    Wire.endTransmission();
}
/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of LTR303ALS .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    None
 ========================================================================================================================================*/
void GB_LTR303ALS_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t tempData = 0;
	GB_LTR303ALS_Read_Reg_Data( regAddr, &tempData) ;	
	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
	data <<= (start_bit - len + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tempData &= ~(mask); // zero all important bits in existing byte
	tempData |= data; // combine data with existing byte
	GB_LTR303ALS_Write_Reg_Data(regAddr,  tempData);
}
/*=========================================================================================================================================
 * @brief     Reset LTR303ALS
 * @param     LTR303ALS   LTR303ALS Struct
 * @return    None
 ========================================================================================================================================*/
void GB_LTR303ALS_Soft_Reset ( GebraBit_LTR303ALS * LTR303ALS )  
{
	GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_ALS_CONTR, START_MSB_BIT_AT_1, BIT_LENGTH_1, 1);
	delay(100);
	LTR303ALS->RESET = DONE ;
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_ALS_Mode ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_ALS_Mode als ) 
{
 GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_ALS_CONTR, START_MSB_BIT_AT_0, BIT_LENGTH_1, als);
 LTR303ALS->ALS_MODE = als ;
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_ALS_Gain ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_ALS_Gain gain ) 
{
 GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_ALS_CONTR, START_MSB_BIT_AT_4, BIT_LENGTH_3, gain);
 LTR303ALS->ALS_GAIN = gain ;
 	switch(LTR303ALS->ALS_GAIN)
	 {
	  case ALS_GAIN_1X:
		LTR303ALS->ALS_GAIN_VALUE = 1 ;
    break;
		case ALS_GAIN_2X:
		LTR303ALS->ALS_GAIN_VALUE = 2 ;
    break;	
		case ALS_GAIN_4X:
		LTR303ALS->ALS_GAIN_VALUE = 4 ;
    break;	
		case ALS_GAIN_8X:
		LTR303ALS->ALS_GAIN_VALUE = 8 ;
    break;
		case ALS_GAIN_48X:
		LTR303ALS->ALS_GAIN_VALUE = 48 ;
    break;	
		case ALS_GAIN_96X:
		LTR303ALS->ALS_GAIN_VALUE = 96 ;
    break;			
	 }
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Measurement_Repeat_Rate ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Measurement_Rate rate ) 
{
 GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_ALS_MEAS_RATE, START_MSB_BIT_AT_2, BIT_LENGTH_3, rate);
 LTR303ALS->MEASUREMENT_RATE = rate ;
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Integration_Time ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Integration_Time intg ) 
{
 GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_ALS_MEAS_RATE, START_MSB_BIT_AT_5, BIT_LENGTH_3, intg);
 LTR303ALS->INTEGRATION_TIME = intg ;
 switch(LTR303ALS->INTEGRATION_TIME)
	 {
	  case ALS_INTEGTIME_100_mS:
		LTR303ALS->INTEGRATION_TIME_VALUE = 1 ;
    break;
		case ALS_INTEGTIME_50_mS:
		LTR303ALS->INTEGRATION_TIME_VALUE = 0.5 ;
    break;	
		case ALS_INTEGTIME_200_mS:
		LTR303ALS->INTEGRATION_TIME_VALUE = 2 ;
    break;	
		case ALS_INTEGTIME_400_mS: 
		LTR303ALS->INTEGRATION_TIME_VALUE = 4 ;
    break;
		case ALS_INTEGTIME_150_mS:
		LTR303ALS->INTEGRATION_TIME_VALUE = 1.5 ;
    break;	
		case ALS_INTEGTIME_250_mS:
		LTR303ALS->INTEGRATION_TIME_VALUE = 2.5 ;
    break;
		case ALS_INTEGTIME_300_mS:
		LTR303ALS->INTEGRATION_TIME_VALUE = 3 ;
    break;	
		case ALS_INTEGTIME_350_mS:
		LTR303ALS->INTEGRATION_TIME_VALUE = 3.5 ;
    break;		
	 }
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Read_Part_ID ( GebraBit_LTR303ALS * LTR303ALS  ) 
{
 GB_LTR303ALS_Read_Reg_Data(LTR303ALS_PART_ID, &LTR303ALS->PART_ID);
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Read_Manufacture_ID ( GebraBit_LTR303ALS * LTR303ALS ) 
{
 GB_LTR303ALS_Read_Reg_Data(LTR303ALS_MANUFAC_ID, &LTR303ALS->MANUFACTURE_ID);
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Read_ALS_STATUS ( GebraBit_LTR303ALS * LTR303ALS ) 
{
 uint8_t status;
 GB_LTR303ALS_Read_Reg_Data(LTR303ALS_ALS_STATUS, &status);
 LTR303ALS->DATA_STATUS = (status & 0x04)>>2  ;
 LTR303ALS->INTERRRUPT_STATUS = (status & 0x08)>>3  ;
 LTR303ALS->DATA = (status & 0x80)>>7  ;
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Interrupt_Mode ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Interrupt_Mode mode ) 
{
 GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_INTERRUPT, START_MSB_BIT_AT_1, BIT_LENGTH_1, mode);
 LTR303ALS->INTERRUPT_MODE = mode ;
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Interrupt_Polarity ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Interrupt_Polarity polar ) 
{
 GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_INTERRUPT, START_MSB_BIT_AT_2, BIT_LENGTH_1, polar);
 LTR303ALS->INTERRUPT_POLARITY = polar ;
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Interrupt_Persist ( GebraBit_LTR303ALS * LTR303ALS , LTR303ALS_Interrupt_Persist persist ) 
{
 GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_INTERRUPT_PERSIST, START_MSB_BIT_AT_3, BIT_LENGTH_4, persist);
 LTR303ALS->INTERRUPT_PERSIST = persist ;
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Interrupt_Upper_Limitation ( GebraBit_LTR303ALS * LTR303ALS , uint16_t limit ) 
{
 GB_LTR303ALS_Write_Reg_Data(LTR303ALS_ALS_THRES_UP_1, (uint8_t)limit>>8);
 GB_LTR303ALS_Write_Reg_Data(LTR303ALS_ALS_THRES_UP_0, (uint8_t)(limit&0xFF));
 LTR303ALS->INTERRUPT_UPPER_THRESHOLD = limit ;
}
/*=========================================================================================================================================
 * @brief     Read LTR303ALS PROM
 * @param     LTR303ALS   LTR303ALS Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_LTR303ALS_Interrupt_Lower_Limitation ( GebraBit_LTR303ALS * LTR303ALS , uint16_t limit ) 
{
 GB_LTR303ALS_Write_Reg_Data(LTR303ALS_ALS_THRES_LOW_1, (uint8_t)limit>>8);
 GB_LTR303ALS_Write_Reg_Data(LTR303ALS_ALS_THRES_LOW_0, (uint8_t)(limit&0xFF));
 LTR303ALS->INTERRUPT_LOWER_THRESHOLD = limit ;
}
/*=========================================================================================================================================
 * @brief     initialize LTR303ALS
 * @param     LTR303ALS     LTR303ALS Struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_LTR303ALS_initialize( GebraBit_LTR303ALS * LTR303ALS )
{
  GB_LTR303ALS_Soft_Reset   ( LTR303ALS ) ;
  GB_LTR303ALS_Read_Part_ID ( LTR303ALS ) ;
	GB_LTR303ALS_Read_Manufacture_ID ( LTR303ALS ) ;
//	GB_LTR303ALS_Interrupt_Upper_Limitation( LTR303ALS , 40000);
//	GB_LTR303ALS_Interrupt_Lower_Limitation( LTR303ALS , 30000);
	GB_LTR303ALS_Interrupt_Mode( LTR303ALS , INT_PIN_INACTIVE ) ;
	GB_LTR303ALS_Interrupt_Polarity( LTR303ALS , ACTIVE_LOW ) ;
	GB_LTR303ALS_Interrupt_Persist( LTR303ALS , CONSECUTIVE_5_ALS_VALUE_OUT_OF_THR_RANGE ) ;
	GB_LTR303ALS_ALS_Mode ( LTR303ALS ,  ACTIVE );
}
/*=========================================================================================================================================
 * @brief     Configure LTR303ALS
 * @param     LTR303ALS  Configure LTR303ALS 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_LTR303ALS_Configuration(GebraBit_LTR303ALS * LTR303ALS)
{
  GB_LTR303ALS_ALS_Gain( LTR303ALS , ALS_GAIN_96X ) ;
	GB_LTR303ALS_Measurement_Repeat_Rate( LTR303ALS , ALS_MEASRATE_200_mS ) ;
	GB_LTR303ALS_Integration_Time( LTR303ALS , ALS_INTEGTIME_100_mS ) ;
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature And Pressure And Humidity from Register 
 * @param     LTR303ALS  store Raw Data Of Temprature in GebraBit_LTR303ALS Staruct REGISTER_RAW_TEMPERATURE & REGISTER_RAW_PRESSURE & REGISTER_RAW_HUMIDITY
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_LTR303ALS_Get_Register_Raw_Pressure_Temperature_Humidity(GebraBit_LTR303ALS * LTR303ALS)
{
	GB_LTR303ALS_Read_ALS_STATUS( LTR303ALS );
	if ( (LTR303ALS->DATA_STATUS == NEW_DATA)&&(LTR303ALS->DATA == DATA_IS_VALID) ) //( (LTR303ALS->DATA_STATUS == NEW_DATA)&&(LTR303ALS->DATA == DATA_IS_VALID) )    OR    ( LTR303ALS->DATA_STATUS == NEW_DATA )
	{    
   GB_LTR303ALS_Burst_Read( LTR303ALS_ALS_DATA_CH1_0 , LTR303ALS->REGISTER_DATA , 4);
	 LTR303ALS->ALS_DATA_CH1 = LTR303ALS->REGISTER_DATA[1]<<8|LTR303ALS->REGISTER_DATA[0]  ;
   LTR303ALS->ALS_DATA_CH0 = LTR303ALS->REGISTER_DATA[3]<<8|LTR303ALS->REGISTER_DATA[2]  ;
	}
} 
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature And Pressure And Humidity from Register 
 * @param     LTR303ALS  store Raw Data Of Temprature in GebraBit_LTR303ALS Staruct REGISTER_RAW_TEMPERATURE & REGISTER_RAW_PRESSURE & REGISTER_RAW_HUMIDITY
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_LTR303ALS_Lux_Reading(GebraBit_LTR303ALS * LTR303ALS)
{
	LTR303ALS->RATIO = ((float)LTR303ALS->ALS_DATA_CH1)/(float)(LTR303ALS->ALS_DATA_CH1 + LTR303ALS->ALS_DATA_CH0 );
	if(LTR303ALS->RATIO<0.45)
	 LTR303ALS->ALS_LUX =(((1.7743 * (float)LTR303ALS->ALS_DATA_CH0) + (1.1059 * (float)LTR303ALS->ALS_DATA_CH1)) / (float)LTR303ALS->ALS_GAIN_VALUE) / (float)LTR303ALS->INTEGRATION_TIME_VALUE ;
	else if((LTR303ALS->RATIO<0.64)&&(LTR303ALS->RATIO>=0.45))
	 LTR303ALS->ALS_LUX =(((4.2785 * (float)LTR303ALS->ALS_DATA_CH0) - (1.9548 * (float)LTR303ALS->ALS_DATA_CH1)) / (float)LTR303ALS->ALS_GAIN_VALUE) / (float)LTR303ALS->INTEGRATION_TIME_VALUE ;
  else if((LTR303ALS->RATIO<0.85)&&(LTR303ALS->RATIO>=0.64))
	 LTR303ALS->ALS_LUX =(((0.5926 * (float)LTR303ALS->ALS_DATA_CH0) + (0.1185 * (float)LTR303ALS->ALS_DATA_CH1)) / (float)LTR303ALS->ALS_GAIN_VALUE) / (float)LTR303ALS->INTEGRATION_TIME_VALUE ;
  else
	LTR303ALS->ALS_LUX = 0; 
} 
/*=========================================================================================================================================
 * @brief     Get Data Directly 
 * @param     LTR303ALS       GebraBit_LTR303ALS Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_LTR303ALS_Get_Data(GebraBit_LTR303ALS * LTR303ALS)
{
  GB_LTR303ALS_Get_Register_Raw_Pressure_Temperature_Humidity( LTR303ALS );
	GB_LTR303ALS_Lux_Reading(LTR303ALS);
}
///*=========================================================================================================================================
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/
//GB_LTR303ALS_Read_Reg_Data (LTR303ALS_ALS_MEAS_RATE,&DT);
//GB_LTR303ALS_Read_Reg_Bits (LTR303ALS_ALS_MEAS_RATE, START_MSB_BIT_AT_5, BIT_LENGTH_6, &DT);
//GB_LTR303ALS_Write_Reg_Data(LTR303ALS_ALS_MEAS_RATE, 0x2D)	;
//GB_LTR303ALS_Read_Reg_Data (LTR303ALS_ALS_MEAS_RATE,&DT);
//GB_LTR303ALS_Read_Reg_Bits (LTR303ALS_ALS_MEAS_RATE, START_MSB_BIT_AT_5, BIT_LENGTH_6, &DT);
//GB_LTR303ALS_Write_Reg_Bits(LTR303ALS_ALS_MEAS_RATE, START_MSB_BIT_AT_5, BIT_LENGTH_3, 3);
//GB_LTR303ALS_Read_Reg_Data (LTR303ALS_ALS_MEAS_RATE,&DT);
//GB_LTR303ALS_Read_Reg_Bits (LTR303ALS_ALS_MEAS_RATE, START_MSB_BIT_AT_5, BIT_LENGTH_3, &DT);