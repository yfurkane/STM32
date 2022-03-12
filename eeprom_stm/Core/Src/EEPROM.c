
/**
  ******************************************************************************

  EEPROM.c Using the HAL I2C Functions
  Author:   ControllersTech
  Updated:  Feb 16, 2021

  ******************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/

#include "EEPROM.h"
#include "math.h"
#include "string.h"

// Define the I2C
extern I2C_HandleTypeDef hi2c1;
#define EEPROM_I2C &hi2c1

// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0xAE

// Define the Page Size and number of pages
#define PAGE_SIZE 8     // in Bytes
#define PAGE_NUM  256    // number of pages



/*****************************************************************************************************************************************/
uint8_t bytes_temp[4];


// function to determine the remaining bytes
uint16_t bytestowrite (uint16_t size, uint16_t offset)
{
	if ((size+offset)<PAGE_SIZE) return size;
	else return PAGE_SIZE-offset;
}

/* write the data to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{

	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(PAGE_SIZE)/log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	// write the data
	for (int i=0; i<numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);  // calculate the remaining bytes to be written

		HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 1000);  // write the data to the EEPROM

		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay (5);  // Write cycle delay (5ms)
	}
}

void float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}


/*Write the Float/Integer values to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the float/integer value that you want to write
 */

void EEPROM_Write_NUM (uint16_t page, uint16_t offset, float data)
{

	float2Bytes(bytes_temp, data);

	EEPROM_Write(page, offset, bytes_temp, 4);
}

/* Reads the single Float/Integer values from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @returns the float/integer value
 */

float EEPROM_Read_NUM (uint16_t page, uint16_t offset)
{
	uint8_t buffer[4];

	EEPROM_Read(page, offset, buffer, 4);

	return (Bytes2float(buffer));
}

/* READ the data from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	int paddrposition = log(PAGE_SIZE)/log(2);

	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	for (int i=0; i<numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 1000);
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;
	}
}

/* Erase a page in the EEPROM Memory
 * @page is the number of page to erase
 * In order to erase multiple pages, just use this function in the for loop
 */
void EEPROM_PageErase (uint16_t page)
{
	// calculate the memory address based on the page number
	int paddrposition = log(PAGE_SIZE)/log(2);
	uint16_t MemAddress = page<<paddrposition;

	// create a buffer to store the reset values
	uint8_t data[PAGE_SIZE];
	memset(data,0xff,PAGE_SIZE);

	// write the data to the EEPROM
	HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, data, PAGE_SIZE, 1000);

	HAL_Delay (5);  // write cycle delay 
}

void EEPROM_WriteData(uint8_t data[])
{
		uint8_t datay1[8];
		uint8_t datay2[8];
		datay1[0] = data[0];
		datay1[1] = data[1];
    datay1[2] = data[2];
		datay1[3] = data[3];
		datay1[4] = data[4];
		datay1[5] = data[5];
		datay1[6] = data[6];
		datay1[7] = data[7];
		datay2[0] = data[8];
		datay2[1] = data[9];
		datay2[2] = data[10];
		datay2[3] = data[11];
		datay2[4] = data[12];
		datay2[5] = data[13];
		datay2[6] = data[14];
		datay2[7] = data[15];
		HAL_Delay(5);
	EEPROM_Write(3, 0, datay1, strlen((char *)datay1));
	EEPROM_Write(3, 0, datay2, strlen((char *)datay2));


}
