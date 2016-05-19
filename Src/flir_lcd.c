/*-----------------------------------------------------------------
 * Name:      
 * Purpose:   
 *-----------------------------------------------------------------
 * 
 * Copyright (c) *reserve
 
||                       _      _               ||
||    /\  /\  __  _  __ | | __ | | ____  ___ _  ||
||   /  \/  \/ _ ` |/ _ ` |/ _ ` |/ _  \/ _ ` | ||
||  / /\  /\  (_|    (_|    (_|    (_)   (_)  | ||
|| /_/  \/  \_.__, |\__, _|\__, _|\____/\___. | ||
|| =====================================|____/  ||
||                                              ||

 -----------------------------------------------------------------*/
 
/********************************************************************************************************
 *                                               INCLUDES
 ********************************************************************************************************/
/* System ralted */
#include "stm32l0xx_hal.h"
#include <string.h> 	
#include "stdlib.h"

/* Special requirement */
#include "flir_lcd.h"
#include "font.h"  

/* Delay function */
#include "delay.h"


/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               GLOBAL VARIABLES
 ********************************************************************************************************/
// Basic LCD information 
_lcd_dev lcddev; 

/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
// SPI1 handler variable, delare in main.c
extern SPI_HandleTypeDef hspi1;
 
/********************************************************************************************************
 *                                               EXTERNAL FUNCTIONS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               LOCAL VARIABLES
 ********************************************************************************************************/
// Point color
static uint16_t POINT_COLOR=0x0000;	
// Background color
static uint16_t BACK_COLOR=0xFFFF;  

// buffer used to clear screen
static uint8_t screenClear[LCD_CLEAR_BUF_SIZ];

/********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

void LCD_WR_REG(uint16_t);
void LCD_WR_DATA(uint16_t);
void LCD_WR_DATA8(uint8_t );
void LCD_WR_REG_DATA(uint8_t , uint16_t );
void LCD_WriteRAM_Prepare(void);

/********************************************************************************************************
 *                                               PUBLIC FUNCTIONS
 ********************************************************************************************************/
 
/*********************************************************************
 * @fn      LCD_SetCursor
 *
 * @brief   Set cursor position. The size is 160*128.
 *          And the top left is the starting address, (0,0)
 *
 *			X axis is the long side, Y is the short side.
 *
 * @param   uint16_t Xpos - X position. 
 *			uint16_t Ypos - Y position.
 *
 * @return  none
 */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	LCD_WR_REG(lcddev.setxcmd); 
	LCD_WR_DATA8(Xpos>>8); 
	LCD_WR_DATA8(Xpos&0XFF);	 
	Xpos = 159;
	LCD_WR_DATA8(Xpos>>8); 
	LCD_WR_DATA8(Xpos&0XFF);	
	
	LCD_WR_REG(lcddev.setycmd); 
	LCD_WR_DATA8(Ypos>>8); 
	LCD_WR_DATA8(Ypos&0XFF);
	Ypos = 127;
	LCD_WR_DATA8(Ypos>>8); 
	LCD_WR_DATA8(Ypos&0XFF);	
} 	

/*********************************************************************
 * @fn      LCD_DrawPoint
 *
 * @brief   Draw a single point with x.y position provide.
 *
 *			X axis is the long side, Y is the short side.
 *
 * @param   uint16_t x - X position. 
 *			uint16_t y - Y position.
 *
 * @return  none
 */
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	// set cursor location
	LCD_SetCursor(x,y);		 
	// start writing command
	LCD_WriteRAM_Prepare();	
	LCD_WR_DATA(POINT_COLOR); 
} 

/*********************************************************************
 * @fn      LCD_Init
 *
 * @brief   Init LCD. Set basic information about the LCD
 *
 *			- Set color mode 65k
 *			- Revert X,Y axis
 *			- Set Frame Rate and power settings.
 *
 * @param   none
 *
 * @return  none
 */
void LCD_Init(void)
{ 	 	
	// Power on sequence first, HW/SW reset
	LCD_REST=0;		 
 	delay_ms(50); // delay 20 ms 
	LCD_REST=1;		 
 	delay_ms(50); // delay 20 ms 

	SPILCD_RST_RESET ;	//LCD_RST=0	 //SPI接口复位
	delay_ms(20); // delay 20 ms 
	SPILCD_RST_SET ;	//LCD_RST=1		
	delay_ms(20);

	lcddev.width=128;
	lcddev.height=160;
	lcddev.wramcmd=0X2C;
	lcddev.setxcmd=0X2A;
	lcddev.setycmd=0X2B; 	
	
	//Sleep out
	LCD_WR_REG(0x11);
	delay_ms(120); //Delay 120ms
	//------------------------------------ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB3);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x3C);
	//------------------------------------End ST7735S Frame Rate-----------------------------------------//
	LCD_WR_REG(0xB4); //Dot inversion
	LCD_WR_DATA8(0x00); // 0x03
	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x04);
	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0XC0);
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);
	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x2A);
	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0xEE);
	//---------------------------------End ST7735S Power Sequence-------------------------------------//
	LCD_WR_REG(0xC5); //VCOM
	LCD_WR_DATA8(0x1A);
	LCD_WR_REG(0x36); //MX, MY, RGB mode
	LCD_WR_DATA8(0x20); // invert raw/column
	//------------------------------------ST7735S Gamma Sequence-----------------------------------------//
	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x22);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x30);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2A);
	LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x2E);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x13);
	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x16);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x23);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x25);
	LCD_WR_DATA8(0x2D);
	LCD_WR_DATA8(0x3B);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x01);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x13);
	//------------------------------------End ST7735S Gamma Sequence-----------------------------------------//
	LCD_WR_REG(0x3A); //65k mode
	LCD_WR_DATA8(0x05);
	LCD_WR_REG(0x29); //Display on

	// reset to white screen
	LCD_Clear(BLACK); 
}  


/*********************************************************************
 * @fn      LCD_Clear
 *
 * @brief   Clear LCD. Set screen to sigle color. 
 *
 * @param   n
 *
 * @return  none
 */
void LCD_Clear(uint16_t color)
{
	uint32_t index=0;      
	
	// set cousor
	LCD_SetCursor(0,0);
	
	// prepare to write
	LCD_WriteRAM_Prepare();     
	
	memset(screenClear, color, sizeof(screenClear));
	
	// LCD_CS=0
	SPILCD_CS_RESET;  
	SPILCD_RS_SET;	
	
	// block sending
	for(index = 0; index < LCD_CLEAR_ROUND; index ++)
	{
		HAL_SPI_Transmit(&hspi1, (uint8_t*)screenClear, LCD_CLEAR_BUF_SIZ, 100);	
	}
	
	// LCD_CS=1		
	SPILCD_CS_SET;  
}  

/********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

/*********************************************************************
 * @fn      LCD_WR_REG
 *
 * @brief   Send one byte register address, blocking send.
 *
 * @param   uint16_t regval - the address
 *
 * @return  none
 */
void LCD_WR_REG(uint16_t regval)
{ 
	uint8_t temp;

	// LCD_CS=0
	SPILCD_CS_RESET;  
	// Reset RS to indicate a register writing.
	SPILCD_RS_RESET;

	// send information
	temp = regval&0x00FF;
	HAL_SPI_Transmit(&hspi1, &temp, 1 , 20);

	//LCD_CS=1
	SPILCD_CS_SET;  	   		 
}


/*********************************************************************
 * @fn      LCD_WR_DATA
 *
 * @brief   Write two bytes data address, blocking send.
 *
 * @param   uint16_t data - the data
 *
 * @return  none
 */
void LCD_WR_DATA(uint16_t data)
{
	//LCD_CS=0
 	SPILCD_CS_RESET;
	// Set RS to indicate a data transmit
	SPILCD_RS_SET;	
	
	// send data
	HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 2, 20);
	
	//LCD_CS=1
	SPILCD_CS_SET; 		
}

/*********************************************************************
 * @fn      LCD_WR_DATA8
 *
 * @brief   Write one byte data address, blocking send.
 *
 * @param   uint16_t da - the data
 *
 * @return  none
 */
void LCD_WR_DATA8(uint8_t da)   
{
	//LCD_CS=0
 	SPILCD_CS_RESET;
	// Set RS to indicate a data transmit
	SPILCD_RS_SET;	

	// send data
	HAL_SPI_Transmit(&hspi1, &da, 1 , 20);
	
	//LCD_CS=1  	
	SPILCD_CS_SET;   			 
}	

/*********************************************************************
 * @fn      LCD_WR_DATA8
 *
 * @brief   Write two bytes data to an register address, blocking send.
 *
 * @param   uint16_t LCD_RegValue - the data
 *	        uint8_t LCD_Reg - the address
 *
 * @return  none
 */
void LCD_WR_REG_DATA(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
	LCD_WR_REG(LCD_Reg);
	LCD_WR_DATA(LCD_RegValue);
}

/*********************************************************************
 * @fn      LCD_WriteRAM_Prepare
 *
 * @brief   Prepare to send graphic information to GRam.
 *			The graphic data should be transmit after this command. 
 *
 * @param   none
 *
 * @return  none
 */
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);  
}	



/*********************************************************************
 */

