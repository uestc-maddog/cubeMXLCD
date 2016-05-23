/*-----------------------------------------------------------------
 * Name:      flir_lcd.h
 * Purpose:   work with flir_lcd.c
 *-----------------------------------------------------------------
 * Provide macro defination and function/variable extern for 
 * flir_lcd.c 
 *
 * The pin usage is:
 *
 * PB0 - RST
 * PB1 - RS
 * PB2 - CE
 * PA5 - SCK
 * PA7 - MOSI
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

#include "sys.h"

/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/
/* LCD basic information */
#define LCD_RAW					128
#define LCD_COLUMN			160
#define LCD_PIXPOINT		20480

#define LCD_FULSIZE_BUF		40960

/* buffer size defination */
// clear buffer size
#define LCD_CLEAR_BUF_SIZ	320	
#define LCD_CLEAR_ROUND		(LCD_FULSIZE_BUF/LCD_CLEAR_BUF_SIZ)


/* Control related*/
//LCD_RST
#define SPILCD_RST_SET		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)//PB0 
#define SPILCD_RST_RESET	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)//PB0 

//LCD_RS//dc  
#define SPILCD_RS_SET			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)//PB1 
#define SPILCD_RS_RESET		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)//PB1 

//LCD_CS  
#define SPILCD_CS_SET 	 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)//PB2 
#define SPILCD_CS_RESET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)//PB2

//LCD REST    
#define	LCD_REST PBout(1) 			

/* Color define */
#define WHITE				0xFFFF
#define BLACK				0x0000	  
#define BLUE				0x001F  
#define BRED				0XF81F
#define GRED 				0XFFE0
#define GBLUE				0X07FF
#define RED       	0xF800
#define MAGENTA   	0xF81F
#define GREEN      	0x07E0
#define CYAN       	0x7FFF
#define YELLOW     	0xFFE0
#define BROWN 			0XBC40
#define BRRED 			0XFC07
#define GRAY  			0X8430 


/* Type Define */
typedef struct  
{ 					    
	uint16_t width;			// LCD width
	uint16_t height;		// LCD height
	uint16_t id;				// LCD ID
	uint8_t	wramcmd;		// write commmand
	uint8_t  setxcmd;		// set x position command
	uint8_t  setycmd;		// set y position command
}_lcd_dev; 


/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/



/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
// LCD configuration variable
extern _lcd_dev lcddev;	
 
/********************************************************************************************************
 *                                               EXTERNAL FUNCTIONS
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
extern void LCD_SetCursor(uint16_t , uint16_t );

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
extern void LCD_DrawPoint(uint16_t ,uint16_t );

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
extern void LCD_Init(void);

/*********************************************************************
 * @fn      LCD_Clear
 *
 * @brief   Clear LCD. Set screen to sigle color. 
 *
 * @param   n
 *
 * @return  none
 */
extern void LCD_Clear(uint16_t);

/*********************************************************************
 */

