/*-----------------------------------------------------------------
 * Name:      flir_display.h
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
 #ifndef FLIR_DISPLAY_H_
 #define FLIR_DISPLAY_H_
/********************************************************************************************************
 *                                               INCLUDES
 ********************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/
// flir camera data buffer size
#define LCD_FLIR_SPI_BUF_SIZ	320
#define LCD_FLIR_RX_BUF_SIZ		82

#define FLIR_TELE_LINE	60

#define LCD_YPOS_OFFSET	4

// FLIR_CS  
#define FLIR_CS_SET 	 	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)//PB12 
#define FLIR_CS_RESET  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)//PB12

// FLIR_PW_LOW
#define FLIR_PWLOW_SET	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)// PA8
#define FLIR_PWLOW_RESET	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

// FLIR_RESET
#define FLIR_RESETLOW_SET	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)// PB15
#define FLIR_RESETLOW_RESET	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)

/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
extern volatile bool flir_TXCpl;
extern volatile bool flir_RXCpl;

extern uint8_t rawPos_buf; // save next start row position
extern uint8_t colPos_buf; // save next start column position

/********************************************************************************************************
 *                                               EXTERNAL FUNCTIONS
 ********************************************************************************************************/

/*********************************************************************
 * @fn      initFlir_Display
 *
 * @brief   Init flir display. Initial hardware required and parameters.
 *          register event and event callback function.
 *
 * @param   
 *
 * @return  
 */
extern void initFlir_Display( void );

/*********************************************************************
 * @fn      initFlir_Display
 *
 * @brief   Init flir display. Initial hardware required and parameters.
 *          register event and event callback function.
 *
 * @param   none
 *
 * @return  none
 */
extern bool flir_display_startReceive( void );

/*********************************************************************
 * @fn      flir_display_recDataCheck
 *
 * @brief   Check whether a receiving data is valid. Check CRC to 
 *					verify the synchronize and then check ID to determine 
 *          whether need to display this data.
 *
 * @param   none
 *
 * @return  none
 */
extern bool flir_display_recDataCheck( void );

/*********************************************************************
 * @fn      flir_display_recDataCheck
 *
 * @brief   Check whether a receiving data is valid. Check CRC to 
 *					verify the synchronize and then check ID to determine 
 *          whether need to display this data.
 *
 * @param   none
 *
 * @return  none
 */
bool flir_display_startRec( void );
	
#endif
/*********************************************************************
 */
  
	
