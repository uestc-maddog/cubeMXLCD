/*-----------------------------------------------------------------
 * Name:      flir_display.c
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
#include "flir_display.h"

#include "crc16.h"

/********************************************************************************************************
 *                                                 MACROS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               CONSTANTS
 ********************************************************************************************************/

 
/********************************************************************************************************
 *                                               GLOBAL VARIABLES
 ********************************************************************************************************/
volatile bool flir_TXCpl = true;
volatile bool flir_RXCpl = false;

// control variable
uint8_t rawPos_buf; // save next start row position
uint8_t colPos_buf; // save next start column position

/********************************************************************************************************
 *                                               EXTERNAL VARIABLES
 ********************************************************************************************************/
//DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi1; 
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;

/********************************************************************************************************
 *                                               EXTERNAL FUNCTIONS
 ********************************************************************************************************/


/********************************************************************************************************
 *                                               LOCAL VARIABLES
 ********************************************************************************************************/
// flir camera one row graphic data
 
// transmit buffer
static uint16_t flir_rxBuf[LCD_FLIR_RX_BUF_SIZ];		// RX buffer, size LCD_FLIR_RX_BUF_SIZ. One raw data
static uint16_t flir_rx_tempBuf[LCD_FLIR_RX_BUF_SIZ]; // cache, save RX buffer value when valid to save time
static uint16_t flir_txBuf[LCD_FLIR_SPI_BUF_SIZ]; // TX buffer, size LCD_FLIR_SPI_BUF_SIZ. two raw data

/********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/
bool flir_startSeq( void );
bool flir_reSyc( void );
void flir_endDisplay( void );
void flir_startDisplay( uint16_t Xpos, uint16_t Ypos );
void flir_display_DMAPoll(uint8_t *pdata, uint16_t siz);

/********************************************************************************************************
 *                                               PUBLIC FUNCTIONS
 ********************************************************************************************************/
 
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
void initFlir_Display( void )
{
	// enable clock, SPI, DMA intterrupt
	// already implemented in main.c. No need to initialize here.
	
	// init LCD parameters.
	rawPos_buf = 0;
	colPos_buf = 0;
	flir_TXCpl = true;
	flir_RXCpl = false;
	
	// init LCD
	LCD_Init();
	
	// print the black area in the top and bottom
	// prepare black color
	memset(flir_txBuf, BLACK, LCD_FLIR_SPI_BUF_SIZ);
	
	// start from (0,0), set the top 4 raw pixels as black
	flir_startDisplay(0, 0);
	// first 2 raw
	flir_display_DMAPoll((uint8_t *)flir_txBuf, LCD_FLIR_SPI_BUF_SIZ);
	// continue for the second 2 raw
	flir_display_DMAPoll((uint8_t *)flir_txBuf, LCD_FLIR_SPI_BUF_SIZ);
	// end this transmission
	flir_endDisplay();
	
	// set cursor again to print the last 4 raw pixels as black
	flir_startDisplay(0, 124);
	// first 2 raw
	flir_display_DMAPoll((uint8_t *)flir_txBuf, LCD_FLIR_SPI_BUF_SIZ);
	// continue for the second 2 raw
	flir_display_DMAPoll((uint8_t *)flir_txBuf, LCD_FLIR_SPI_BUF_SIZ);
	// end this transmission
	flir_endDisplay();

	// start flir camera through a start up sequence
	flir_startSeq();
}

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
bool flir_display_startRec( void )
{
	// start receiving frams
	HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)flir_rxBuf, LCD_FLIR_RX_BUF_SIZ);
	return true;
}

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
static uint16_t buf[2000];
static uint16_t temp = 0;
bool flir_display_recDataCheck( void )
{
	uint16_t crcTemp;
	uint16_t idTemp;
	uint8_t rawTemp;
	uint16_t i;
	
	// reset complete flag 
	flir_RXCpl = false;
	
	// obtain crc value
	crcTemp = flir_rxBuf[1];
	
	// obtain the ID
	idTemp = flir_rxBuf[0];

	// set the four most-significant bits of ID and crc part as zero, prepare to send
	flir_rxBuf[0] &= 0x0fff;
	flir_rxBuf[1] = 0;
	
	// decide whether a re-synchronize is needed
	if(crcTemp != CalcCRC16Bytes(LCD_FLIR_RX_BUF_SIZ, (char*)flir_rxBuf))
	{
		// start re-synchronize
		if (rawPos_buf != 0)
			flir_reSyc();
		else
			flir_display_startRec();
		// return fail
		return false;
	}
	
	// data valid, copy valid data to a processing buffer
	memcpy(flir_rx_tempBuf, flir_rxBuf, LCD_FLIR_RX_BUF_SIZ);
	// clear receiving buf
	memset(flir_rxBuf, 0, LCD_FLIR_RX_BUF_SIZ);
	// now, receiving DMA is okay to perform another receiving

	
	// now the data is valid, check whether need to display the frame
	if((idTemp & 0x0f00) != 0x0f00)
	{		
		// check whether previous transmit finish, poll here
		i = 1000;
		while(!flir_TXCpl)
		{
			// delay 1ms and check again
			HAL_Delay(1);
			i--;
			
			// if not able to finish TX withn 1s, abort.
			if(!i)
			{
				flir_reSyc();
				return false;
			}
		}
		
		// frame data available, prepare to send frame
		for(i = 0; i < (LCD_FLIR_RX_BUF_SIZ - 2); i++)
		{
			// copy data from the first data value to the last
			flir_txBuf[2 * i - 1] = flir_rx_tempBuf[i + 4];
			flir_txBuf[2 * i] = flir_rx_tempBuf[i + 4];
		}
		
		// assemble tx buffer finish, transmit data to LCD through DMA
		// this operation may vary from different packet, telemety line maybe.
		// get raw number
		rawTemp = idTemp & 0x00ff;
		// whether the raw is telemetry line?
		if(rawTemp < FLIR_TELE_LINE)
		{
			// its a frame data
			// whether the raw is continous?
			if(rawTemp != rawPos_buf)
			{
				// course doesn't match, re-established connection
				flir_reSyc();
				return false;
			}
			// save current raw information
			rawPos_buf ++;
					
			// clear TX finish flag
			flir_TXCpl = false;
			
			// transmit data
			HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)flir_txBuf, LCD_FLIR_SPI_BUF_SIZ);
			
			// return success
			flir_display_startRec();
			
		buf[temp] = idTemp;
		temp++;
		if(temp == 80)
			while(1);			
			return true;
		}	
		else
		{
			// its a telemetry line, what to do now?
			
			// return success
			flir_display_startRec();
			return true;
		}
	}
	else
	{
		// frame not ready. wait and request again
		//HAL_Delay(1);
		HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)flir_rxBuf, LCD_FLIR_RX_BUF_SIZ);
		
		return false;
	}
}
	
 /********************************************************************************************************
 *                                               LOCAL FUNCTIONS
 ********************************************************************************************************/

/*********************************************************************
 * @fn      flir_reSyc
 *
 * @brief   re-synchronize the flir camera
 *
 * @param   none
 *
 * @return  none
 */
bool flir_reSyc( void )
{
	// de-assert CE for at least 185ms
	// FLIR_CS_SET;
	
	// re-synchronize frame. display should reset
	rawPos_buf = 0;
	colPos_buf = 0;
	flir_TXCpl = true;
	flir_RXCpl = false;
	
	// delay 200ms
	HAL_Delay(200);
	
	// assert CE, the flir cammera is now re-synchronized
	// FLIR_CS_RESET;

	// start receive again
	flir_display_startRec();
	
	return true;
}

/*********************************************************************
 * @fn      flir_startSeq
 *
 * @brief   flir camera startup sequence
 *
 * @param   none
 *
 * @return  none
 */
bool flir_startSeq( void )
{
	// assert PWR_DWN_L
	FLIR_PWLOW_RESET;
	// assert RESET
	FLIR_RESETLOW_RESET;
	
	// delay 5000 clk cycle. 25MHz, 5000 cycle, 0.2ms. Delay 1ms here
	HAL_Delay(400);
	
	// de- assert PWR_DWN_L
	FLIR_PWLOW_SET;
	
	HAL_Delay(400);	
	
	// De-assert RESET
	FLIR_RESETLOW_SET;
	
	// assert CE, the flir cammera is now re-synchronized
	// FLIR_CS_RESET;
	
	// delay for 1s
	HAL_Delay(1000);
	
	return true;
}

/*********************************************************************
 * @fn      flir_startDisplay
 *
 * @brief   set parameters to start a display transmit. The function 
 *					should followed by flir_display function to transmit data
 *					At last flir_endDisplay should be call to end an transmision.
 *
 * @param   uint16_t Xpos -> X start position for this tranmission
 *					uint16_t Ypos -> Y start position for this tranmission
 *
 * @return  none
 */
void flir_startDisplay( uint16_t Xpos, uint16_t Ypos )
{
	// set cousor
	LCD_SetCursor(Xpos, Ypos);
	
	// prepare to write
	LCD_WriteRAM_Prepare();    
	
	// set ready for graphic transmit
	SPILCD_CS_RESET;  
	SPILCD_RS_SET;	
}

/*********************************************************************
 * @fn      flir_display_DMAPoll
 *
 * @brief   Using DMA polling to tranmit SPI data to LCD
 *
 * @param   uint8_t *pdata -> data buffer
 *					uint16_t siz -> data size
 *
 * @return  none
 */
void flir_display_DMAPoll(uint8_t *pdata, uint16_t siz)
{
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)pdata, siz);
	// polling for 10ms by defaul
	HAL_DMA_PollForTransfer(&hdma_spi1_tx,HAL_DMA_FULL_TRANSFER,10);
}

/*********************************************************************
 * @fn      flir_endDisplay
 *
 * @brief   Reset SPI parameters. Enable to start another data transmit
 *
 * @param   none
 *
 * @return  none
 */
void flir_endDisplay( void )
{
	// reset SPI interface
	
	//SPILCD_RS_RESET;
	SPILCD_CS_SET;
}

/*********************************************************************
 */
