#include <stdio.h>
#include "stm32l0xx_hal.h"

#include "lepton.h"

#define LEPTON_RESET_L_HIGH	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define LEPTON_RESET_L_LOW	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)

#define LEPTON_PW_DWN_HIGH	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define LEPTON_PW_DWN_LOW	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

extern SPI_HandleTypeDef hspi2;

#define RING_SIZE (4)
lepton_buffer lepton_buffers[RING_SIZE];
static uint32_t current_buffer_index = 0;
static lepton_xfer_state xfer_state = LEPTON_XFER_STATE_START;

// These replace HAL library functions as they're a lot shorter and more specialized
static inline HAL_StatusTypeDef start_lepton_spi_dma(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static inline HAL_StatusTypeDef setup_lepton_spi_rx(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
static void lepton_spi_rx_dma_cplt(DMA_HandleTypeDef *hdma);

lepton_buffer* get_next_lepton_buffer()
{
  current_buffer_index = ((current_buffer_index + 1) % RING_SIZE);
  lepton_buffer* packet = &lepton_buffers[current_buffer_index];
  packet->status = LEPTON_STATUS_OK;
  return packet;
}

lepton_status complete_lepton_transfer(lepton_buffer* buffer)
{
  // TODO: additional synchronization desired?
  return buffer->status;
}

lepton_buffer* lepton_transfer(void)
{
  lepton_buffer *buf;
  HAL_StatusTypeDef status;

  // DEBUG_PRINTF("Transfer starting: %p@%p\r\n", buf, packet);

  switch (xfer_state)
  {
  default:
  case LEPTON_XFER_STATE_START:
    buf = get_next_lepton_buffer();
    status = setup_lepton_spi_rx(&hspi2, (uint8_t*)(&buf->line[0]), FRAME_TOTAL_LENGTH);
    break;
  case LEPTON_XFER_STATE_SYNC:
    buf = &lepton_buffers[current_buffer_index];
    status = setup_lepton_spi_rx(&hspi2, (uint8_t*)(&buf->line[0]), FRAME_TOTAL_LENGTH);
    break;
  case LEPTON_XFER_STATE_DATA:
    buf = &lepton_buffers[current_buffer_index];
    status = setup_lepton_spi_rx(&hspi2, (uint8_t*)(&buf->line[1]), FRAME_TOTAL_LENGTH);
    break;
  }

  if (status != HAL_OK)
  {
    // DEBUG_PRINTF("Error setting up SPI DMA receive: %d\r\n", status);
    buf->status = LEPTON_STATUS_RESYNC;
    return buf;
  }

  buf->status = LEPTON_STATUS_TRANSFERRING;
  return buf;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  // DEBUG_PRINTF("SPI error!\n\r");
}

static void lepton_spi_rx_dma_cplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef* hspi = ( SPI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  lepton_buffer *buffer = (lepton_buffer *)hspi->pRxBuffPtr;

  /* Disable Rx/Tx DMA Requests and reset some peripheral state */
  hspi->Instance->CR2 &= (uint32_t)(~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));
  hspi->TxXferCount = hspi->RxXferCount = 0;
  hspi->State = HAL_SPI_STATE_READY;

  switch (xfer_state)
  {
  case LEPTON_XFER_STATE_START:
    // get out of start mode; then fall through to the next block
    xfer_state = LEPTON_XFER_STATE_SYNC;

  case LEPTON_XFER_STATE_SYNC:
    // Checking the first line to see if we're in sync yet
    if ((buffer->header[0] & 0x0f00) != 0x0f00)
    {
      xfer_state = LEPTON_XFER_STATE_DATA;
    }
    buffer->status = LEPTON_STATUS_CONTINUE;
    break;

  default:
  case LEPTON_XFER_STATE_DATA:
    // lepton frame complete
    // we started on second line, so (packet - 1) points to the beginning of the buffer
    // buffer = (lepton_buffer*)(packet - 1);
    // uint8_t frame = buffer->lines[IMAGE_OFFSET_LINES + IMAGE_NUM_LINES - 1].header[0] & 0xff;

    // restart for next transfer
    xfer_state = LEPTON_XFER_STATE_START;

    // buffer->status = ((frame == (IMAGE_NUM_LINES - 1)) ? LEPTON_STATUS_OK : LEPTON_STATUS_RESYNC);
    break;
  }
}

void lepton_init(void )
{
  int i;
  for (i = 0; i < RING_SIZE; i++)
  {
    lepton_buffers[i].number = i;
    lepton_buffers[i].status = LEPTON_STATUS_OK;
    // DEBUG_PRINTF("Initialized lepton buffer %d @ %p\r\n", i, &lepton_buffers[i]);
  }

	LEPTON_RESET_L_LOW;
  LEPTON_PW_DWN_LOW;

  HAL_Delay(190);
  LEPTON_PW_DWN_HIGH;

	HAL_Delay(190);
  LEPTON_RESET_L_HIGH;

  hspi2.hdmarx->XferCpltCallback = lepton_spi_rx_dma_cplt;

  /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  is performed in DMA reception complete callback  */
  hspi2.hdmatx->XferCpltCallback = NULL;
  hspi2.hdmatx->XferErrorCallback = NULL;

  /* Clear DBM bit */
  // hspi2.hdmarx->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);
  // hspi2.hdmatx->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /*Init field not used in handle to zero */
  hspi2.RxISR = 0;
  hspi2.TxISR = 0;

  /* Enable SPI peripheral */
  __HAL_SPI_ENABLE(&hspi2);

}

static inline HAL_StatusTypeDef start_lepton_spi_dma(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
	
	HAL_SPI_Receive_DMA(&hspi2, (uint8_t *)DstAddress, DataLength);
//  hdma->Instance->CR &= ~DMA_SxCR_EN;

//  /* Configure DMA Stream data length */
//  hdma->Instance->NDTR = DataLength;

//  /* Memory to Peripheral */
//  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
//  {
//    /* Configure DMA Stream destination address */
//    hdma->Instance->PAR = DstAddress;

//    /* Configure DMA Stream source address */
//    hdma->Instance->M0AR = SrcAddress;
//  }
//  /* Peripheral to Memory */
//  else
//  {
//    /* Configure DMA Stream source address */
//    hdma->Instance->PAR = SrcAddress;

//    /* Configure DMA Stream destination address */
//    hdma->Instance->M0AR = DstAddress;
//  }

//  hdma->Instance->CR |= (DMA_IT_TC | DMA_SxCR_EN);

  return HAL_OK;
}

static inline HAL_StatusTypeDef setup_lepton_spi_rx(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  /* Configure communication */
  hspi->State       = HAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

  hspi->pTxBuffPtr  = hspi->pRxBuffPtr  = (uint8_t*)pData;
  hspi->TxXferSize  = hspi->RxXferSize  = Size;
  hspi->TxXferCount = hspi->RxXferCount = Size;

  /* Enable the Tx DMA Stream */
  start_lepton_spi_dma(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DR, hspi->TxXferCount);

  /* Enable the Rx DMA Stream */
  start_lepton_spi_dma(hspi->hdmarx, (uint32_t)&hspi->Instance->DR, (uint32_t)hspi->pRxBuffPtr, hspi->RxXferCount);

  /* Enable Rx DMA Request */
  /* Enable Tx DMA Request */
  hspi->Instance->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;

  return HAL_OK;

}
