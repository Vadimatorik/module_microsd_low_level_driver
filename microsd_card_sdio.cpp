#include "microsd_card_sdio.h"
#include "dma.h"

microsd_sdio::microsd_sdio ( const microsd_sdio_cfg_t* const cfg ) : cfg( cfg ) {
	this->handle.Instance			= SDIO;
	this->handle.Init.ClockEdge					= SDIO_CLOCK_EDGE_RISING;
	this->handle.Init.ClockBypass				= SDIO_CLOCK_BYPASS_DISABLE;
	this->handle.Init.ClockPowerSave			= SDIO_CLOCK_POWER_SAVE_DISABLE;
	this->handle.Init.BusWide					= cfg->wide;
	this->handle.Init.HardwareFlowControl		= SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	this->handle.Init.ClockDiv					= cfg->div;

	this->handle.obj								= this;

	/// DMA обязателен! На AT не рассматривается.
	this->handle.hdmatx								= &this->hdma_tx;
	this->handle.hdmatx->Instance					= this->cfg->dma_tx;
	this->handle.hdmatx->Init.Channel				= this->cfg->dma_tx_ch;
	this->handle.hdmatx->Init.Direction				= DMA_MEMORY_TO_PERIPH;
	this->handle.hdmatx->Init.PeriphInc				= DMA_PINC_DISABLE;
	this->handle.hdmatx->Init.MemInc				= DMA_MINC_ENABLE;
	this->handle.hdmatx->Init.PeriphDataAlignment	= DMA_PDATAALIGN_WORD;
	this->handle.hdmatx->Init.MemDataAlignment		= DMA_MDATAALIGN_WORD;
	this->handle.hdmatx->Init.Mode					= DMA_PFCTRL;
	this->handle.hdmatx->Init.Priority				= DMA_PRIORITY_LOW;
	this->handle.hdmatx->Init.FIFOMode				= DMA_FIFOMODE_ENABLE;
	this->handle.hdmatx->Init.FIFOThreshold			= DMA_FIFO_THRESHOLD_FULL;
	this->handle.hdmatx->Init.MemBurst				= DMA_MBURST_INC4;
	this->handle.hdmatx->Init.PeriphBurst			= DMA_PBURST_INC4;

	this->handle.hdmarx								= &this->hdma_rx;
	this->handle.hdmarx->Instance					= this->cfg->dma_rx;
	this->handle.hdmarx->Init.Channel				= this->cfg->dma_rx_ch;
	this->handle.hdmarx->Init.Direction				= DMA_PERIPH_TO_MEMORY;
	this->handle.hdmarx->Init.PeriphInc				= DMA_PINC_DISABLE;
	this->handle.hdmarx->Init.MemInc				= DMA_MINC_ENABLE;
	this->handle.hdmarx->Init.PeriphDataAlignment	= DMA_PDATAALIGN_WORD;
	this->handle.hdmarx->Init.MemDataAlignment		= DMA_MDATAALIGN_WORD;
	this->handle.hdmarx->Init.Mode					= DMA_PFCTRL;
	this->handle.hdmarx->Init.Priority				= DMA_PRIORITY_LOW;
	this->handle.hdmarx->Init.FIFOMode				= DMA_FIFOMODE_ENABLE;
	this->handle.hdmarx->Init.FIFOThreshold			= DMA_FIFO_THRESHOLD_FULL;
	this->handle.hdmarx->Init.MemBurst				= DMA_MBURST_INC4;
	this->handle.hdmarx->Init.PeriphBurst			= DMA_PBURST_INC4;

	this->handle.hdmarx                             = &this->hdma_rx;
	this->handle.hdmarx->Parent                     = &this->handle;

	this->handle.hdmatx                             = &this->hdma_tx;
	this->handle.hdmatx->Parent                     = &this->handle;

    this->m = USER_OS_STATIC_MUTEX_CREATE( &mb );
    this->s = USER_OS_STATIC_BIN_SEMAPHORE_CREATE( &this->sb );
}

void microsd_sdio::dma_rx_handler ( void ) {
	HAL_DMA_IRQHandler( &this->hdma_rx );
}

void microsd_sdio::dma_tx_handler ( void ) {
	HAL_DMA_IRQHandler( &this->hdma_tx );
}

void microsd_sdio::sdio_handler ( void ) {
	HAL_SD_IRQHandler( &this->handle );
}

EC_MICRO_SD_TYPE microsd_sdio::initialize ( void ) const {
	HAL_StatusTypeDef r;
	dma_clk_on( this->cfg->dma_tx );
	r = HAL_DMA_DeInit( &this->hdma_tx );
	if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;
	r = HAL_DMA_Init( &this->hdma_tx );
	if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;
	dma_clk_on( this->cfg->dma_rx );
	r = HAL_DMA_DeInit( &this->hdma_rx );
	if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;
	r = HAL_DMA_Init( &this->hdma_rx );
	if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;

	dma_irq_on( this->cfg->dma_tx, 6 );
	dma_irq_on( this->cfg->dma_rx, 6 );

	NVIC_SetPriority( SDIO_IRQn, 6 );
	NVIC_EnableIRQ( SDIO_IRQn );

	__HAL_RCC_SDIO_CLK_ENABLE();

	r = HAL_SD_Init( &this->handle );
	if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;
	return this->get_type();
}

EC_MICRO_SD_TYPE microsd_sdio::get_type ( void ) const {
	switch ( this->handle.SdCard.CardType ) {
		case CARD_SECURED:				return EC_MICRO_SD_TYPE::ERROR;
		case CARD_SDSC:					return EC_MICRO_SD_TYPE::SDSC;
		case CARD_SDHC_SDXC:			return EC_MICRO_SD_TYPE::SDHC_OR_SDXC;
		default:						return EC_MICRO_SD_TYPE::SDSD;
	}
}

EC_SD_RESULT microsd_sdio::read_sector ( uint32_t sector, uint8_t *target_array, uint32_t cout_sector, uint32_t timeout_ms  )	const {
	if ( this->m != nullptr )			USER_OS_TAKE_MUTEX( this->m, portMAX_DELAY );

	EC_SD_RESULT rv = EC_SD_RESULT::ERROR;
    xSemaphoreTake ( this->s, 0 );

	HAL_StatusTypeDef r;
	r = HAL_SD_ReadBlocks_DMA( &this->handle, target_array, sector, cout_sector );

	if ( r == HAL_OK ) {

	if ( xSemaphoreTake ( this->s, timeout_ms ) == pdTRUE ) {
		rv = EC_SD_RESULT::OK;
	}

	}

	if ( this->m != nullptr )			USER_OS_GIVE_MUTEX( this->m );

	return rv;
}

EC_SD_RESULT microsd_sdio::write_sector ( const uint8_t* const source_array, uint32_t sector, uint32_t cout_sector, uint32_t timeout_ms ) const {
	if ( this->m != nullptr )			USER_OS_TAKE_MUTEX( this->m, portMAX_DELAY );

	EC_SD_RESULT rv = EC_SD_RESULT::ERROR;
    xSemaphoreTake ( this->s, 0 );

	HAL_StatusTypeDef r;
	r = HAL_SD_WriteBlocks_DMA( &this->handle, (uint8_t*)source_array, sector, cout_sector );

	if ( r == HAL_OK ) {

		if ( xSemaphoreTake ( this->s, timeout_ms ) == pdTRUE ) {
			rv = EC_SD_RESULT::OK;
		}

	}
	if ( this->m != nullptr )			USER_OS_GIVE_MUTEX( this->m );

	return rv;
}

void microsd_sdio::give_semaphore ( void ) {
    if ( this->s ) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR ( this->s, &xHigherPriorityTaskWoken);
    }
}

extern "C" {

void HAL_SD_TxCpltCallback( SD_HandleTypeDef *hsd ) {
	microsd_sdio* o = ( microsd_sdio* )hsd->obj;
    o->give_semaphore();
}

void HAL_SD_RxCpltCallback( SD_HandleTypeDef *hsd ) {
	microsd_sdio* o = ( microsd_sdio* )hsd->obj;
    o->give_semaphore();
}

}

EC_SD_STATUS microsd_sdio::send_status ( void )  const {
	//HAL_SD_StateTypeDef s = HAL_SD_GetState( &this->handle );

	/// Допилить, если будет надо.
	return  EC_SD_STATUS::OK;
}
