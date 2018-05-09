#include "microsd_card_sdio.h"

#ifdef MODULE_MICROSD_CARD_SDIO_ENABLED

MicrosdSdio::MicrosdSdio ( const microsd_sdio_cfg_t* const cfg ) : cfg( cfg ) {
	this->handle.Instance						= SDIO;
	this->handle.Init.ClockEdge					= SDIO_CLOCK_EDGE_RISING;
	this->handle.Init.ClockBypass				= SDIO_CLOCK_BYPASS_DISABLE;
	this->handle.Init.ClockPowerSave			= SDIO_CLOCK_POWER_SAVE_DISABLE;
	this->handle.Init.BusWide					= SDIO_BUS_WIDE_1B;
	this->handle.Init.HardwareFlowControl		= SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	this->handle.Init.ClockDiv					= cfg->div;

	this->handle.obj								= this;

	if (  this->cfg->dmaRx != nullptr ) {
		this->handle.hdmarx								= &this->dmaRx;

		this->handle.hdmarx->Instance					= this->cfg->dmaRx;
		this->handle.hdmarx->Init.Channel				= this->cfg->dmaRxCh;
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

		this->handle.hdmarx->Parent					 = &this->handle;
	}

	this->m = USER_OS_STATIC_MUTEX_CREATE( &mb );
	this->s = USER_OS_STATIC_BIN_SEMAPHORE_CREATE( &this->sb );
}

void MicrosdSdio::dmaRxHandler ( void ) {
	HAL_DMA_IRQHandler( &this->dmaRx );
}

void MicrosdSdio::dmaTxHandler ( void ) {
	HAL_DMA_IRQHandler( &this->hdma_tx );
}

void MicrosdSdio::sdioHandler ( void ) {
	HAL_SD_IRQHandler( &this->handle );
}

EC_MICRO_SD_TYPE MicrosdSdio::initialize ( void ) {
	HAL_StatusTypeDef r;

	if ( HAL_SD_GetState( &this->handle ) == HAL_SD_STATE_RESET ) {		/// Первый запуск.
		if (  this->cfg->dmaRx != nullptr ) {
			dmaClkOn( this->cfg->dmaRx );
			r = HAL_DMA_DeInit( &this->dmaRx );
			if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;
			r = HAL_DMA_Init( &this->dmaRx );
			if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;

			dmaIrqOn( this->cfg->dmaRx, this->cfg->dmaRxIrqPrio );
		}

		if ( this->cfg->dmaRx == nullptr ) {
			NVIC_SetPriority( SDIO_IRQn, this->cfg->sdioIrqPrio );
			NVIC_EnableIRQ( SDIO_IRQn );
		}

		__HAL_RCC_SDIO_CLK_ENABLE();

		r = HAL_SD_DeInit( &this->handle );
		if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;
		r = HAL_SD_Init( &this->handle );
		if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;
		r = HAL_SD_ConfigWideBusOperation( &this->handle, this->cfg->wide );
		if ( r != 0 ) return EC_MICRO_SD_TYPE::ERROR;

		return this->getType();
	} else {	/// Интерфейс уже настроен, надо карту.
		r = HAL_SD_InitCard( &this->handle );
		if ( r != HAL_StatusTypeDef::HAL_OK )
			return EC_MICRO_SD_TYPE::ERROR;
		return this->getType();
	}
}

EC_MICRO_SD_TYPE MicrosdSdio::getType ( void ) {
	EC_MICRO_SD_TYPE t;
	t = ( this->handle.SdCard.CardVersion == CARD_V1_X ) ? EC_MICRO_SD_TYPE::SD1 : EC_MICRO_SD_TYPE::SD2;
	if ( this->handle.SdCard.CardType == CARD_SDHC_SDXC ) {
		t = ( EC_MICRO_SD_TYPE )((uint32_t)t | ( uint32_t )EC_MICRO_SD_TYPE::BLOCK);
	}

	return t;
}

EC_SD_RESULT MicrosdSdio::readSector ( uint32_t sector, uint8_t *target_array, uint32_t cout_sector, uint32_t timeout_ms )	{
	USER_OS_TAKE_MUTEX( this->m, portMAX_DELAY );

	EC_SD_RESULT rv = EC_SD_RESULT::ERROR;
	HAL_StatusTypeDef r;

	xSemaphoreTake ( this->s, 0 );

	if ( this->cfg->dmaRx != nullptr ) {
		r = HAL_SD_ReadBlocks_DMA( &this->handle, target_array, sector, cout_sector );
	} else {
		r = HAL_SD_ReadBlocks_IT( &this->handle, target_array, sector, cout_sector );
	}

	if ( r == HAL_OK ) {
		if ( xSemaphoreTake ( this->s, timeout_ms ) == pdTRUE ) {
			rv = EC_SD_RESULT::OK;
		};

		/// Ждем отзыва о том, что карта снова готова.
		if ( rv == EC_SD_RESULT::OK ) {
			uint32_t timeout_flag = 1000;
			while( timeout_flag ) {
				if ( HAL_SD_GetCardState( &this->handle ) == HAL_SD_CARD_TRANSFER ) break;
				timeout_flag--;
			}
			if ( timeout_flag == 0 ) {
				rv = EC_SD_RESULT::ERROR;
			}
		}
	}

	USER_OS_GIVE_MUTEX( this->m );

	return rv;
}

EC_SD_RESULT MicrosdSdio::writeSector ( const uint8_t* const source_array, uint32_t sector, uint32_t cout_sector, uint32_t timeout_ms ) {
	USER_OS_TAKE_MUTEX( this->m, portMAX_DELAY );

	EC_SD_RESULT rv = EC_SD_RESULT::ERROR;
	HAL_StatusTypeDef r;

	__disable_irq();
	r = HAL_SD_WriteBlocks( &this->handle, (uint8_t*)source_array, sector, cout_sector, timeout_ms );
	__enable_irq();

	if ( r == HAL_OK ) {
		rv = EC_SD_RESULT::OK;
		uint32_t timeout_flag = 1000;
		while( timeout_flag ) {
			if ( HAL_SD_GetCardState( &this->handle ) == HAL_SD_CARD_TRANSFER ) break;
			timeout_flag--;
		}
		if ( timeout_flag == 0 ) {
			rv = EC_SD_RESULT::ERROR;
		}
	}

	USER_OS_GIVE_MUTEX( this->m );

	return rv;
}

void MicrosdSdio::giveSemaphore ( void ) {
	if ( this->s ) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR ( this->s, &xHigherPriorityTaskWoken);
	}
}

extern "C" {

void HAL_SD_RxCpltCallback( SD_HandleTypeDef *hsd ) {
	MicrosdSdio* o = ( MicrosdSdio* )hsd->obj;
	o->giveSemaphore();
}

}

EC_SD_STATUS MicrosdSdio::getStatus ( void ) {
	if ( this->handle.State == HAL_SD_STATE_RESET ) {
		return  EC_SD_STATUS::NOINIT;
	}

	HAL_SD_CardStateTypeDef s = HAL_SD_GetCardState( &this->handle );
	if ( s == HAL_SD_CARD_TRANSFER ) {
		return  EC_SD_STATUS::OK;
	} else {
		return  EC_SD_STATUS::NOINIT;
	}
}

EC_SD_RESULT MicrosdSdio::getSectorCount ( uint32_t& sectorCount ) {
	if ( this->handle.State == HAL_SD_STATE_RESET ) {
		return  EC_SD_RESULT::ERROR;
	}

	sectorCount = this->handle.SdCard.LogBlockNbr;
	return EC_SD_RESULT::OK;
}

EC_SD_RESULT MicrosdSdio::getBlockSize ( uint32_t& blockSize ) {
	if ( this->handle.State == HAL_SD_STATE_RESET ) {
		return  EC_SD_RESULT::ERROR;
	}

	blockSize = this->handle.SdCard.LogBlockSize;
	return EC_SD_RESULT::OK;
}

#endif
