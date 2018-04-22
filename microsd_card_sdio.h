#pragma once

#include "mc_hardware_interfaces_pin.h"
#include "user_os.h"
#include "microsd_base.h"

#ifdef STM32F2

#endif

#include "stm32f2xx_hal_sd.h"
#include "stm32f2xx_hal_rcc.h"
#include "stm32f2xx_hal_dma.h"

struct microsd_sdio_cfg_t {
	uint32_t					wide;				/// SDIO_BUS_WIDE_1B, SDIO_BUS_WIDE_4B, SDIO_BUS_WIDE_8B.
	uint32_t					div;				/// IS_SDIO_CLKDIV( value ).

	/// DMA обязателен.
	DMA_Stream_TypeDef*         dma_rx;				/// Из мерии DMAx_Streamx.
	uint32_t                    dma_rx_ch;			/// Из серии DMA_CHANNEL_x.
	uint8_t						dma_rx_irq_prio;	/// Если задан dma_rx.
	uint8_t						sdio_irq_prio;		/// Задается если не указан dma_rx. Передача всегда идет в ручную (без DMA и IT).
};


class MicrosdSdio : public MicrosdBase {
public:
	MicrosdSdio ( const microsd_sdio_cfg_t* const cfg );

	EC_MICRO_SD_TYPE	initialize				( void );

	EC_MICRO_SD_TYPE	getType					( void );

	EC_SD_RESULT		readSector				( uint32_t sector,
												  uint8_t *target_array,
												  uint32_t cout_sector,
												  uint32_t timeout_ms  );

	EC_SD_RESULT		writeSector				( const uint8_t* const source_array,
												  uint32_t sector,
												  uint32_t cout_sector,
												  uint32_t timeout_ms  );

	EC_SD_STATUS		getStatus				( void );

	void	dmaRxHandler	( void );
	void	dmaTxHandler	( void );
	void	sdioHandler		( void );

	void    giveSemaphore ( void );         // Отдать симафор из прерывания (внутренняя функция.
private:
	const microsd_sdio_cfg_t* const cfg;

	SD_HandleTypeDef							handle;
	DMA_HandleTypeDef							hdma_tx;
	DMA_HandleTypeDef							hdma_rx;

	USER_OS_STATIC_MUTEX                    	m = nullptr;
	USER_OS_STATIC_MUTEX_BUFFER             	mb;

	USER_OS_STATIC_BIN_SEMAPHORE_BUFFER     	sb;
	USER_OS_STATIC_BIN_SEMAPHORE				s = nullptr;
};