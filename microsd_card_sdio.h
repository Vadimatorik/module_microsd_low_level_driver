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


class microsd_sdio : public microsd_base {
public:
	microsd_sdio ( const microsd_sdio_cfg_t* const cfg );

	EC_MICRO_SD_TYPE	initialize					( void )									const;
	EC_MICRO_SD_TYPE	get_type					( void )									const;
	EC_SD_RESULT		read_sector					( uint32_t sector, uint8_t *target_array, uint32_t cout_sector, uint32_t timeout_ms  )	const;
	EC_SD_RESULT		write_sector				( const uint8_t* const source_array, uint32_t sector, uint32_t cout_sector, uint32_t timeout_ms  )	const;
	EC_SD_STATUS		get_status					( void ) const;

	void dma_rx_handler ( void );
	void dma_tx_handler ( void );
	void sdio_handler ( void );

    void    give_semaphore ( void );         // Отдать симафор из прерывания (внутренняя функция.
private:
	const microsd_sdio_cfg_t* const cfg;

	mutable	SD_HandleTypeDef							handle;
	mutable DMA_HandleTypeDef							hdma_tx;
	mutable DMA_HandleTypeDef							hdma_rx;

	mutable USER_OS_STATIC_MUTEX                    	m = nullptr;
	mutable USER_OS_STATIC_MUTEX_BUFFER             	mb;

    mutable USER_OS_STATIC_BIN_SEMAPHORE_BUFFER     	sb;
    mutable USER_OS_STATIC_BIN_SEMAPHORE				s = nullptr;
};
