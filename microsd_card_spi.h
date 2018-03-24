#pragma once

#include "mc_hardware_interfaces_spi.h"
#include "mc_hardware_interfaces_pin.h"
#include "user_os.h"
#include "microsd_base.h"

struct microsd_spi_cfg_t {
	const pin_base*			 const cs;			 // Вывод CS, подключенный к microsd.
			spi_master_8bit_base* const p_spi;
	const uint32_t				slow;
	const uint32_t				fast;
};

class microsd_spi : public microsd_base {
public:
	microsd_spi ( const microsd_spi_cfg_t* const cfg );

	EC_MICRO_SD_TYPE	initialize					( void )									const;
	EC_MICRO_SD_TYPE	get_type					( void )									const;
	EC_SD_RESULT		read_sector					( uint32_t sector, uint8_t *target_array, uint32_t cout_sector, uint32_t timeout_ms  )	const;
	EC_SD_RESULT		write_sector				( const uint8_t* const source_array, uint32_t sector, uint32_t cout_sector, uint32_t timeout_ms  )	const;
	EC_SD_STATUS		get_status					( void )									const;

private:
	// Переключение CS.
	void			cs_low							( void )									const;		 // CS = 0, GND.
	void			cs_high							( void )									const;		 // CS = 1, VDD.

	// Передать count пустых байт (шлем 0xFF).
	EC_SD_RES	send_empty_package					( uint16_t count )							const;

	// Передача 1 пустого байта. Требуется после каждой команды для ОЧЕНЬ старых карт.
	EC_SD_RES	send_wait_one_package				( void )									const;

	// Пропускаем count приших байт.
	EC_SD_RES	lose_package						( uint16_t count )							const;

	// Ждем от команды специального маркера.
	EC_SD_RES	wait_mark							( uint8_t mark )							const;

	// Сами отправляем маркер.
	EC_SD_RES	send_mark							( uint8_t mark )							const;

	// Просто передача команды.
	EC_SD_RES	send_cmd							( uint8_t cmd, uint32_t arg, uint8_t crc )	const;

	// Получаем адресс сектора (для аргумента команды чтения/записи).
	uint32_t	get_arg_address						( uint32_t sector )							const;

	// Отправить ACMD.
	EC_SD_RES	send_acmd							( uint8_t acmd, uint32_t arg, uint8_t crc )	const;


	// Ждать R1 (если r1 != nullptr, то еще вернуть R1 ).
	EC_SD_RES	wait_r1								( uint8_t* r1 = nullptr )					const;

	EC_SD_RES	wait_r2								( uint16_t* r2 )							const;

	// Принимаем R3 (регистр OCR).
	EC_SD_RES	wait_r3								( uint32_t* r3 )							const;

	// Принимаем r7.
	EC_SD_RES	wait_r7								( uint32_t* r7 )							const;

	const microsd_spi_cfg_t*	const cfg;

	mutable USER_OS_STATIC_MUTEX_BUFFER	 mutex_buf;
	mutable USER_OS_STATIC_MUTEX			mutex			 = nullptr;

	mutable EC_MICRO_SD_TYPE				type_microsd	= EC_MICRO_SD_TYPE::ERROR;			 // Тип microSD.
};
