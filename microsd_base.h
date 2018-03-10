#pragma once

#include <stdint.h>

enum class EC_SD_RESULT {
	OK		= 0,		// 0: Successful
	ERROR   = 1,		// 1: R/W Error
	WRPRT   = 2,		// 2: Write Protected
	NOTRDY  = 3,		// 3: Not Ready
	PARERR  = 4			// 4: Invalid Parameter
};

enum class EC_SD_STATUS {
	OK			= 0,
	NOINIT		= 0x01,	/* Drive not initialized */
	NODISK		= 0x02,	/* No medium in the drive */
	PROTECT		= 0x04	/* Write protected */
};

enum class EC_MICRO_SD_TYPE {
	ERROR			= 0,
	SDSC			= 1,
	SDSD			= 2,
	SDHC_OR_SDXC	= 3
};

class microsd_base {
public:
	//**********************************************************************
	// Метод:
	// 1. Распознает тип карты.
	// 2. Инициализирует ее в соответсвии с ее типом.
	//**********************************************************************
	virtual EC_MICRO_SD_TYPE   initialize		   ( void ) const = 0;

	//**********************************************************************
	// Метод возвращает тип карты, определенный initialize.
	//**********************************************************************
	virtual EC_MICRO_SD_TYPE   get_type				( void ) const = 0;

	//**********************************************************************
	// Основываясь на ранее определенном типе (методом initialize)
	// Производит пробуждение карты.
	// Возвраащет:
	// OK - в случае удачного пробуждения.
	// NOTRDY - если не удалось.
	// ERROR - ошибка интерфейса.
	//**********************************************************************
	virtual EC_SD_RESULT		wake_up				( void ) const = 0;

	// Считать сектор: структура карты, указатель на первый байт, куда будут помещены данные.
	// Адрес внутри microsd. В зависимости от карты: либо первого байта, откуда считать (выравнивание по 512 байт), либо адрес
	// сектора (каждый сектор 512 байт).
	virtual EC_SD_RESULT		read_sector			( uint32_t sector, uint8_t *target_array )		const = 0;

	// Записать по адресу address массив src длинной 512 байт.
	virtual EC_SD_RESULT		write_sector		( uint8_t *source_array, uint32_t sector )	const = 0;

	virtual EC_SD_STATUS		send_status			( void ) const = 0;
};
