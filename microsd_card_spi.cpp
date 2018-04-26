#include "microsd_card_spi.h"

#define CMD0		( 0x40 )														  // Программный сброс.
#define CMD1		( 0x40 + 1)													   // Инициировать процесс инициализации.
#define CMD8		( 0x40 + 8 )													  // Уточнить поддерживаемое нарпряжение.
#define CMD13		( 0x40 + 13 )														// Статус карты, если вставлена.
#define CMD17	   ( 0x40 + 17 )													 // Считать блок.
#define CMD24	   ( 0x40 + 24 )													 // Записать блок.
#define CMD55	   ( 0x40 + 55 )													 // Указание, что далее ACMD.
#define CMD58	   ( 0x40 + 58 )													 // Считать OCR регистр карты.

#define ACMD41	  ( 0x40 + 41 )													 // Инициировать процесс инициализации.
#define ACMD55	  ( 0x40 + 55 )													 // Инициировать процесс инициализации.

#define CMD17_MARK  ( 0b11111110 )
#define CMD24_MARK  ( 0b11111110 )


MicrosdSpi::MicrosdSpi ( const microsdSpiCfg* const cfg ) : cfg( cfg ) {
	this->m = USER_OS_STATIC_MUTEX_CREATE( &this->mb );
}

//**********************************************************************
// Низкоуровневые функции.
//**********************************************************************

// Управление линией CS.
void MicrosdSpi::csLow ( void ) {
	this->cfg->cs->reset();
}

void MicrosdSpi::csHigh ( void ) {
	this->cfg->cs->set();
}

// Передать count 0xFF.
EC_SD_RES MicrosdSpi::sendEmptyPackage ( uint16_t count ) {
	if ( this->cfg->s->txOneItem( 0xFF, count, 10 ) == BASE_RESULT::OK ) {
		return EC_SD_RES::OK;
	} else {
		return EC_SD_RES::IO_ERROR;
	}
}

// Передача одного 0xFF. Требуется после каждой команды для ОЧЕНЬ старых карт.
EC_SD_RES MicrosdSpi::sendWaitOnePackage ( void ) {
	this->csLow();
	EC_SD_RES r = this->sendEmptyPackage( 1 );
	this->csHigh();
	return r;
}

// Ждем от карты "маркер"
// - специальный байт, показывающий, что далее идет команда/данные.
EC_SD_RES MicrosdSpi::waitMark ( uint8_t mark ) {
	EC_SD_RES  r = EC_SD_RES::TIMEOUT;

	this->csLow();

	for ( int l_d = 0; l_d < 3; l_d++ ) {											   // До 3 мс даем возможность карте ответить.
		for ( int loop = 0; loop < 10; loop++ ) {
			uint8_t input_buf;

			if ( this->cfg->s->rx( &input_buf, 1, 10, 0xFF ) != BASE_RESULT::OK ) {
				r = EC_SD_RES::IO_ERROR;
				break;
			}

			if ( input_buf == mark ) {
				r = EC_SD_RES::OK;
				break;
			}
		}
		if ( r != EC_SD_RES::TIMEOUT ) break;
		USER_OS_DELAY_MS(1);
	}

	this->csHigh();

	return r;
}

// Пропустить n байт.
EC_SD_RES MicrosdSpi::losePackage ( uint16_t count ) {
	this->csLow();
	EC_SD_RES r = this->sendEmptyPackage( count );
	this->csHigh();
	return r;
}

EC_SD_RES MicrosdSpi::sendCmd ( uint8_t cmd, uint32_t arg, uint8_t crc ) {
	this->csLow();

	uint8_t output_package[6];
	output_package[0] = cmd;
	output_package[1] = ( uint8_t )( arg >> 24 );
	output_package[2] = ( uint8_t )( arg >> 16 );
	output_package[3] = ( uint8_t )( arg >> 8 );
	output_package[4] = ( uint8_t )( arg );
	output_package[5] = crc;

	EC_SD_RES r = EC_SD_RES::OK;
	if ( this->cfg->s->tx( output_package, 6, 10 ) != BASE_RESULT::OK ) {
		r = EC_SD_RES::IO_ERROR;
	}

	this->csHigh();

	return r;
}

// Сами отправляем маркер (нужно, например, для записи).
EC_SD_RES MicrosdSpi::sendMark ( uint8_t mark ) {
	this->csLow();

	EC_SD_RES r = EC_SD_RES::OK;
	if ( this->cfg->s->tx( &mark, 1, 10 ) != BASE_RESULT::OK ) {
		r = EC_SD_RES::IO_ERROR;
	}

	this->csHigh();

	return r;
}

// Ожидаем R1 (значение R1 нам не нужно).
EC_SD_RES MicrosdSpi::waitR1 ( uint8_t* r1 ) {
	this->csLow();
	EC_SD_RES  r = EC_SD_RES::TIMEOUT;

	// Карта должна принять команду в течении 3 обращений (чаще всего на 2-й итерации).
	for ( int loop = 0; loop < 3; loop++ ) {
		uint8_t input_buf;

		if ( this->cfg->s->rx( &input_buf, 1, 10, 0xFF ) != BASE_RESULT::OK ) {
			r = EC_SD_RES::IO_ERROR;
			break;
		}

		// Сброшенный старший бит символизирует успешное принятие R1 ответа.
		if ( ( input_buf & ( 1 << 7 ) ) == 0 ) {
			if ( r1 != nullptr ) {
				*r1 = input_buf;
			}
			r = EC_SD_RES::OK;
			break;
		}
	}

	this->csHigh();
	return r;
}

#define R1_ILLEGAL_COMMAND_MSK		( 1 << 2 )

// Ждем R3 (регистр OCR).
EC_SD_RES MicrosdSpi::waitR3 ( uint32_t* r3 ) {
	uint8_t r1;
	EC_SD_RES r = this->waitR1( &r1 );

	if ( r == EC_SD_RES::OK ) {
		if ( r1 & R1_ILLEGAL_COMMAND_MSK ) {
			return EC_SD_RES::R1_ILLEGAL_COMMAND;
		}
	} else {
		return r;
	}

	this->csLow();

	uint8_t buf_u8[4];

	if ( this->cfg->s->rx( buf_u8, 4, 10, 0xFF ) != BASE_RESULT::OK ) {
		r = EC_SD_RES::IO_ERROR;
	}

	uint32_t buf_u32 = 0;
	buf_u32 |= buf_u8[ 0 ] << 24;
	buf_u32 |= buf_u8[ 1 ] << 16;
	buf_u32 |= buf_u8[ 2 ] << 8;
	buf_u32 |= buf_u8[ 3 ];

	*r3 = buf_u32;

	this->csHigh();

	return r;
}

EC_SD_RES MicrosdSpi::waitR2 ( uint16_t* r2 ) {
	uint8_t r1;
	EC_SD_RES r = this->waitR1( &r1 );

	if ( r == EC_SD_RES::OK ) {
		if ( r1 & R1_ILLEGAL_COMMAND_MSK ) {
			return EC_SD_RES::R1_ILLEGAL_COMMAND;
		}
	} else {
		return r;
	}

	this->csLow();

	uint8_t buf_u8;

	if ( this->cfg->s->rx( &buf_u8, 1, 10, 0xFF ) != BASE_RESULT::OK ) {
		r = EC_SD_RES::IO_ERROR;
	}

	uint16_t buf_u16 = 0;
	buf_u16 |= r1 << 8;
	buf_u16 |= buf_u8;

	*r2 = buf_u16;

	this->csHigh();

	return r;
}


// Ждем ответа r7.
EC_SD_RES MicrosdSpi::waitR7 ( uint32_t* r7 ) {
	return this->waitR3( r7 );   // Структура r3 и r7 идентичны по формату. Так экономим память.
}

// Передаем CMD55,
// Дожидаемся сообщения об успешном принятии.
// Если не удалось принять - информируем об ошибке и выходим.
EC_SD_RES MicrosdSpi::sendAcmd ( uint8_t acmd, uint32_t arg, uint8_t crc ) {
	EC_SD_RES r = this->sendCmd( CMD55, 0, 0 );
	if ( r != EC_SD_RES::OK )			  return r;

	r = this->waitR1();
	if ( r != EC_SD_RES::OK )			  return r;

	r = this->sendCmd( acmd, arg, crc );
	return r;
}

// Получая сектор, возвращает адресс, который следует отправить с параметром карты.
// Иначе говоря, в зависимости от типа адресации либо возвращает тот же номер сектора,
// либо номер первого байта.
uint32_t MicrosdSpi::getArgAddress ( uint32_t sector ) {
	if ( ( this->typeMicrosd == EC_MICRO_SD_TYPE::SDSC ) || ( this->typeMicrosd == EC_MICRO_SD_TYPE::SDSD ) ) {
		return 0x200 * sector;
	} else {
		return sector;
	}
}

//**********************************************************************
// Основной функционал.
//**********************************************************************
#define R1_ILLEGAL_COMMAND_MSK				( 1 << 2 )
#define R1_IN_IDLE_STATE_MSK				( 1 << 0 )
#define ACMD41_HCS_MSK						( 1 << 30 )
#define OCR_CCS_MSK							( 1 << 30 )
// Определяем тип карты и инициализируем ее.
EC_MICRO_SD_TYPE MicrosdSpi::initialize ( void ) {
	EC_SD_RES res_op;

	uint8_t			r1;
	uint32_t		r7;

	USER_OS_TAKE_MUTEX( this->m, portMAX_DELAY );
	this->cfg->s->setPrescaler( this->cfg->slow );
	this->typeMicrosd = EC_MICRO_SD_TYPE::ERROR;

	do {
		if ( this->sendCmd( CMD0, 0, 0x95 )		!= EC_SD_RES::OK )					break;				// Программный сброс.
		if ( this->waitR1()						!= EC_SD_RES::OK )					break;				// Значение R1 не важно.
		if ( this->sendCmd( CMD8, 0x1AA, 0x87 )	!= EC_SD_RES::OK )					break;				// 2.7-3.6 В.
		res_op = this->waitR7( &r7 );
		if ( ( res_op == EC_SD_RES::IO_ERROR ) || ( res_op == EC_SD_RES::TIMEOUT) )		break;
		if ( res_op == EC_SD_RES::R1_ILLEGAL_COMMAND ) {
			while(1);
		} else {
			// В ответ на CMD8 c параметром 1 ( 2.7-3.6 В ) + 0xAA (тестовое значение, выбранное на обум)
			// должно прийти эхо ( параметр 1 и 0xAA ).
			if ( r7 != 0x1AA )															break;

			int l = 500;
			for ( ; l != 0; l-- ) {
				if ( this->sendAcmd( ACMD41, ACMD41_HCS_MSK, 0 ) != EC_SD_RES::OK )	break;
				if ( this->waitR1( &r1 )			!= EC_SD_RES::OK )					break;
				if ( r1 == 0 )															break;
				USER_OS_DELAY_MS( 1 );
			}

			if ( !l ) 																	break;				// Закончились попытки инициализировать.

			if ( this->sendCmd( CMD58, 0, 0 )		!= EC_SD_RES::OK )					break;
			uint32_t ocr;
			res_op = this->waitR3( &ocr );
			if ( res_op != EC_SD_RES::OK )												break;
			if ( ocr & OCR_CCS_MSK ) {
				this->typeMicrosd = EC_MICRO_SD_TYPE::SDHC_OR_SDXC;
			} else {
				this->typeMicrosd = EC_MICRO_SD_TYPE::SDSD;
			}

		}

	} while ( false );

	// Теперь с SD можно работать на высоких скоростях.
	if ( this->typeMicrosd != EC_MICRO_SD_TYPE::ERROR ) {
		this->cfg->s->setPrescaler( this->cfg->fast );
	}

	USER_OS_GIVE_MUTEX( this->m );

	return this->typeMicrosd;
}

EC_SD_STATUS MicrosdSpi::getStatus ( void ) {
	if ( this->getType() == EC_MICRO_SD_TYPE::ERROR ) {
		return EC_SD_STATUS::NOINIT;
	}

	uint16_t		r2;

	USER_OS_TAKE_MUTEX( this->m, portMAX_DELAY );
	this->cfg->s->setPrescaler( this->cfg->slow );
	EC_SD_STATUS r = EC_SD_STATUS::NODISK;

	do {
		if ( this->sendCmd( CMD13, 0, 0 )	!= EC_SD_RES::OK )					break;
		EC_SD_RES res_r2 = this->waitR2( &r2 );

		if ( res_r2	!= EC_SD_RES::OK ) {
			USER_OS_GIVE_MUTEX( this->m );
			this->initialize();
			continue;
		}
	} while( 0 );

	if ( r2 & ( R1_IN_IDLE_STATE_MSK << 8 ) ) {
		r = EC_SD_STATUS::NOINIT;
	} else {
		r = EC_SD_STATUS::OK;
	}

	USER_OS_GIVE_MUTEX( this->m );

	return r;
}

EC_MICRO_SD_TYPE MicrosdSpi::getType ( void ) {
	return this->typeMicrosd;
}

// Считать сектор.
// dst - указатель на массив, куда считать 512 байт.
// sector - требуемый сектор, с 0.
// Предполагается, что с картой все хорошо (она определена, инициализирована).

EC_SD_RESULT MicrosdSpi::readSector ( uint32_t sector, uint8_t *target_array, uint32_t cout_sector, uint32_t timeout_ms  ) {
	( void )timeout_ms;

	uint32_t address;

	this->cfg->s->setPrescaler( this->cfg->fast );

	EC_SD_RESULT r = EC_SD_RESULT::ERROR;


	USER_OS_TAKE_MUTEX( this->m, portMAX_DELAY );

	uint8_t* p_buf = target_array;

	do {
		address = this->getArgAddress( sector );											  // В зависимости от типа карты - адресация может быть побайтовая или поблочная
																									// (блок - 512 байт).

		if ( this->sendCmd( CMD17, address, 0 )	 != EC_SD_RES::OK ) break;		  // Отправляем CMD17.
		uint8_t r1;
		if ( this->waitR1( &r1 )					!= EC_SD_RES::OK ) break;
		if ( r1 != 0 ) break;
		if ( this->waitMark( CMD17_MARK )		   != EC_SD_RES::OK ) break;

		// Считываем 512 байт.
		this->csLow();

		if ( this->cfg->s->rx( p_buf, 512, 100, 0xFF )	  != BASE_RESULT::OK ) break;
		uint8_t crc_in[2] = {0xFF, 0xFF};

		if ( this->cfg->s->rx( crc_in, 2, 10, 0xFF )			   != BASE_RESULT::OK ) break;
		if ( this->sendWaitOnePackage()		   != EC_SD_RES::OK ) break;

		if ( cout_sector == 0 ) {
			r = EC_SD_RESULT::OK;
			break;
		} else {
			sector++;							// Будем писать следующий сектор.
			p_buf += 512;						// 512 байт уже записали.
			cout_sector--;						// cout_sector 1 сектор записали.
		}

	}  while ( true );

	this->csHigh();

	USER_OS_GIVE_MUTEX( this->m );

	return r;
}

// Записать по адресу address массив src длинной 512 байт.
EC_SD_RESULT MicrosdSpi::writeSector ( const uint8_t* const source_array, uint32_t sector, uint32_t cout_sector, uint32_t timeout_ms  ) {
	( void )timeout_ms;

	uint32_t address;

	this->cfg->s->setPrescaler( this->cfg->fast );

	EC_SD_RESULT r = EC_SD_RESULT::ERROR;

	USER_OS_TAKE_MUTEX( this->m, portMAX_DELAY );

	uint8_t* p_buf = ( uint8_t* )source_array;

	do {
		address = this->getArgAddress( sector );	  // В зависимости от типа карты - адресация может быть побайтовая или поблочная
															// (блок - 512 байт).

		if ( this->sendCmd( CMD24, address, 0 )				 != EC_SD_RES::OK ) break;				  // Отправляем CMD24.
		uint8_t r1;
		if ( this->waitR1( &r1 )								!= EC_SD_RES::OK ) break;
		if ( r1 != 0 ) break;
		if ( this->sendWaitOnePackage()					   != EC_SD_RES::OK ) break;				  // Обязательно ждем 1 пакет.
		if ( this->sendMark( CMD24_MARK )					   != EC_SD_RES::OK ) break;

		// Пишем 512 байт.
		this->csLow();

		if ( this->cfg->s->tx( p_buf, 512, 100 )	!= BASE_RESULT::OK )   break;
		uint8_t crc_out[2] = { 0 };					  // Отправляем любой CRC.
		if ( this->cfg->s->tx( crc_out, 2, 100 )		   != BASE_RESULT::OK )   break;
		// Сразу же должен прийти ответ - принята ли команда записи.
		uint8_t answer_write_commend_in;
		if ( this->cfg->s->rx( &answer_write_commend_in, 1, 10, 0xFF ) != BASE_RESULT::OK )   break;
		if ( ( answer_write_commend_in & ( 1 << 4 ) ) != 0 ) break;
		answer_write_commend_in &= 0b1111;
		if ( answer_write_commend_in != 0b0101 )			 break; // Если не успех - выходим.
		// Ждем окончания записи.
		uint8_t write_wait = 0;
		while ( write_wait == 0 ) {
			if ( this->cfg->s->rx( &write_wait, 1, 10, 0xFF ) != BASE_RESULT::OK ) break;
		}
		if ( write_wait == 0 ) break;
		if ( this->sendWaitOnePackage() != EC_SD_RES::OK ) break;


		if ( cout_sector == 0 ) {
			r = EC_SD_RESULT::OK;
			break;
		} else {
			sector++;							// Будем писать следующий сектор.
			p_buf += 512;						// 512 байт уже записали.
			cout_sector--;						// cout_sector 1 сектор записали.
		}
	} while ( true );

	this->csHigh();

	USER_OS_GIVE_MUTEX( this->m );

	return r;
}



