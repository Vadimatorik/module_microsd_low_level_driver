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


microsd_spi::microsd_spi ( const microsd_spi_cfg_t* const cfg ) : cfg( cfg ) {
	this->mutex = USER_OS_STATIC_MUTEX_CREATE( &this->mutex_buf );
}

//**********************************************************************
// Низкоуровневые функции.
//**********************************************************************

// Управление линией CS.
void microsd_spi::cs_low ( void ) const {
	this->cfg->cs->reset();
}

void microsd_spi::cs_high ( void ) const {
	this->cfg->cs->set();
}

// Передать count 0xFF.
EC_SD_RES microsd_spi::send_empty_package ( uint16_t count ) const {
	if ( this->cfg->p_spi->tx_one_item( 0xFF, count, 10 ) == BASE_RESULT::OK ) {
		return EC_SD_RES::OK;
	} else {
		return EC_SD_RES::IO_ERROR;
	}
}

// Передача одного 0xFF. Требуется после каждой команды для ОЧЕНЬ старых карт.
EC_SD_RES microsd_spi::send_wait_one_package ( void ) const {
	this->cs_low();
	EC_SD_RES r = this->send_empty_package( 1 );
	this->cs_high();
	return r;
}

// Ждем от карты "маркер"
// - специальный байт, показывающий, что далее идет команда/данные.
EC_SD_RES microsd_spi::wait_mark ( uint8_t mark ) const {
	EC_SD_RES  r = EC_SD_RES::TIMEOUT;

	this->cs_low();

	for ( int l_d = 0; l_d < 3; l_d++ ) {											   // До 3 мс даем возможность карте ответить.
		for ( int loop = 0; loop < 10; loop++ ) {
			uint8_t input_buf;

			if ( this->cfg->p_spi->rx( &input_buf, 1, 10, 0xFF ) != BASE_RESULT::OK ) {
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

	this->cs_high();

	return r;
}

// Пропустить n байт.
EC_SD_RES microsd_spi::lose_package ( uint16_t count ) const {
	this->cs_low();
	EC_SD_RES r = this->send_empty_package( count );
	this->cs_high();
	return r;
}

EC_SD_RES microsd_spi::send_cmd ( uint8_t cmd, uint32_t arg, uint8_t crc ) const {
	this->cs_low();

	uint8_t output_package[6];
	output_package[0] = cmd;
	output_package[1] = ( uint8_t )( arg >> 24 );
	output_package[2] = ( uint8_t )( arg >> 16 );
	output_package[3] = ( uint8_t )( arg >> 8 );
	output_package[4] = ( uint8_t )( arg );
	output_package[5] = crc;

	EC_SD_RES r = EC_SD_RES::OK;
	if ( this->cfg->p_spi->tx( output_package, 6, 10 ) != BASE_RESULT::OK ) {
		r = EC_SD_RES::IO_ERROR;
	}

	this->cs_high();

	return r;
}

// Сами отправляем маркер (нужно, например, для записи).
EC_SD_RES microsd_spi::send_mark ( uint8_t mark ) const {
	this->cs_low();

	EC_SD_RES r = EC_SD_RES::OK;
	if ( this->cfg->p_spi->tx( &mark, 1, 10 ) != BASE_RESULT::OK ) {
		r = EC_SD_RES::IO_ERROR;
	}

	this->cs_high();

	return r;
}

// Ожидаем R1 (значение R1 нам не нужно).
EC_SD_RES microsd_spi::wait_r1 ( uint8_t* r1 ) const {
	this->cs_low();
	EC_SD_RES  r = EC_SD_RES::TIMEOUT;

	// Карта должна принять команду в течении 3 обращений (чаще всего на 2-й итерации).
	for ( int loop = 0; loop < 3; loop++ ) {
		uint8_t input_buf;

		if ( this->cfg->p_spi->rx( &input_buf, 1, 10, 0xFF ) != BASE_RESULT::OK ) {
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

	this->cs_high();
	return r;
}

#define R1_ILLEGAL_COMMAND_MSK		( 1 << 2 )

// Ждем R3 (регистр OCR).
EC_SD_RES microsd_spi::wait_r3 ( uint32_t* r3 ) const {
	uint8_t r1;
	EC_SD_RES r = this->wait_r1( &r1 );

	if ( r == EC_SD_RES::OK ) {
		if ( r1 & R1_ILLEGAL_COMMAND_MSK ) {
			return EC_SD_RES::R1_ILLEGAL_COMMAND;
		}
	} else {
		return r;
	}

	this->cs_low();

	uint8_t buf_u8[4];

	if ( this->cfg->p_spi->rx( buf_u8, 4, 10, 0xFF ) != BASE_RESULT::OK ) {
		r = EC_SD_RES::IO_ERROR;
	}

	uint32_t buf_u32 = 0;
	buf_u32 |= buf_u8[ 0 ] << 24;
	buf_u32 |= buf_u8[ 1 ] << 16;
	buf_u32 |= buf_u8[ 2 ] << 8;
	buf_u32 |= buf_u8[ 3 ];

	*r3 = buf_u32;

	this->cs_high();

	return r;
}

EC_SD_RES microsd_spi::wait_r2 ( uint16_t* r2 ) const {
	uint8_t r1;
	EC_SD_RES r = this->wait_r1( &r1 );

	if ( r == EC_SD_RES::OK ) {
		if ( r1 & R1_ILLEGAL_COMMAND_MSK ) {
			return EC_SD_RES::R1_ILLEGAL_COMMAND;
		}
	} else {
		return r;
	}

	this->cs_low();

	uint8_t buf_u8;

	if ( this->cfg->p_spi->rx( &buf_u8, 1, 10, 0xFF ) != BASE_RESULT::OK ) {
		r = EC_SD_RES::IO_ERROR;
	}

	uint16_t buf_u16 = 0;
	buf_u16 |= r1 << 8;
	buf_u16 |= buf_u8;

	*r2 = buf_u16;

	this->cs_high();

	return r;
}


// Ждем ответа r7.
EC_SD_RES microsd_spi::wait_r7 ( uint32_t* r7 ) const {
	return this->wait_r3( r7 );   // Структура r3 и r7 идентичны по формату. Так экономим память.
}

// Передаем CMD55,
// Дожидаемся сообщения об успешном принятии.
// Если не удалось принять - информируем об ошибке и выходим.
EC_SD_RES microsd_spi::send_acmd ( uint8_t acmd, uint32_t arg, uint8_t crc ) const {
	EC_SD_RES r = this->send_cmd( CMD55, 0, 0 );
	if ( r != EC_SD_RES::OK )			  return r;

	r = this->wait_r1();
	if ( r != EC_SD_RES::OK )			  return r;

	r = this->send_cmd( acmd, arg, crc );
	return r;
}

// Получая сектор, возвращает адресс, который следует отправить с параметром карты.
// Иначе говоря, в зависимости от типа адресации либо возвращает тот же номер сектора,
// либо номер первого байта.
uint32_t microsd_spi::get_arg_address ( uint32_t sector ) const {
	if ( ( this->type_microsd == EC_MICRO_SD_TYPE::SDSC ) || ( this->type_microsd == EC_MICRO_SD_TYPE::SDSD ) ) {
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
EC_MICRO_SD_TYPE microsd_spi::initialize ( void ) const {
	EC_SD_RES res_op;

	uint8_t			r1;
	uint32_t		r7;

	if ( this->mutex != nullptr )			USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );
	this->cfg->p_spi->set_prescaler( this->cfg->slow );
	this->type_microsd = EC_MICRO_SD_TYPE::ERROR;

	do {
		if ( this->send_cmd( CMD0, 0, 0x95 )		!= EC_SD_RES::OK )					break;				// Программный сброс.
		if ( this->wait_r1()						!= EC_SD_RES::OK )					break;				// Значение R1 не важно.
		if ( this->send_cmd( CMD8, 0x1AA, 0x87 )	!= EC_SD_RES::OK )					break;				// 2.7-3.6 В.
		res_op = this->wait_r7( &r7 );
		if ( ( res_op == EC_SD_RES::IO_ERROR ) || ( res_op == EC_SD_RES::TIMEOUT) )		break;
		if ( res_op == EC_SD_RES::R1_ILLEGAL_COMMAND ) {
			while(1);
		} else {
			// В ответ на CMD8 c параметром 1 ( 2.7-3.6 В ) + 0xAA (тестовое значение, выбранное на обум)
			// должно прийти эхо ( параметр 1 и 0xAA ).
			if ( r7 != 0x1AA )															break;

			int l = 500;
			for ( ; l != 0; l-- ) {
				if ( this->send_acmd( ACMD41, ACMD41_HCS_MSK, 0 ) != EC_SD_RES::OK )	break;
				if ( this->wait_r1( &r1 )			!= EC_SD_RES::OK )					break;
				if ( r1 == 0 )															break;
				USER_OS_DELAY_MS( 1 );
			}

			if ( !l ) 																	break;				// Закончились попытки инициализировать.

			if ( this->send_cmd( CMD58, 0, 0 )		!= EC_SD_RES::OK )					break;
			uint32_t ocr;
			res_op = this->wait_r3( &ocr );
			if ( res_op != EC_SD_RES::OK )												break;
			if ( ocr & OCR_CCS_MSK ) {
				this->type_microsd = EC_MICRO_SD_TYPE::SDHC_OR_SDXC;
			} else {
				this->type_microsd = EC_MICRO_SD_TYPE::SDSD;
			}

		}

	} while ( false );

	// Теперь с SD можно работать на высоких скоростях.
	if ( this->type_microsd != EC_MICRO_SD_TYPE::ERROR ) {
		this->cfg->p_spi->set_prescaler( this->cfg->fast );
	}

	if ( this->mutex != nullptr )			USER_OS_GIVE_MUTEX( this->mutex );

	return this->type_microsd;
}

EC_SD_STATUS microsd_spi::get_status ( void ) const {
	if ( this->get_type() == EC_MICRO_SD_TYPE::ERROR ) {
		this->initialize();
	}

	uint16_t		r2;

	if ( this->mutex != nullptr )			USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );
	this->cfg->p_spi->set_prescaler( this->cfg->slow );
	EC_SD_STATUS r = EC_SD_STATUS::NODISK;

	do {
		if ( this->send_cmd( CMD13, 0, 0 )	!= EC_SD_RES::OK )					break;
		EC_SD_RES res_r2 = this->wait_r2( &r2 );

		if ( res_r2	!= EC_SD_RES::OK ) {
			USER_OS_GIVE_MUTEX( this->mutex );
			this->initialize();
			continue;
		}
	} while( 0 );

	if ( r2 & ( R1_IN_IDLE_STATE_MSK << 8 ) ) {
		r = EC_SD_STATUS::NOINIT;
	} else {
		r = EC_SD_STATUS::OK;
	}

	if ( this->mutex != nullptr )			USER_OS_GIVE_MUTEX( this->mutex );

	return r;
}

EC_MICRO_SD_TYPE microsd_spi::get_type ( void ) const {
	return this->type_microsd;
}

// Считать сектор.
// dst - указатель на массив, куда считать 512 байт.
// sector - требуемый сектор, с 0.
// Предполагается, что с картой все хорошо (она определена, инициализирована).

EC_SD_RESULT microsd_spi::read_sector ( uint32_t sector, uint8_t *target_array, uint32_t cout_sector, uint32_t timeout_ms  ) const {
	( void )timeout_ms;

	uint32_t address;

	this->cfg->p_spi->set_prescaler( this->cfg->fast );

	EC_SD_RESULT r = EC_SD_RESULT::ERROR;

	if ( this->mutex != nullptr )
		USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );

	uint8_t* p_buf = target_array;

	do {
		address = this->get_arg_address( sector );											  // В зависимости от типа карты - адресация может быть побайтовая или поблочная
																									// (блок - 512 байт).

		if ( this->send_cmd( CMD17, address, 0 )	 != EC_SD_RES::OK ) break;		  // Отправляем CMD17.
		uint8_t r1;
		if ( this->wait_r1( &r1 )					!= EC_SD_RES::OK ) break;
		if ( r1 != 0 ) break;
		if ( this->wait_mark( CMD17_MARK )		   != EC_SD_RES::OK ) break;

		// Считываем 512 байт.
		this->cs_low();

		if ( this->cfg->p_spi->rx( p_buf, 512, 100, 0xFF )	  != BASE_RESULT::OK ) break;
		uint8_t crc_in[2] = {0xFF, 0xFF};

		if ( this->cfg->p_spi->rx( crc_in, 2, 10, 0xFF )			   != BASE_RESULT::OK ) break;
		if ( this->send_wait_one_package()		   != EC_SD_RES::OK ) break;

		if ( cout_sector == 0 ) {
			r = EC_SD_RESULT::OK;
			break;
		} else {
			sector++;							// Будем писать следующий сектор.
			p_buf += 512;						// 512 байт уже записали.
			cout_sector--;						// cout_sector 1 сектор записали.
		}

	}  while ( true );

	this->cs_high();

	if ( this->mutex != nullptr )
		USER_OS_GIVE_MUTEX( this->mutex );

	return r;
}

// Записать по адресу address массив src длинной 512 байт.
EC_SD_RESULT microsd_spi::write_sector ( const uint8_t* const source_array, uint32_t sector, uint32_t cout_sector, uint32_t timeout_ms  ) const {
	( void )timeout_ms;

	uint32_t address;

	this->cfg->p_spi->set_prescaler( this->cfg->fast );

	EC_SD_RESULT r = EC_SD_RESULT::ERROR;

	if ( this->mutex != nullptr )
		USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );

	uint8_t* p_buf = ( uint8_t* )source_array;

	do {
		address = this->get_arg_address( sector );	  // В зависимости от типа карты - адресация может быть побайтовая или поблочная
															// (блок - 512 байт).

		if ( this->send_cmd( CMD24, address, 0 )				 != EC_SD_RES::OK ) break;				  // Отправляем CMD24.
		uint8_t r1;
		if ( this->wait_r1( &r1 )								!= EC_SD_RES::OK ) break;
		if ( r1 != 0 ) break;
		if ( this->send_wait_one_package()					   != EC_SD_RES::OK ) break;				  // Обязательно ждем 1 пакет.
		if ( this->send_mark( CMD24_MARK )					   != EC_SD_RES::OK ) break;

		// Пишем 512 байт.
		this->cs_low();

		if ( this->cfg->p_spi->tx( p_buf, 512, 100 )	!= BASE_RESULT::OK )   break;
		uint8_t crc_out[2] = { 0 };					  // Отправляем любой CRC.
		if ( this->cfg->p_spi->tx( crc_out, 2, 100 )		   != BASE_RESULT::OK )   break;
		// Сразу же должен прийти ответ - принята ли команда записи.
		uint8_t answer_write_commend_in;
		if ( this->cfg->p_spi->rx( &answer_write_commend_in, 1, 10, 0xFF ) != BASE_RESULT::OK )   break;
		if ( ( answer_write_commend_in & ( 1 << 4 ) ) != 0 ) break;
		answer_write_commend_in &= 0b1111;
		if ( answer_write_commend_in != 0b0101 )			 break; // Если не успех - выходим.
		// Ждем окончания записи.
		uint8_t write_wait = 0;
		while ( write_wait == 0 ) {
			if ( this->cfg->p_spi->rx( &write_wait, 1, 10, 0xFF ) != BASE_RESULT::OK ) break;
		}
		if ( write_wait == 0 ) break;
		if ( this->send_wait_one_package() != EC_SD_RES::OK ) break;


		if ( cout_sector == 0 ) {
			r = EC_SD_RESULT::OK;
			break;
		} else {
			sector++;							// Будем писать следующий сектор.
			p_buf += 512;						// 512 байт уже записали.
			cout_sector--;						// cout_sector 1 сектор записали.
		}
	} while ( true );

	this->cs_high();

	if ( this->mutex != nullptr )
		USER_OS_GIVE_MUTEX( this->mutex );

	return r;
}



