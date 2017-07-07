#include "microsd_card.h"
#include <cstring>



// CRC для microSD.
uint8_t microsd_spi::crc7 ( uint8_t *d, uint32_t l ) const {
    uint8_t data;
    uint8_t crc = 0;
    for ( uint32_t cnt = 0; cnt < l; cnt++ ){
        data = d[cnt];
        for (int i=0; i<8; i++){
            crc <<= 1;
            if ((data & 0x80)^(crc & 0x80)){
                crc ^=0x09;
            };
            data <<= 1;
        }
    }
    crc=(crc<<1)|1;
    return crc;
}


// Ждем нужный байт на входе.
// Параметры: нужно передать желаемый байт на шине.
// CS упраляется отдельно.
// Если придет за time_ms миллисекунд - то мы возвращаем RES_OK. Иначе RES_ERROR.
EC_SD_RESULT microsd_spi::wait_byte_by_spi ( uint32_t number_repetitions, uint8_t state ) const {
	uint8_t buf;
    for ( uint32_t loop=0; loop < number_repetitions; loop++ ) {	// Проходимся time_ms раз (с задержкой в 1 мс после каждого неудачного).
        this->cfg->spi->rx( &buf, 1, 10, 0xFF);                     // Отправлять нужно обязательно 0xFF.
        if ( buf == state ) return EC_SD_RESULT::OK;                    // Если пришел нужный байт - выходим.
    };
    return EC_SD_RESULT::ERROR;                                     // Если за number_repetitions нужный уровень не пришел - выходим.
}

// Отправить 0xFF l раз. Для формирования нужного колличества байт "ожидания".
void microsd_spi::delay_command_out ( uint16_t  l ) const {
	uint8_t buffer[l];								// Создаем буффер, который будет отправлен по SPI.
    memset(buffer, 0xFF, l);						// Заполняем 0xFF (пустышка).
    this->cfg->spi->tx( buffer, l, 10 );
}

// Функция отправляет комманду с выбранным аргументом.
void microsd_spi::out_command ( uint8_t command, uint32_t arg ) const {
	// Заполняем массив для вычисления CRC.
    uint8_t buf[6];
    for (int i=1; i<5; i++) {
        buf[i] = ( uint8_t )( arg >> ( ( 4 - i ) * 8 ) );
	}
    buf[0] = command;
    buf[5] = this->crc7( buf, 5 );

	// Выдаем комманду.
    this->cfg->spi->tx( buf, 6, 10 );// CMD (сама комманда) + аргумент. 0x40 должно быть заранее прибавлено + CRC.
}

// Ожидание R1 ответа.
EC_SD_RESULT microsd_spi::read_r1 ( uint8_t& r1 ) const {	// Передаем FD SPI и указатель на переменную, в которую поместим R0 (если он придет).
    for (uint32_t l1=0; l1<10; l1++) {                      // Отправляем 10 0xFF. Если ничего не придет - ждем 1 мс и снова. Так 10 раз.
        r1 = 0;                                             // Отправляем 0xFF.
        this->cfg->spi->rx( &r1, 1, 10, 0xFF );
        if ((r1 & (1<<7)) == 0)                             // Если пришло не 0xFF, то наш r1 пришел!
            return EC_SD_RESULT::OK;                        // Возвращаем успешное чтение.
    }
    return EC_SD_RESULT::ERROR;                             // Если 10 попыток ничего не дали - выходим с ошибкой.
}

// Ожидание R3 ответа.
EC_SD_RESULT microsd_spi::read_r3 ( uint8_t& r1, uint32_t& r3 ) const {	// Передаем FD SPI и указатель на переменную, в которую поместим R0 (если он придет).
    if ( this->read_r1( r1 ) != EC_SD_RESULT::OK ) return EC_SD_RESULT::ERROR;               // Если R1 пришел четко.
    if ( ( r1 & (1<<2) ) == 0) {					// Если команда, после которой должен прийти R7 поддерживается картой, то читаем еще 4 байта.
        this->cfg->spi->rx( &r3, 4, 10, 0xFF);
        return EC_SD_RESULT::OK;							// Приняли 4 байта (OCR) - выходим.
    } else return EC_SD_RESULT::PARERR;					// Такой команды нет.
}

// Ожидание R7 ответа.
EC_SD_RESULT microsd_spi::read_r7 ( uint8_t& r1, uint32_t& r7 ) const {	// Передаем FD SPI и указатель на переменную, в которую поместим R1(если он придет) и R7 (после прихода R1).
    if ( this->read_r1( r1 ) != EC_SD_RESULT::OK ) return EC_SD_RESULT::ERROR;	// Если R1 не пришла - выдаем ошибку.			// Если R1 пришел четко.
        if ( ( r1 & (1<<2) ) == 0 ) {	// Если команда, после которой должен прийти R7 поддерживается картой, то читаем еще 4 байта.
             this->cfg->spi->rx( &r7, 4, 10, 0xFF );
        return EC_SD_RESULT::OK;		// Приняли 4 байта - выходим.
    } else return EC_SD_RESULT::PARERR;	// Такой команды нет.
    return EC_SD_RESULT::OK;
}

// Функция-оболочка для _read_r1, _read_r3, _read_r7.
// Нужна для того, чтобы после принятия ответа (в не зависимости от того, успешен он или нет)
// сделать задержку в 1 байт 0xFF. Т.к. некоторые карты не успевают подготовить ответ.
// Просто дополнить функция ожидания ответов (_read_r1, _read_r3, _read_r7) задержкой - нельзя.
// Т.к, например, в случае чтения r3, сначала читается r1 и затем сразу, без паузы, второй байт r3.
// Задержка между в этом случае разбила бы все.
// Параметры:
// R_number - номер ответа, который ожидаем (1, 3, 7).
// r1 - указатель, на возвращаемый r1, если придет.
// r_next - указатель на массив, в который будет помещен ответ для r3, r7.
EC_SD_RESULT microsd_spi::consider_answer( const MICRO_SD_ANSWER_TYPE type, uint8_t& r1, uint32_t& r_next ) const {
    EC_SD_RESULT res = EC_SD_RESULT::ERROR;
    switch( type ) {// Кладем результат операции, чтобы после дополнительных 8 бит - вернуть его.
        case MICRO_SD_ANSWER_TYPE::R1: res = this->read_r1( r1); break;			// После того, как мы уже использовали R_number для определения типа ответа, ее можно использовать, чтобы положить ответ обратно.
        case MICRO_SD_ANSWER_TYPE::R3: res = this->read_r3( r1, r_next ); break;
        case MICRO_SD_ANSWER_TYPE::R7: res = this->read_r7( r1, r_next ); break;
	};
    this->delay_command_out( 1 );							// Ждем 1 байт (Для старых карт. Иначе пойдут глюки).
    return res;		// Возвращаем результат.
}

// Отправляем ACMD комманду.
EC_SD_RESULT microsd_spi::out_ACMD_command ( uint8_t command, uint32_t arg ) const {
    this->out_command( 0x40 + 55, 0 );                                  // CMD55.
    uint8_t r1;
    uint32_t buf;
    if (this->consider_answer( MICRO_SD_ANSWER_TYPE::R1, r1, buf ) == EC_SD_RESULT::OK) {	// Если команда прошла.
        this->out_command( command, arg );
        return EC_SD_RESULT::OK;
	} else {
        return EC_SD_RESULT::ERROR;								// Если CMD55 не прошла, то выходим с ошибкой.
	}
}

// Перевести карту в режим SPI (перезагрузить). 100 импульсов при CS = 1.
void microsd_spi::reset ( void ) const {
    this->cfg->cs->set();								// Переводим CS в 1 (для перевода в SPI режим).
	uint8_t buffer[20];
	memset(buffer, 0xFF, sizeof(buffer));
    this->cfg->spi->tx( buffer, 20, 10 );
    this->cfg->cs->reset();									// Включаем карту.
}

// Функция отправляет CMD0 до тех пор, пока карта не ответит 0.
// CMD1 -> смотрим байт. Если не 0, то еще раз. Не ждем ответа R1!!!
// Иначе упадем в бесконечный цикл.
// Т.к. карта может в принципе не ответить (часть отвечают с первого раза и
// все хорошо, а часть не с первого и происходит зависание.).
// Возвращаемое:
// RES_OK - мы проснулись успешно.
//
EC_SD_RESULT microsd_spi::wake ( void ) const {

    for (int init_loop=0; init_loop < 10; init_loop++){		// Пытаемся разбудить карту 10 раз. Если не ответит - выходим с ошибкой.
        this->out_command( 0x41, 0 );
		for (int l = 0; l<5; l++){	// Ждем не более 5 байт и выходим (но должен, по идеи, на 2-й, считая с 1).
            uint8_t r1;
            this->cfg->spi->rx( &r1, 1, 10, 0xFF );
            if (r1 == 0) return EC_SD_RESULT::OK;		// Мы проснулись. Все четко.
		}
	}
    return EC_SD_RESULT::ERROR;			// Если за 10 попыток не проснулись - выходим.
}

// Определяем тип карты и инициализируем ее.
MICRO_SD_TYPE microsd_spi::card_type_definition_and_init ( void ) const {
	uint8_t r1;												// Сюда кладем r0 ответ.
	uint32_t r7;											// Сюда кладем r7 ответ.
	uint32_t r3;											// OCR.
    uint32_t buf;       // Затычка, когда требуется считавть r1, а нужена еще 1 неиспользуемая переменная.
    this->reset();
    //port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->init_spi_baudrate);
    this->out_command( 0x40, 0  );				// CMD0 (переводим карту в режим инициализации).
    if ( this->consider_answer( MICRO_SD_ANSWER_TYPE::R1, r1, buf ) == EC_SD_RESULT::ERROR ) {
        return MICRO_SD_TYPE::ERROR;		// Если R1 не придет - выходим с ошибкой (защита от вытаскивания карты/глюков карты).
	};
	if (!((r1 == 1) | (r1 == 0))) {
        return MICRO_SD_TYPE::ERROR;					// Если пришел ответ не 1 (в стадии инициализации) или 0 (уже готова), то выходим с ошибкой.
	}
    this->out_command( 0x48, 0x1AA );			// CMD8 (команда о переходе на питание в диапазоне 2.7-3.3 В).
    EC_SD_RESULT result = this->consider_answer( MICRO_SD_ANSWER_TYPE::R7, r1, r7 );		// Принимаем ответ от CMD8.
    if (result == EC_SD_RESULT::OK){
	// SD версии 2.х стандартной емкости(SDSC версии 2.х) или SDHC.

		for (int count_delay_ms=0; count_delay_ms<10;	count_delay_ms++){	// Команда может тупить до 10 мс. Если так и не будет нормального ответа - выйти с ошибкой. Есть карты, у которых некорректный SPI.
            if (this->out_ACMD_command( 0x41, (1<<30)) == EC_SD_RESULT::OK) {	// Если CMD55 не пройдет, выйти.
                EC_SD_RESULT result_cmd41 = this->consider_answer( MICRO_SD_ANSWER_TYPE::R1, r1, buf );	// Читаем ответ.
                if (result_cmd41 == EC_SD_RESULT::ERROR) {
                    return MICRO_SD_TYPE::ERROR;		// Если R1 не придет - выходим с ошибкой (защита от вытаскивания карты/глюков карты).
				};
				if (r1 == 0) {
					break;												// Если пришел четкий ответ - выходим.
				};
			} else {
                 return MICRO_SD_TYPE::ERROR;
			};
		};
		if (r1 != 0) {
            return MICRO_SD_TYPE::ERROR;		// Сюда приходим либо с четким ответом, либо по истечении тайм-аута.
		};
        this->out_command( 0x40 + 58, 0 ); // CMD58 чтобы получить r3 (OCR).
        if ( this->consider_answer( MICRO_SD_ANSWER_TYPE::R3, r1, r3) == EC_SD_RESULT::OK ){
			if ((r3 & (1<<30)) == 0){
                return MICRO_SD_TYPE::SD_V2_BYTE;	// SDHC или SDXC (sd_ver 2). По блокам!.
			} else {
                return MICRO_SD_TYPE::SD_V2_BLOCK;			// SDSC (sd ver 2) по байтам.
			};
		} else {
            return MICRO_SD_TYPE::ERROR;	// С картой что-то не так!
		};
	} else {
        if ( result == EC_SD_RESULT::ERROR ) {
            return MICRO_SD_TYPE::ERROR; // Если была ошибка приема R1.
		}
		// Иначе просто нет такой команды (но R1 получен успешно).
		// Карта MMC или SD версии 1.х.
        this->out_ACMD_command( 0x41, (1<<30) );	// Посылаем ACMD41. MMC ее не знает.
        if ( this->consider_answer(MICRO_SD_ANSWER_TYPE::R1, r1, buf ) == EC_SD_RESULT::ERROR) {
            return MICRO_SD_TYPE::ERROR;	// Если R1 не придет - выходим с ошибкой (защита от вытаскивания карты/глюков карты).
		};
		if ((r1 & (1<<2)) == 0){							// Если команда распознана.
            return MICRO_SD_TYPE::SD_V1;						// То это карта первого типа.
		} else {											// Иначе это MMC.
			while(r1 != 0){									// Шлем CMD1 пока не придет R1 = 0.
                this->out_command( 0x41, 0 );
                if (this->consider_answer( MICRO_SD_ANSWER_TYPE::R1, r1, buf ) == EC_SD_RESULT::ERROR) return MICRO_SD_TYPE::ERROR;// Если R1 не придет - выходим с ошибкой (защита от вытаскивания карты/глюков карты).
			};
            return MICRO_SD_TYPE::MMC_V3;
		};
	};
}

// Считать сектор: структура карты, указатель на первый байт, куда будут помещены данные.
// Адрес внутри microsd. В зависимости от карты: либо первого байта, откуда считать (выравнивание по 512 байт), либо адрес
// сектора (каждый сектор 512 байт).
FRESULT microsd_spi::read_sector ( uint8_t *dst, uint32_t address ) const {
//	if (d->cfg->spi_mutex != NULL) {	// Если мы используем mutex для предотвращения множественного доступа, то ждем его.
//		 xSemaphoreTake (*d->cfg->spi_mutex, (TickType_t) (TickType_t)100 ); // Ждем, пока освободится SPI.
//	};
    USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );

	uint8_t loop = 10;				// Колличество попыток обращения.
    while (1) {
        if (loop == 0) {			// Если колличество попыток исчерпано - выходим. Защита от зависания.
            this->cfg->cs->set();
            USER_OS_GIVE_MUTEX( this->mutex );
            return FRESULT::DISK_ERR;
		};
    //	port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->init_spi_baudrate);				// Можно жить дальше.
        this->type_microsd = this->card_type_definition_and_init();
        if (this->type_microsd == MICRO_SD_TYPE::ERROR) {				// Если карта не запустилась - выходим с ошибкой.
			loop--;
			continue;
		}
    //	port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->spi_baudrate_job);				// Можно жить дальше.
        address = this->get_address_for_sd_card( address );// В зависимости от типа карты - адресация может быть побайтовая или поблочная (блок - 512 байт).
        this->out_command( 0x40 + 17, address );	// Отправляем CMD17.
        uint8_t r1;
        uint32_t buf;
        if (this->consider_answer(MICRO_SD_ANSWER_TYPE::R1, r1, buf) != EC_SD_RESULT::OK) {
			loop--;
			continue; // Если R1 не пришел - еще раз CMD17.
		}
        if (r1 != 0) {
			loop--;
			continue;
		}
        if (this->wait_byte_by_spi( 30, 0xFE ) != EC_SD_RESULT::OK) {
			loop--;
			continue;
		}
		break;
	};

    this->cfg->spi->rx( dst, 512, 100, 0xFF );// Считываем 512 байт.
	uint8_t crc_in[2] = {0xFF, 0xFF};	// Обязательно заполнить. Иначе карта примет мусор за команду и далее все закрешется.
    this->cfg->spi->rx( crc_in, 2, 10, 0xFF );// Читаем CRC.

    this->cfg->cs->set();

    USER_OS_GIVE_MUTEX( this->mutex );
    return FRESULT::OK;
}



// Пытаемся получить от карты ответ на попытку записать блок.
MICRO_SD_ANSWER_WRITE microsd_spi::wait_answer_write ( uint32_t number_attempts ) const {
    for ( uint32_t loop=0; loop < number_attempts; loop++ ) {	// Делаем number_attempts попыток с интервалом 1 мс выловить данные.
            uint8_t buf = 0;
            this->cfg->spi->rx( &buf, 1, 10, 0xFF );
            if ((buf & (1<<4)) == 0) {					// Если пришел ответ.
                buf &= 0b1111;							// Накладываем маску ответов.
                if ( buf == (uint8_t)MICRO_SD_ANSWER_WRITE::DATA_IN_OK )           return MICRO_SD_ANSWER_WRITE::DATA_IN_OK;
                if ( buf == (uint8_t)MICRO_SD_ANSWER_WRITE::DATA_IN_CRC_ERROR )    return MICRO_SD_ANSWER_WRITE::DATA_IN_CRC_ERROR;
                if ( buf == (uint8_t)MICRO_SD_ANSWER_WRITE::DATA_IN_WRITE_ERROR )  return MICRO_SD_ANSWER_WRITE::DATA_IN_WRITE_ERROR;
                return MICRO_SD_ANSWER_WRITE::DATA_IN_ANSWER_ERROR;		// Если ответ не вписывается в стандарт - ошибка.
            }
    };
    return MICRO_SD_ANSWER_WRITE::DATA_IN_ANSWER_ERROR;	// Такой байт не пришел. Выходим с ошибкой.
}


// Записать по адресу address массив src длинной 512 байт.
FRESULT microsd_spi::write_sector ( uint32_t address, uint8_t *src ) const {
    FRESULT result_write_sector;
	uint8_t buf = 0xFF;											// Сюда кладем r0 ответ.
    USER_OS_TAKE_MUTEX( this->mutex, 100 );

	uint8_t loop = 10;				// Колличество попыток обращения.
	while (1){
		if (loop == 0){			// Если колличество попыток исчерпано - выходим. Защита от зависания.
            this->cfg->cs->set();
            USER_OS_GIVE_MUTEX( this->mutex );
            return FRESULT::DISK_ERR;
		};
        //port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->init_spi_baudrate);				// Можно жить дальше.
        this->type_microsd = this->card_type_definition_and_init();
        if (this->type_microsd == MICRO_SD_TYPE::ERROR) {				// Если карта не запустилась - выходим с ошибкой.
            loop--;
            continue;
        }
        //port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->spi_baudrate_job);				// Можно жить дальше.
        address = this->get_address_for_sd_card( address );// В зависимости от типа карты - адресация может быть побайтовая или поблочная (блок - 512 байт).
        this->out_command( 0x40 + 24, address );	// Шлем CMD24 с адресом сектора/байта, с которого будем читать (в зависимости от карты, должно быть заранее продумано по типу карты) в который будем писать.
        this->delay_command_out( 1 );		// Перед отправкой флага рекомендуется подождать >= 1 байт. Для надежности - ждем 10.
        buf = 0xFE;
        this->cfg->spi->tx( &buf, 1, 10 );// Шлем флаг, что далее идут данные.
        this->cfg->spi->tx( src, 512, 100 );// Выкидываем данные.
		uint8_t crc_out[2] = {0xFF, 0xFF};						// CRC пока что шлем левое (нужно реализовать CRC по идеи, чтобы потом включить режим с его поддержкой и увеличить вероятность успешной передачи).
        this->cfg->spi->tx( crc_out, 2, 10 );// Передаем CRC.
        if ( this->wait_answer_write( 100 ) != MICRO_SD_ANSWER_WRITE::DATA_IN_OK ) {	// Если ошибка.
			loop--;
			continue;
		};
		// Линия должна перейти в 0x00, потом в 0xff.
        if ((this->wait_byte_by_spi( 3, 0) != EC_SD_RESULT::OK ) && (this->wait_byte_by_spi( 3, 0xFF) != EC_SD_RESULT::OK ) ){
			loop--;
			continue;
		};
        result_write_sector =  FRESULT::OK;			// Все четко, выходим.
		break;
	};

    this->cfg->cs->set();
    USER_OS_GIVE_MUTEX( this->mutex );
	return result_write_sector;
}

// Показываем, инициализирована ли карта. Используем enum из fatfs.
STA microsd_spi::microsd_card_get_card_info ( void ) const {
    if ( this->type_microsd != MICRO_SD_TYPE::ERROR ) { // Если карты была инициализирована.
        return STA::OK;
	} else {
        return STA::NOINIT;
	}
}

// Получая сектор, возвращает адресс, который следует отправить с параметром карты.
// Иначе говоря, в зависимости от типа адресации либо возвращает тот же номер сектора,
// либо номер первого байта.
uint32_t microsd_spi::get_address_for_sd_card ( uint32_t sector ) const {
    if ( this->type_microsd != MICRO_SD_TYPE::SD_V2_BLOCK ) {		// Лишь у SDHC_OR_SDXC_TYPE_SDCARD адресация по блокам. У остальных - побайтовая, кратная 512.
		return 0x200 * sector;		// Если адресация побайтовая, а нам пришел номер сектора.
	} else {
		return sector;
	}
}

// Получаем регистр с полным описанием флешки.
EC_SD_RESULT microsd_spi::get_CSD ( uint8_t *src ) const {
    this->cfg->cs->reset();		// Подключаемся к карте.
    if (this->wake() == EC_SD_RESULT::OK){	// Если карта успешно открыта.
        this->out_command( 0x40 + 9, 0 );	// Требуем CSD.
        uint8_t r1 = 0xff;
        uint32_t buf;
        if (this->consider_answer( MICRO_SD_ANSWER_TYPE::R1, r1, buf ) == EC_SD_RESULT::OK){				// Если R1 пришел.
			if (r1 == 0){											// При этом он "чистый" (без ошибок).
                this->cfg->spi->rx( src, 16, 0xFF );// Считываем CSD.
                this->cfg->cs->set();
                return EC_SD_RESULT::OK;
			}
		}
	};
    this->cfg->cs->set();	// Во всех остальных случаях выходим с ошибкой.
    return EC_SD_RESULT::ERROR;
}
