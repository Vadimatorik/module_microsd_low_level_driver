#include "microsd_card.h"

#define CMD0        ( 0x40 )                                                          // Программный сброс.
#define CMD1        ( 0x40 + 1)                                                       // Инициировать процесс инициализации.
#define CMD8        ( 0x40 + 8 )                                                      // Уточнить поддерживаемое нарпряжение.
#define CMD17       ( 0x40 + 17 )                                                     // Считать блок.
#define CMD24       ( 0x40 + 24 )                                                     // Записать блок.
#define CMD55       ( 0x40 + 55 )                                                     // Указание, что далее ACMD.
#define CMD58       ( 0x40 + 58 )                                                     // Считать OCR регистр карты.

#define ACMD41      ( 0x40 + 41 )                                                     // Инициировать процесс инициализации.
#define ACMD55      ( 0x40 + 55 )                                                     // Инициировать процесс инициализации.

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
EC_RES_WAITING microsd_spi::send_empty_package ( uint16_t count ) const {
    if ( this->cfg->p_spi->tx_one_item( 0xFF, count, 10 ) == SPI::BASE_RESULT::OK ) {
        return EC_RES_WAITING::OK;
    } else {
        return EC_RES_WAITING::IO_ERROR;
    }
}

// Передача одного 0xFF. Требуется после каждой команды для ОЧЕНЬ старых карт.
EC_RES_WAITING microsd_spi::send_wait_one_package ( void ) const {
    this->cs_low();
    EC_RES_WAITING r = this->send_empty_package( 1 );
    this->cs_high();
    return r;
}

// Ждем от карты "маркер"
// - специальный байт, показывающий, что далее идет команда/данные.
EC_RES_WAITING microsd_spi::wait_mark ( uint8_t mark ) const {
    EC_RES_WAITING  r = EC_RES_WAITING::TIMEOUT;

    this->cs_low();

    for ( int l_d = 0; l_d < 5; l_d++ ) {                                               // До 5 мс даем возможность карте ответить.
        for ( int loop = 0; loop < 10; loop++ ) {
            uint8_t input_buf;

            if ( this->cfg->p_spi->rx( &input_buf, 1, 10, 0xFF ) != SPI::BASE_RESULT::OK ) {
                r = EC_RES_WAITING::IO_ERROR;
                break;
            }

            if ( input_buf == mark ) {
                r = EC_RES_WAITING::OK;
                break;
            }
        }
        if ( r != EC_RES_WAITING::TIMEOUT ) break;
        USER_OS_DELAY_MS(1);
    }

    this->cs_high();

    return r;
}

// Перевод карты врежим (CS по умолчанию 1).
EC_RES_WAITING microsd_spi::init_spi_mode ( void ) const {
    return this->send_empty_package( 10 );
}

// Пропустить n байт.
EC_RES_WAITING microsd_spi::lose_package ( uint16_t count ) const {
    this->cs_low();
    EC_RES_WAITING r = this->send_empty_package( count );
    this->cs_high();
    return r;
}

EC_RES_WAITING microsd_spi::send_cmd ( uint8_t cmd, uint32_t arg, uint8_t crc ) const {
    this->cs_low();

    uint8_t output_package[6];
    output_package[0] = cmd;
    output_package[1] = ( uint8_t )( arg >> 24 );
    output_package[2] = ( uint8_t )( arg >> 16 );
    output_package[3] = ( uint8_t )( arg >> 8 );
    output_package[4] = ( uint8_t )( arg );
    output_package[5] = crc;

    EC_RES_WAITING r = EC_RES_WAITING::OK;
    if ( this->cfg->p_spi->tx( output_package, 6, 10 ) != SPI::BASE_RESULT::OK ) {
        r = EC_RES_WAITING::IO_ERROR;
    }

    this->cs_high();

    return r;
}

// Сами отправляем маркер (нужно, например, для записи).
EC_RES_WAITING microsd_spi::send_mark ( uint8_t mark ) const {
    this->cs_low();

    EC_RES_WAITING r = EC_RES_WAITING::OK;

    if ( this->cfg->p_spi->tx( &mark, 1, 10 ) != SPI::BASE_RESULT::OK ) {
        r = EC_RES_WAITING::IO_ERROR;
    }

    this->cs_high();

    return r;
}

// Ожидаем R1 (значение R1 нам не нужно).
EC_RES_WAITING microsd_spi::wait_r1 ( uint8_t* r1 ) const {
    this->cs_low();

    EC_RES_WAITING  r = EC_RES_WAITING::TIMEOUT;

    // Карта должна принять команду в течении 10 обращений (чаще всего на 2-й итерации).
    for ( int loop = 0; loop < 10; loop++ ) {
        uint8_t input_buf;

        if ( this->cfg->p_spi->rx( &input_buf, 1, 10, 0xFF ) != SPI::BASE_RESULT::OK ) {
            r = EC_RES_WAITING::IO_ERROR;
            break;
        }

        // Сброшенный старший бит символизирует успешное принятие R1 ответа.
        if ( ( input_buf & ( 1 << 7 ) ) == 0 ) {
            if ( r1 != nullptr ) {
                *r1 = input_buf;
            }
            r = EC_RES_WAITING::OK;
            break;
        }
    }

    this->cs_high();

    return r;
}

// Ждем R3 (регистр OCR).
EC_RES_WAITING microsd_spi::wait_r3 ( uint32_t* r3 ) const {
    EC_RES_WAITING r = this->wait_r1();

    if ( r != EC_RES_WAITING::OK ) {
        return r;
    }

    this->cs_low();

    if ( this->cfg->p_spi->rx( ( uint8_t* )r3, 4, 10, 0xFF ) != SPI::BASE_RESULT::OK ) {
        r = EC_RES_WAITING::IO_ERROR;
    }

    this->cs_high();

    return r;
}

// Ждем ответа r7.
EC_RES_WAITING microsd_spi::wait_r7 ( uint32_t* r7 ) const {
    return this->wait_r3( r7 );   // Структура r3 и r7 идентичны по формату. Так экономим память.
}

// Передаем CMD55,
// Дожидаемся сообщения об успешном принятии.
// Если не удалось принять - информируем об ошибке и выходим.
EC_RES_WAITING microsd_spi::send_acmd ( uint8_t acmd, uint32_t arg, uint8_t crc ) const {
    EC_RES_WAITING r = this->send_cmd( CMD55, 0, 0 );
    if ( r != EC_RES_WAITING::OK )              return r;

    r = this->wait_r1();
    if ( r != EC_RES_WAITING::OK )              return r;

    r = this->send_empty_package( 1 );
    if ( r != EC_RES_WAITING::OK )              return r;

    r = this->send_cmd( acmd, arg, crc );
    return r;
}

// Получая сектор, возвращает адресс, который следует отправить с параметром карты.
// Иначе говоря, в зависимости от типа адресации либо возвращает тот же номер сектора,
// либо номер первого байта.
uint32_t microsd_spi::get_arg_address ( uint32_t sector ) const {
    if ( this->type_microsd != EC_MICRO_SD_TYPE::SD_V2_BLOCK ) {        // Лишь у SDHC_OR_SDXC_TYPE_SDCARD адресация по блокам. У остальных - побайтовая, кратная 512.
        return 0x200 * sector;        // Если адресация побайтовая, а нам пришел номер сектора.
    } else {
        return sector;
    }
}

//**********************************************************************
// Основной функционал.
//**********************************************************************

// Определяем тип карты и инициализируем ее.
EC_MICRO_SD_TYPE microsd_spi::initialize ( void ) const {
    if ( this->mutex != nullptr )
        USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );

    this->cfg->p_spi->set_prescaler( this->cfg->slow );

    EC_MICRO_SD_TYPE r = EC_MICRO_SD_TYPE::ERROR;

    do {
        if ( this->init_spi_mode()               != EC_RES_WAITING::OK ) break;                  // Переводим micro-sd в режим spi.
        if ( this->send_cmd( CMD0, 0, 0x95 )     != EC_RES_WAITING::OK ) break;                  // Делаем программный сброс.
        if ( this->wait_r1()                     != EC_RES_WAITING::OK ) break;
        if ( this->send_wait_one_package()       != EC_RES_WAITING::OK ) break;

        if ( this->send_cmd( CMD8, 0x1AA, 0x87 ) != EC_RES_WAITING::OK ) break;                  // Запрашиваем поддерживаемый диапазон напряжений.
        uint8_t r1;
        if ( this->wait_r1( &r1 )                != EC_RES_WAITING::OK ) break;                  // R1 в любом случае должен прийти.
        // Если CMD8 не поддерживается, значит перед нами SD версии 1 или MMC.
        if ( ( r1 & ( 1 << 2 ) ) != 0 ) {
            // SD может инициализироваться до 1 секунды.
            for ( int loop_acmd41 = 0; loop_acmd41 < 1000; loop_acmd41++ ) {                        // MMC не поддерживает ACMD41.
                if ( this->send_acmd( ACMD41, 1 << 30, 0 )   != EC_RES_WAITING::OK ) break;
                if ( this->wait_r1( &r1 )                    != EC_RES_WAITING::OK ) break;
                if ( this->send_wait_one_package()           != EC_RES_WAITING::OK ) break;

                if ( ( r1 & ( 1 << 2 ) ) != 0 ) {                                                   // Если команда не поддерживается, то перед нами MMC.
                    // SD может инициализироваться до 1 секунды.
                    for ( int loop = 0; loop < 1000; loop++ ) {                                     // Пытаемся проинициализировать MMC карту.
                        if ( this->send_cmd( CMD1, 0, 0 )        != EC_RES_WAITING::OK ) break;
                        if ( this->wait_r1( &r1 )                != EC_RES_WAITING::OK ) break;
                        if ( this->send_wait_one_package()       != EC_RES_WAITING::OK ) break;
                        if ( r1 == 0 ) {
                            this->type_microsd = EC_MICRO_SD_TYPE::MMC_V3;
                            r = EC_MICRO_SD_TYPE::MMC_V3;   // MMC была успешно проинициализирована.
                            break;
                        }
                        USER_OS_DELAY_MS( 1 );
                    }
                    break;                                  // По истечении тайм-аута - выходим с неудачей.
                }

                if ( r1 == 0 ) {
                    this->type_microsd = EC_MICRO_SD_TYPE::SD_V1;
                    r = EC_MICRO_SD_TYPE::SD_V1;
                    break;
                }
                USER_OS_DELAY_MS( 1 );
            }
        } else { // Карта V2+.
            // Нам должен прийти полный ответ от CMD8 (рас уж команда принята).
            // Но так как начало (r1) ответа r8 мы уже приняли, а оставшаяся нам не нужна, то просто пропускаем ее мимо.
            if ( this->lose_package( 4 )        != EC_RES_WAITING::OK ) break;
            if ( this->send_wait_one_package()  != EC_RES_WAITING::OK ) break;
            // SD может инициализироваться до 1 секунды.
            for ( int loop_acmd41 = 0; loop_acmd41 < 1000; loop_acmd41++ ) {
                if ( this->send_acmd( ACMD55, 0, 0 )   != EC_RES_WAITING::OK ) break;
                if ( this->wait_r1( &r1 )              != EC_RES_WAITING::OK ) break;
                if ( r1 != 1 ) break;

                if ( this->send_acmd( ACMD41, 0x40000000, 0x77 )   != EC_RES_WAITING::OK ) break;
                if ( this->wait_r1( &r1 )                    != EC_RES_WAITING::OK ) break;
                if ( this->send_wait_one_package()           != EC_RES_WAITING::OK ) break;

                if ( r1 == 0 ) break;
                USER_OS_DELAY_MS( 1 );
            }
            if ( r1 != 0 ) break; // Мы вышли по timeout-у?
            // Карта инициализирована, осталось определить, v2 это или sdhc.
            if ( this->send_cmd( CMD58, 0, 0x95 )    != EC_RES_WAITING::OK ) break;
            uint32_t ocr;
            if ( this->wait_r3( &ocr )               != EC_RES_WAITING::OK ) break;
            if ( this->send_wait_one_package()       != EC_RES_WAITING::OK ) break;

            if ( ( ocr & (1 << 30) ) == 0 ) {               // Если бит CCS в OCR сброшен, то...
                this->type_microsd = EC_MICRO_SD_TYPE::SD_V2_BYTE;
                r = EC_MICRO_SD_TYPE::SD_V2_BYTE;
            } else {
                this->type_microsd = EC_MICRO_SD_TYPE::SD_V2_BLOCK;
                r = EC_MICRO_SD_TYPE::SD_V2_BLOCK;
            }
        }
    } while ( false );

    // Теперь с SD можно работать на высоких скоростях.
    if ( r != EC_MICRO_SD_TYPE::ERROR ) {
        this->cfg->p_spi->set_prescaler( this->cfg->fast );

    }

    if ( this->mutex != nullptr )
        USER_OS_GIVE_MUTEX( this->mutex );

    return r;
}

EC_MICRO_SD_TYPE microsd_spi::get_type ( void ) const {
    return this->type_microsd;
}

EC_SD_RESULT microsd_spi::wake_up ( void ) const {
    if ( this->type_microsd == EC_MICRO_SD_TYPE::ERROR )
        return EC_SD_RESULT::NOTRDY;

    if ( this->mutex != nullptr )
        USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );

    EC_SD_RESULT r = EC_SD_RESULT::NOTRDY;
    this->cfg->p_spi->set_prescaler( this->cfg->slow );

    uint8_t r1;

    do {
        if ( ( this->type_microsd == EC_MICRO_SD_TYPE::SD_V1 ) |
             ( this->type_microsd == EC_MICRO_SD_TYPE::MMC_V3 ) ) {
            for ( int loop = 0; loop < 1000; loop++ ) {
                if ( this->send_cmd( CMD1, 0, 0 )        != EC_RES_WAITING::OK ) break;
                if ( this->wait_r1( &r1 )                != EC_RES_WAITING::OK ) break;
                if ( this->send_wait_one_package()       != EC_RES_WAITING::OK ) break;
                if ( r1 == 0 ) {
                    r = EC_SD_RESULT::OK;
                    break;
                }
                USER_OS_DELAY_MS( 1 );
            }
            break;
        }

        // Если тут, то V2+.
        for ( int loop_acmd41 = 0; loop_acmd41 < 1000; loop_acmd41++ ) {
            if ( this->send_acmd( ACMD41, 1 << 30, 0 )   != EC_RES_WAITING::OK ) break;
            if ( this->wait_r1( &r1 )                    != EC_RES_WAITING::OK ) break;
            if ( this->send_wait_one_package()           != EC_RES_WAITING::OK ) break;
            if ( r1 == 0 ) {
                r = EC_SD_RESULT::OK;
                break;
            }
            USER_OS_DELAY_MS( 1 );
        }
    } while ( false );

    if ( this->mutex != nullptr )
        USER_OS_GIVE_MUTEX( this->mutex );

    return r;
}

// Считать сектор.
// dst - указатель на массив, куда считать 512 байт.
// sector - требуемый сектор, с 0.
// Предполагается, что с картой все хорошо (она определена, инициализирована).

EC_SD_RESULT microsd_spi::read_sector ( uint32_t sector, uint8_t *target_array ) const {
    uint32_t address;

    this->cfg->p_spi->set_prescaler( this->cfg->fast );

    EC_SD_RESULT r = EC_SD_RESULT::ERROR;

    if ( this->mutex != nullptr )
        USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );

    address = this->get_arg_address( sector );                                              // В зависимости от типа карты - адресация может быть побайтовая или поблочная
                                                                                            // (блок - 512 байт).
    do {
        if ( this->send_cmd( CMD17, address, 0 )     != EC_RES_WAITING::OK ) break;          // Отправляем CMD17.
        uint8_t r1;
        if ( this->wait_r1( &r1 )                    != EC_RES_WAITING::OK ) break;
        if ( r1 != 0 ) break;
        if ( this->wait_mark( CMD17_MARK )           != EC_RES_WAITING::OK ) break;

        // Считываем 512 байт.
        this->cs_low();

        if ( this->cfg->p_spi->rx( target_array, 512, 100, 0xFF )      != SPI::BASE_RESULT::OK ) break;
        uint8_t crc_in[2] = {0xFF, 0xFF};

        if ( this->cfg->p_spi->rx( crc_in, 2, 10, 0xFF )               != SPI::BASE_RESULT::OK ) break;
        if ( this->send_wait_one_package()           != EC_RES_WAITING::OK ) break;
        r = EC_SD_RESULT::OK;
    }  while ( false );

    this->cs_high();

    if ( this->mutex != nullptr )
        USER_OS_GIVE_MUTEX( this->mutex );

    return r;
}

// Записать по адресу address массив src длинной 512 байт.
EC_SD_RESULT microsd_spi::write_sector ( uint8_t *source_array, uint32_t sector ) const {
    uint32_t address;

    this->cfg->p_spi->set_prescaler( this->cfg->fast );

    EC_SD_RESULT r = EC_SD_RESULT::ERROR;

    if ( this->mutex != nullptr )
        USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );

    address = this->get_arg_address( sector );      // В зависимости от типа карты - адресация может быть побайтовая или поблочная
                                                    // (блок - 512 байт).

    do {
        if ( this->send_cmd( CMD24, address, 0 )                 != EC_RES_WAITING::OK ) break;                  // Отправляем CMD24.
        uint8_t r1;
        if ( this->wait_r1( &r1 )                                != EC_RES_WAITING::OK ) break;
        if ( r1 != 0 ) break;
        if ( this->send_wait_one_package()                       != EC_RES_WAITING::OK ) break;                  // Обязательно ждем 1 пакет.
        if ( this->send_mark( CMD24_MARK )                       != EC_RES_WAITING::OK ) break;

        // Пишем 512 байт.
        this->cs_low();

        if ( this->cfg->p_spi->tx( source_array, 512, 100 )    != SPI::BASE_RESULT::OK )   break;
        uint8_t crc_out[2] = { 0 };                      // Отправляем любой CRC.
        if ( this->cfg->p_spi->tx( crc_out, 2, 100 )           != SPI::BASE_RESULT::OK )   break;
        // Сразу же должен прийти ответ - принята ли команда записи.
        uint8_t answer_write_commend_in;
        if ( this->cfg->p_spi->rx( &answer_write_commend_in, 1, 10, 0xFF ) != SPI::BASE_RESULT::OK )   break;
        if ( ( answer_write_commend_in & ( 1 << 4 ) ) != 0 ) break;
        answer_write_commend_in &= 0b1111;
        if ( answer_write_commend_in != 0b0101 )             break; // Если не успех - выходим.
        // Ждем окончания записи.
        uint8_t write_wait = 0;
        while ( write_wait == 0 ) {
            if ( this->cfg->p_spi->rx( &write_wait, 1, 10, 0xFF ) != SPI::BASE_RESULT::OK ) break;
        }
        if ( write_wait == 0 ) break;
        if ( this->send_wait_one_package() != EC_RES_WAITING::OK ) break;
        r = EC_SD_RESULT::OK;
    } while ( false );

    this->cs_high();

    if ( this->mutex != nullptr )
        USER_OS_GIVE_MUTEX( this->mutex );

    return r;
}


