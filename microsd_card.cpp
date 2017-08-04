#include "microsd_card.h"
#include <cstring>

#define CMD0        ( 0x40 )                                                          // Программный сброс.
#define CMD1        ( 0x40 + 1)                                                       // Инициировать процесс инициализации.
#define CMD8        ( 0x40 + 8 )                                                      // Уточнить поддерживаемое нарпряжение.
#define CMD17       ( 0x40 + 17 )                                                     // Считать блок.
#define CMD24       ( 0x40 + 24 )                                                     // Записать блок.
#define CMD55       ( 0x40 + 55 )                                                     // Указание, что далее ACMD.
#define CMD58       ( 0x40 + 58 )                                                     // Считать OCR регистр карты.

#define ACMD41      ( 0x40 + 41 )                                                     // Инициировать процесс инициализации.

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

// Передать count пустых байт (шлем 0xFF).
void microsd_spi::send_empty_package ( uint16_t count ) const {
    if ( this->cfg->p_spi_fast->tx_one_item( 0xFF, count, 10 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
}

// Передача 1 пустого байта. Требуется после каждой команды для ОЧЕНЬ старых карт.
void microsd_spi::send_wait_one_package ( void ) const {
    this->cs_low();
    this->send_empty_package( 1 );
    this->cs_high();
}

// Ждем от карты "маркер"
// - специальный байт, показывающий, что далее идет команда/данные.
EC_RES_WAITING microsd_spi::wait_mark ( uint8_t mark ) const {
    this->cs_low();
    uint8_t input_buf;
    for ( int loop = 0; loop < 100; loop++ ) {
        if ( this->cfg->p_spi_fast->rx( &input_buf, 1, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
            while ( true ) {}   //  На случай ошибки SPI. Потом дописать.

        if ( input_buf == mark ) {
            this->cs_high();
            return EC_RES_WAITING::OK;
        }
    }
    this->cs_high();
    return EC_RES_WAITING::TIMEOUT;
}

// Перевод карты в SPI режим.
void microsd_spi::init_spi_mode ( void ) const {
    this->cs_high();
    this->send_empty_package( 10 );
}

// Пропустить n байт.
void microsd_spi::lose_package (uint16_t count ) const {
    this->cs_low();
    this->send_empty_package( count );
    this->cs_high();
}

void microsd_spi::send_cmd ( uint8_t cmd, uint32_t arg, uint8_t crc ) const {
    this->cs_low();

    uint8_t output_package[6];
    output_package[0] = cmd;
    output_package[1] = ( uint8_t )( arg >> 24 );
    output_package[2] = ( uint8_t )( arg >> 16 );
    output_package[3] = ( uint8_t )( arg >> 8 );
    output_package[4] = ( uint8_t )( arg );
    output_package[5] = crc;

    if ( this->cfg->p_spi_fast->tx( output_package, 6, 10 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.

    this->cs_high();
}

// Сами отправляем маркер (нужно, например, для записи).
void microsd_spi::send_mark ( uint8_t mark ) const {
    this->cs_low();
    if ( this->cfg->p_spi_fast->tx( &mark, 1, 10 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
    this->cs_high();
}

// Ожидаем R1 (значение R1 нам не нужно).
EC_RES_WAITING microsd_spi::wait_r1 ( void ) const {
    this->cs_low();
    uint8_t input_buf;
    // Карта должна принять команду в течении 10 обращений (чаще всего на 2-й итерации).
    for ( int loop = 0; loop < 10; loop++ ) {
        if ( this->cfg->p_spi_fast->rx( &input_buf, 1, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
            while ( true ) {}   //  На случай ошибки SPI. Потом дописать.
        // Сброшенный старший бит символизирует успешное принятие R1 ответа.
        if ( ( input_buf & ( 1 << 7 ) ) == 0 ) {
            this->cs_high();
            return EC_RES_WAITING::OK;
        }
    }
    this->cs_high();
    return EC_RES_WAITING::TIMEOUT;
}

// Так же ждем R1, но передаем его поьзователю.
EC_RES_WAITING microsd_spi::wait_r1 ( uint8_t* r1 ) const {
    this->cs_low();
    // Карта должна принять команду в течении 10 обращений (чаще всего на 2-й итерации).
    for ( int loop = 0; loop < 10; loop++ ) {
        if ( this->cfg->p_spi_fast->rx( r1, 1, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
            while ( true ) {}   //  На случай ошибки SPI. Потом дописать.
        // Сброшенный старший бит символизирует успешное принятие R1 ответа.
        if ( ( *r1 & ( 1 << 7 ) ) == 0 ) {
            this->cs_high();
            return EC_RES_WAITING::OK;
        }
    }
    this->cs_high();
    return EC_RES_WAITING::TIMEOUT;
}

// Ждем R3 (регистр OCR).
EC_RES_WAITING microsd_spi::wait_r3 ( uint32_t* r3 ) const {
    if ( this->wait_r1() != EC_RES_WAITING::OK ) return EC_RES_WAITING::TIMEOUT;
    this->cs_low();
    if ( this->cfg->p_spi_fast->rx( ( uint8_t* )r3, 4, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}   //  На случай ошибки SPI. Потом дописать.
    this->cs_high();
    return EC_RES_WAITING::OK;
}

// Ждем ответа r7.
EC_RES_WAITING microsd_spi::wait_r7 ( uint32_t* r7 ) const {
    return this->wait_r3( r7 );   // Структура r3 и r7 идентичны по формату. Так экономим память.
}

// Передаем CMD55,
// Дожидаемся сообщения об успешном принятии.
// Если не удалось принять - информируем об ошибке и выходим.
EC_RES_WAITING microsd_spi::send_acmd ( uint8_t acmd, uint32_t arg, uint8_t crc ) const {
    this->send_cmd( CMD55, 0, 0 );
    EC_RES_WAITING result_wait = this->wait_r1();
    if ( result_wait != EC_RES_WAITING::OK ) return result_wait;
    this->send_empty_package( 1 );
    this->send_cmd( acmd, arg, crc );
    return EC_RES_WAITING::OK;
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
    this->cfg->p_spi_slow->reinit();                                                     // Переконфигурируем SPI в медленный режим ( на время инициализации ).
    this->cfg->p_spi_slow->on();

    EC_MICRO_SD_TYPE func_res = EC_MICRO_SD_TYPE::ERROR;
    do {
        this->init_spi_mode();                                                              // Переводим micro-sd в режим spi.
        this->send_cmd( CMD0, 0, 0x95 );                                                    // Делаем программный сброс.
        if ( this->wait_r1() != EC_RES_WAITING::OK ) break;
        this->send_wait_one_package();
        // Удостоверимся, что команда успешно принята.
        this->send_cmd( CMD8, 0x1AA, 0x87 );                                                // Запрашиваем поддерживаемый диапазон напряжений.
        uint8_t r1;
        if ( this->wait_r1( &r1 ) != EC_RES_WAITING::OK ) break;                            // R1 в любом случае должен прийти.
        // Если CMD8 не поддерживается, значит перед нами SD версии 1 или MMC.
        if ( ( r1 & ( 1 << 2 ) ) != 0 ) {
            // SD может инициализироваться до 1 секунды.
            for ( int loop_acmd41 = 0; loop_acmd41 < 1000; loop_acmd41++ ) {                // MMC не поддерживает ACMD41.
                if ( this->send_acmd( ACMD41, 1 << 30, 0 ) != EC_RES_WAITING::OK ) break;
                if ( this->wait_r1( &r1 ) != EC_RES_WAITING::OK ) break;
                this->send_wait_one_package();
                if ( ( r1 & (1 << 2) ) != 0 ) {                                             // Если команда не поддерживается, то перед нами MMC.
                    // SD может инициализироваться до 1 секунды.
                    for ( int loop = 0; loop < 1000; loop++ ) {                             // Пытаемся проинициализировать MMC карту.
                        this->send_cmd( CMD1, 0, 0 );
                        if ( this->wait_r1( &r1 ) != EC_RES_WAITING::OK ) break;
                        this->send_wait_one_package();
                        if ( r1 == 0 ) {
                            this->type_microsd = EC_MICRO_SD_TYPE::MMC_V3;
                            func_res = EC_MICRO_SD_TYPE::MMC_V3;   // MMC была успешно проинициализирована.
                            break;
                        }
                        USER_OS_DELAY_MS( 1 );
                    }
                }
                if ( r1 == 0 ) {
                    this->type_microsd = EC_MICRO_SD_TYPE::SD_V1;
                    func_res = EC_MICRO_SD_TYPE::SD_V1;
                    break;
                }
                USER_OS_DELAY_MS( 1 );
            }
        } else { // Карта V2+.
            // Нам должен прийти полный ответ от CMD8 (рас уж команда принята).
            // Но так как начало (r1) ответа r8 мы уже приняли, а оставшаяся нам не нужна, то просто пропускаем ее мимо.
            this->lose_package( 4 );
            this->send_wait_one_package();
            // SD может инициализироваться до 1 секунды.
            for ( int loop_acmd41 = 0; loop_acmd41 < 1000; loop_acmd41++ ) {
                if ( this->send_acmd( ACMD41, 1 << 30, 0 ) != EC_RES_WAITING::OK ) break;
                if ( this->wait_r1( &r1 ) != EC_RES_WAITING::OK ) break;
                this->send_wait_one_package();
                if ( r1 == 0 ) break;
                USER_OS_DELAY_MS( 1 );
            }
            if ( r1 != 0 ) break; // Мы вышли по timeout-у?
            // Карта инициализирована, осталось определить, v2 это или sdhc.
            this->send_cmd( CMD58, 0, 0x95 );
            uint32_t ocr;
            if ( this->wait_r3( &ocr ) != EC_RES_WAITING::OK ) break;
            this->send_wait_one_package();
            if ( ( ocr & (1 << 30) ) == 0 ) {               // Если бит CCS в OCR сброшен, то...
                this->type_microsd = EC_MICRO_SD_TYPE::SD_V2_BYTE;
                func_res = EC_MICRO_SD_TYPE::SD_V2_BYTE;
            } else {
                this->type_microsd = EC_MICRO_SD_TYPE::SD_V2_BLOCK;
                func_res = EC_MICRO_SD_TYPE::SD_V2_BLOCK;
            }
        }
    } while ( false );

    if ( func_res != EC_MICRO_SD_TYPE::ERROR ) {
        this->cfg->p_spi_fast->reinit();                                                     // Переконфигурируем SPI в медленный режим ( на время инициализации ).
        this->cfg->p_spi_fast->on();
    }

    return func_res;
}

EC_MICRO_SD_TYPE microsd_spi::get_type ( void ) const {
    return this->type_microsd;
}

EC_SD_RESULT microsd_spi::wake_up ( void ) const {
    if ( this->type_microsd == EC_MICRO_SD_TYPE::ERROR )
        return EC_SD_RESULT::NOTRDY;

    uint8_t r1;

    if ( ( this->type_microsd == EC_MICRO_SD_TYPE::SD_V1 ) |
         ( this->type_microsd == EC_MICRO_SD_TYPE::MMC_V3 ) ) {
        for ( int loop = 0; loop < 1000; loop++ ) {
            this->send_cmd( CMD1, 0, 0 );
            if ( this->wait_r1( &r1 ) != EC_RES_WAITING::OK ) return EC_SD_RESULT::ERROR;
            this->send_wait_one_package();
            if ( r1 == 0 )  return EC_SD_RESULT::OK;
            USER_OS_DELAY_MS( 1 );
        }
        return EC_SD_RESULT::NOTRDY;
    }

    // Если тут, то V2+.
    for ( int loop_acmd41 = 0; loop_acmd41 < 1000; loop_acmd41++ ) {
        if ( this->send_acmd( ACMD41, 1 << 30, 0 ) != EC_RES_WAITING::OK ) return EC_SD_RESULT::ERROR;
        if ( this->wait_r1( &r1 ) != EC_RES_WAITING::OK ) return EC_SD_RESULT::ERROR;
        this->send_wait_one_package();
        if ( r1 == 0 )  return EC_SD_RESULT::OK;
        USER_OS_DELAY_MS( 1 );
    }
    return EC_SD_RESULT::NOTRDY;
}

// Считать сектор.
// dst - указатель на массив, куда считать 512 байт.
// sector - требуемый сектор, с 0.
// Предполагается, что с картой все хорошо (она определена, инициализирована).

EC_SD_RESULT microsd_spi::read_sector ( uint32_t sector, uint8_t *target_array ) const {
    uint32_t address;
    address = this->get_arg_address( sector ); // В зависимости от типа карты - адресация может быть побайтовая или поблочная
                                               // (блок - 512 байт).
    this->send_cmd( CMD17, address, 0 );       // Отправляем CMD17.
    uint8_t r1;
    if ( this->wait_r1( &r1 ) != EC_RES_WAITING::OK )
        return EC_SD_RESULT::ERROR;
    if ( r1 != 0 ) return EC_SD_RESULT::ERROR;

    if ( this->wait_mark( CMD17_MARK ) != EC_RES_WAITING::OK )
        return EC_SD_RESULT::ERROR;

    // Считываем 512 байт.
    this->cs_low();

    if ( this->cfg->p_spi_fast->rx( target_array, 512, 100, 0xFF ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}                // На случай ошибки SPI. Потом дописать.

    uint8_t crc_in[2] = {0xFF, 0xFF};

    if ( this->cfg->p_spi_fast->rx( crc_in, 2, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.

    this->cs_high();

    this->send_wait_one_package();

    return EC_SD_RESULT::OK;
}

// Записать по адресу address массив src длинной 512 байт.
EC_SD_RESULT microsd_spi::write_sector ( uint8_t *source_array, uint32_t sector ) const {
    uint32_t address;
    address = this->get_arg_address( sector );      // В зависимости от типа карты - адресация может быть побайтовая или поблочная
                                                    // (блок - 512 байт).
    this->send_cmd( CMD24, address, 0 );            // Отправляем CMD24.
    uint8_t r1;
    if ( this->wait_r1( &r1 ) != EC_RES_WAITING::OK )
        return EC_SD_RESULT::ERROR;
    if ( r1 != 0 ) return EC_SD_RESULT::ERROR;

    this->send_wait_one_package();                  // Обязательно ждем 1 пакет.
    this->send_mark( CMD24_MARK );

    // Пишем 512 байт.
    this->cs_low();

    if ( this->cfg->p_spi_fast->tx( source_array, 512, 100 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}                           // На случай ошибки SPI. Потом дописать.

    uint8_t crc_out[2] = { 0 };                      // Отправляем любой CRC.
    if ( this->cfg->p_spi_fast->tx( crc_out, 2, 100 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}                            // На случай ошибки SPI. Потом дописать.

    // Сразу же должен прийти ответ - принята ли команда записи.
    uint8_t answer_write_commend_in;
    if ( this->cfg->p_spi_fast->rx( &answer_write_commend_in, 1, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
        return EC_SD_RESULT::ERROR;

    if ( ( answer_write_commend_in & ( 1 << 4 ) ) != 0 )
        return EC_SD_RESULT::ERROR;

    answer_write_commend_in &= 0b1111;
    if ( answer_write_commend_in != 0b0101 )    // Если не успех - выходим.
        return EC_SD_RESULT::ERROR;

    // Ждем окончания записи.
    uint8_t write_wait = 0;
    while ( write_wait == 0 ) {
        if ( this->cfg->p_spi_fast->rx( &write_wait, 1, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
            return EC_SD_RESULT::ERROR;
    }

    this->cs_high();

    this->send_wait_one_package();

    return EC_SD_RESULT::OK;
}


