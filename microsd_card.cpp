#include "microsd_card.h"
#include <cstring>

microsd_spi::microsd_spi ( const microsd_spi_cfg_t* const cfg ) : cfg( cfg ) {
    this->mutex = USER_OS_STATIC_MUTEX_CREATE( &this->mutex_buf );
}

// Определяем тип карты и инициализируем ее.
MICRO_SD_TYPE microsd_spi::initialize ( void ) const {
    return MICRO_SD_TYPE::ERROR;
}

// Считать сектор: структура карты, указатель на первый байт, куда будут помещены данные.
// Адрес внутри microsd. В зависимости от карты: либо первого байта, откуда считать (выравнивание по 512 байт), либо адрес
// сектора (каждый сектор 512 байт).
EC_FRESULT microsd_spi::read_sector ( uint8_t *dst, uint32_t address ) const {
    (void)dst; (void)address;
    return EC_FRESULT::OK;
//    if (d->cfg->spi_mutex != NULL) {    // Если мы используем mutex для предотвращения множественного доступа, то ждем его.
//         xSemaphoreTake (*d->cfg->spi_mutex, (TickType_t) (TickType_t)100 ); // Ждем, пока освободится SPI.
//    };
    /*
    USER_OS_TAKE_MUTEX( this->mutex, portMAX_DELAY );

    uint8_t loop = 10;                // Колличество попыток обращения.
    while (1) {
        if (loop == 0) {            // Если колличество попыток исчерпано - выходим. Защита от зависания.
            this->cfg->cs->set();
            USER_OS_GIVE_MUTEX( this->mutex );
            return EC_FRESULT::DISK_ERR;
        };
    //    port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->init_spi_baudrate);                // Можно жить дальше.
        this->type_microsd = this->card_type_definition_and_init();
        if (this->type_microsd == MICRO_SD_TYPE::ERROR) {                // Если карта не запустилась - выходим с ошибкой.
            loop--;
            continue;
        }
    //    port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->spi_baudrate_job);                // Можно жить дальше.
        address = this->get_address_for_sd_card( address );// В зависимости от типа карты - адресация может быть побайтовая или поблочная (блок - 512 байт).
        this->out_command( 0x40 + 17, address );    // Отправляем CMD17.
        uint8_t r1;
        if (this->consider_answer(MICRO_SD_ANSWER_TYPE::R1, &r1, nullptr) != EC_SD_RESULT::OK) {
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

    // Считываем 512 байт.
    if ( this->cfg->p_spi->rx( dst, 512, 100, 0xFF ) != EC_SPI_BASE_RESULT::OK ) {
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
    }
    uint8_t crc_in[2] = {0xFF, 0xFF};    // Обязательно заполнить. Иначе карта примет мусор за команду и далее все закрешется.

    // Читаем CRC.
    if ( this->cfg->p_spi->rx( crc_in, 2, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK ) {
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
    }

    this->cfg->cs->set();

    USER_OS_GIVE_MUTEX( this->mutex );
    return EC_FRESULT::OK;*/
}



// Пытаемся получить от карты ответ на попытку записать блок.
MICRO_SD_ANSWER_WRITE microsd_spi::wait_answer_write ( uint32_t number_attempts ) const {

    (void)number_attempts;
    return MICRO_SD_ANSWER_WRITE::DATA_IN_OK;
    /*
    for ( uint32_t loop=0; loop < number_attempts; loop++ ) {    // Делаем number_attempts попыток с интервалом 1 мс выловить данные.
            uint8_t buf = 0;
                this->cfg->p_spi ->rx( &buf, 1, 10, 0xFF );
            if ((buf & (1<<4)) == 0) {                    // Если пришел ответ.
                buf &= 0b1111;                            // Накладываем маску ответов.
                if ( buf == (uint8_t)MICRO_SD_ANSWER_WRITE::DATA_IN_OK )           return MICRO_SD_ANSWER_WRITE::DATA_IN_OK;
                if ( buf == (uint8_t)MICRO_SD_ANSWER_WRITE::DATA_IN_CRC_ERROR )    return MICRO_SD_ANSWER_WRITE::DATA_IN_CRC_ERROR;
                if ( buf == (uint8_t)MICRO_SD_ANSWER_WRITE::DATA_IN_WRITE_ERROR )  return MICRO_SD_ANSWER_WRITE::DATA_IN_WRITE_ERROR;
                return MICRO_SD_ANSWER_WRITE::DATA_IN_ANSWER_ERROR;        // Если ответ не вписывается в стандарт - ошибка.
            }
    };
    return MICRO_SD_ANSWER_WRITE::DATA_IN_ANSWER_ERROR;    // Такой байт не пришел. Выходим с ошибкой.*/
}


// Записать по адресу address массив src длинной 512 байт.
EC_FRESULT microsd_spi::write_sector ( uint32_t address, uint8_t *src ) const {
    (void)address; (void)src;
    return EC_FRESULT::OK;
    /*
    EC_FRESULT result_write_sector;
    uint8_t buf = 0xFF;                                            // Сюда кладем r0 ответ.
    USER_OS_TAKE_MUTEX( this->mutex, 100 );

    uint8_t loop = 10;                // Колличество попыток обращения.
    while (1){
        if (loop == 0){            // Если колличество попыток исчерпано - выходим. Защита от зависания.
            this->cfg->cs->set();
            USER_OS_GIVE_MUTEX( this->mutex );
            return EC_FRESULT::DISK_ERR;
        };
        //port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->init_spi_baudrate);                // Можно жить дальше.
        this->type_microsd = this->card_type_definition_and_init();
        if (this->type_microsd == MICRO_SD_TYPE::ERROR) {                // Если карта не запустилась - выходим с ошибкой.
            loop--;
            continue;
        }
        //port_spi_baudrate_update(*d->cfg->spi_fd, d->cfg->spi_baudrate_job);                // Можно жить дальше.
        address = this->get_address_for_sd_card( address );// В зависимости от типа карты - адресация может быть побайтовая или поблочная (блок - 512 байт).
        this->out_command( 0x40 + 24, address );    // Шлем CMD24 с адресом сектора/байта, с которого будем читать (в зависимости от карты, должно быть заранее продумано по типу карты) в который будем писать.
        this->delay_command_out( 1 );        // Перед отправкой флага рекомендуется подождать >= 1 байт. Для надежности - ждем 10.
        buf = 0xFE;

        // Шлем флаг, что далее идут данные.
        if ( this->cfg->p_spi ->tx( &buf, 1, 10 ) != EC_SPI_BASE_RESULT::OK ) {
            while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
        }
        // Выкидываем данные.
        if ( this->cfg->p_spi ->tx( src, 512, 100 ) != EC_SPI_BASE_RESULT::OK ) {
            while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
        }

        // CRC пока что шлем левое (нужно реализовать CRC по идеи, чтобы потом включить режим с его поддержкой и увеличить вероятность успешной передачи).
        uint8_t crc_out[2] = {0xFF, 0xFF};
        // Передаем CRC.
        if ( this->cfg->p_spi->tx( crc_out, 2, 10 ) != EC_SPI_BASE_RESULT::OK ) {
            while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
        }
        if ( this->wait_answer_write( 100 ) != MICRO_SD_ANSWER_WRITE::DATA_IN_OK ) {    // Если ошибка.
            loop--;
            continue;
        };
        // Линия должна перейти в 0x00, потом в 0xff.
        if ((this->wait_byte_by_spi( 3, 0) != EC_SD_RESULT::OK ) && (this->wait_byte_by_spi( 3, 0xFF) != EC_SD_RESULT::OK ) ){
            loop--;
            continue;
        };
        result_write_sector =  EC_FRESULT::OK;            // Все четко, выходим.
        break;
    };

    this->cfg->cs->set();
    USER_OS_GIVE_MUTEX( this->mutex );
    return result_write_sector;*/
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
    if ( this->type_microsd != MICRO_SD_TYPE::SD_V2_BLOCK ) {        // Лишь у SDHC_OR_SDXC_TYPE_SDCARD адресация по блокам. У остальных - побайтовая, кратная 512.
        return 0x200 * sector;        // Если адресация побайтовая, а нам пришел номер сектора.
    } else {
        return sector;
    }
}

// Получаем регистр с полным описанием флешки.
EC_SD_RESULT microsd_spi::get_CSD ( uint8_t *src ) const {
    (void)src;/*
    this->cfg->cs->reset();        // Подключаемся к карте.
    if (this->wake() == EC_SD_RESULT::OK){    // Если карта успешно открыта.
        this->out_command( 0x40 + 9, 0 );    // Требуем CSD.
        uint8_t r1 = 0xff;
        if (this->consider_answer( MICRO_SD_ANSWER_TYPE::R1, &r1, nullptr ) == EC_SD_RESULT::OK){                // Если R1 пришел.
            if (r1 == 0){                                            // При этом он "чистый" (без ошибок).
                // Считываем CSD.
                if ( this->cfg->p_spi ->rx( src, 16, 0xFF ) != EC_SPI_BASE_RESULT::OK ) {
                    while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
                }
                this->cfg->cs->set();
                return EC_SD_RESULT::OK;
            }
        }
    };
    this->cfg->cs->set();    // Во всех остальных случаях выходим с ошибкой.*/
    return EC_SD_RESULT::ERROR;
}
