#include "microsd_card.h"
#include <cstring>

/* MMC/SD command */
#define CMD0	(0)			/* GO_IDLE_STATE */
#define CMD1	(1)			/* SEND_OP_COND (MMC) */
#define	ACMD41	(0x80+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(8)			/* SEND_IF_COND */
#define CMD9	(9)			/* SEND_CSD */
#define CMD10	(10)		/* SEND_CID */
#define CMD12	(12)		/* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	/* SD_STATUS (SDC) */
#define CMD16	(16)		/* SET_BLOCKLEN */
#define CMD17	(17)		/* READ_SINGLE_BLOCK */
#define CMD18	(18)		/* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		/* SET_BLOCK_COUNT (MMC) */
#define	ACMD23	(0x80+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		/* WRITE_BLOCK */
#define CMD25	(25)		/* WRITE_MULTIPLE_BLOCK */
#define CMD32	(32)		/* ERASE_ER_BLK_START */
#define CMD33	(33)		/* ERASE_ER_BLK_END */
#define CMD38	(38)		/* ERASE */
#define CMD55	(55)		/* APP_CMD */
#define CMD58	(58)		/* READ_OCR */

void  microsd_spi::reinit ( void ) const {
    this->mutex     = USER_OS_STATIC_MUTEX_CREATE( &this->mutex_buf );
}

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

/* 1:Ready, 0:Timeout */
int microsd_spi::wait_ready ( uint32_t delay_ms )const {
    uint8_t d = 0;
    do {
        if ( this->cfg->p_spi->rx( &d, 1, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
            while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
        delay_ms--;
        if ( d == 0xFF ) return 1;
        vTaskDelay(1);
    } while ( delay_ms );	/* Wait for card goes ready or timeout */

    return 0;
}


// Перевести карту в режим SPI (перезагрузить). 100 импульсов при CS = 1.
void microsd_spi::reset ( void ) const {
    if ( this->cfg->p_spi->tx_one_item( 0xFF, 10, 10 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
}

void microsd_spi::deselect (void) const{
    this->cfg->cs->set();
    if ( this->cfg->p_spi->tx_one_item( 0xFF, 1, 10 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
}

/* 1:OK, 0:Timeout */
int microsd_spi::select (void)	const{
    this->cfg->cs->reset();
    if ( this->cfg->p_spi->tx_one_item( 0xFF, 1, 10 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
    if (wait_ready(500)) return 1;	/* Wait for card ready */

    deselect();
    return 0;	/* Timeout */
}

uint8_t microsd_spi::send_cmd (	uint8_t cmd, uint32_t arg ) const {
    uint8_t n, res;

    if ( cmd & 0x80 ) {                     // Если передается ACMD, то сначала нужно CMD55.
        cmd &= 0x7F;
        res = this->send_cmd( CMD55, 0 );
        if (res > 1) return res;
    }

    /* Select the card and wait for ready except to stop multiple block read */
    if (cmd != CMD12) {
        this->deselect();
        if (!select()) return 0xFF;
    }

    /* Send command packet */
    uint8_t data[6];
    data[0] = (0x40 | cmd);				/* Start + command index */
    data[1] = arg >> 24;		/* Argument[31..24] */
    data[2] = arg >> 16;		/* Argument[23..16] */
    data[3] = arg >> 8;			/* Argument[15..8] */
    data[4] = arg;				/* Argument[7..0] */
    data[5] = 0x01;							/* Dummy CRC + Stop */
    if (cmd == CMD0) data[5] = 0x95;			/* Valid CRC for CMD0(0) */
    if (cmd == CMD8) data[5] = 0x87;			/* Valid CRC for CMD8(0x1AA) */
    if ( this->cfg->p_spi->tx( data, 6, 10 ) != EC_SPI_BASE_RESULT::OK )
        while ( true ) {};  //  На случай ошибки SPI. Потом дописать.

    /* Receive command resp */
    if (cmd == CMD12) {	/* Diacard following one byte when CMD12 */
        if ( this->cfg->p_spi->tx_one_item( 0xFF, 1, 10 ) != EC_SPI_BASE_RESULT::OK )
            while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
    }
    n = 10;								/* Wait for response (10 bytes max) */
    do {
        if ( this->cfg->p_spi->rx( &res, 1, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )
            while ( true ) {};

    } while ((res & 0x80) && --n);

    return res;							/* Return received response */
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
    return EC_SD_RESULT::ERROR;
/*
    for (int init_loop=0; init_loop < 10; init_loop++){        // Пытаемся разбудить карту 10 раз. Если не ответит - выходим с ошибкой.
        this->out_command( 0x41, 0 );
        for (int l = 0; l<5; l++){    // Ждем не более 5 байт и выходим (но должен, по идеи, на 2-й, считая с 1).
            uint8_t r1;
            if ( this->cfg->p_spi ->rx( &r1, 1, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK ) {
                while ( true ) {}  //  На случай ошибки SPI. Потом дописать.
            }
            if (r1 == 0) return EC_SD_RESULT::OK;        // Мы проснулись. Все четко.
        }
    }
    return EC_SD_RESULT::ERROR;            // Если за 10 попыток не проснулись - выходим.*/

}

// Определяем тип карты и инициализируем ее.
MICRO_SD_TYPE microsd_spi::initialize ( void ) const {
    uint8_t cmd, ocr[4];

    //  if (Stat & STA_NODISK) return Stat;	/* Is card existing in the soket? */

    //FCLK_SLOW();
    this->reset();

    MICRO_SD_TYPE ty = MICRO_SD_TYPE::ERROR;
    if ( this->send_cmd( CMD0, 0 ) == 1 ) {			/* Put the card SPI/Idle state */
        int Timer1 = 1000;						/* Initialization timeout = 1 sec */
        if ( send_cmd( CMD8, 0x1AA ) == 1 ) {	/* SDv2? */
            if ( this->cfg->p_spi->rx( ocr, 4, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )/* Get 32 bit return value of R7 resp */
                while ( true ) {};
        if ( ocr[2] == 0x01 && ocr[3] == 0xAA ) {				/* Is the card supports vcc of 2.7-3.6V? */
                    while (Timer1 && send_cmd(ACMD41, 1 << 30)) {/* Wait for end of initialization with ACMD41(HCS) */
                        Timer1--;
                        vTaskDelay(1);
                    };

                    if (Timer1 && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
                        if ( this->cfg->p_spi->rx( ocr, 4, 10, 0xFF ) != EC_SPI_BASE_RESULT::OK )/* Get 32 bit return value of R7 resp */
                            while ( true ) {};

                        ty = (ocr[0] & 0x40) ? MICRO_SD_TYPE::SD_V2_BLOCK  : MICRO_SD_TYPE::SD_V2_BYTE;	/* Card id SDv2 */
                        Timer1--;
                        vTaskDelay(1);
                    }
                }
            } else {	/* Not SDv2 card */
                if (this->send_cmd( ACMD41, 0 ) <=  1) 	{	/* SDv1 or MMC? */
                    ty = MICRO_SD_TYPE::SD_V1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
                } else {
                    ty = MICRO_SD_TYPE::MMC_V3; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
                }
                /* Wait for end of initialization */
                while (Timer1 && send_cmd(cmd, 0)) {
                    Timer1--;
                    vTaskDelay(1);
                }

                if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set block length: 512 */
                    ty = MICRO_SD_TYPE::ERROR;
            }
        }
        //CardType = ty;	// Card type
        this->deselect();
/*
        if (ty) {			// OK
            FCLK_FAST();			// Set fast clock
            Stat &= ~STA_NOINIT;	// Clear STA_NOINIT flag
        } else {			// Failed
            Stat = STA_NOINIT;
        }

        return Stat;*/
        return ty;
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
