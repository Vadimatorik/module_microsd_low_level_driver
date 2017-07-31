#pragma once

#include "user_hardware_interface.h"

/*
#include "crc.h"                        // Для использования CRC7.
#include "eflib_config.h"                // FreeRTOS.
#include "eflib_config.h"
#include "port_spi.h"                    // Работа с SPI.
#include "errno.h"                        // Коды возвращаемых ошибок.
#include "port_gpio.h"                    // Для дергания CS.
#include "diskio.h"                        // Используем enum отсюда. Для повышения совместимости.
#include "port_uart.h"
#include <string.h>
#include "ff.h"
*/

enum class EC_SD_RESULT {
    OK      = 0,        // 0: Successful
    ERROR   = 1,        // 1: R/W Error
    WRPRT   = 2,        // 2: Write Protected
    NOTRDY  = 3,        // 3: Not Ready
    PARERR  = 4         // 4: Invalid Parameter
};

enum class MICRO_SD_TYPE {
    ERROR           = 0,
    SD_V2_BLOCK     = 1,
    SD_V2_BYTE      = 2,
    SD_V1           = 3,
    MMC_V3          = 4
};

// Ответы, которые приходят от microsd после приема порции в 512 байт.
enum class MICRO_SD_ANSWER_WRITE {
    DATA_IN_OK              = 0b101,            // Данные приняты.
    DATA_IN_CRC_ERROR       = 0b1011,           // CRC ошибка. Данные не приняты.
    DATA_IN_WRITE_ERROR     = 0b1101,           // Ошибка записи.
    DATA_IN_ANSWER_ERROR    = 0                 // Ответ просто не пришел.
};


// Тип возвращаемого значения полностью совместим с FATFS от chan-а FRESULT классом!
enum class FRESULT{
    OK = 0,                 /* (0) Succeeded */
    DISK_ERR,               /* (1) A hard error occurred in the low level disk I/O layer */
    INT_ERR,                /* (2) Assertion failed */
    NOT_READY,              /* (3) The physical drive cannot work */
    NO_FILE,                /* (4) Could not find the file */
    NO_PATH,                /* (5) Could not find the path */
    INVALID_NAME,           /* (6) The path name format is invalid */
    DENIED,                 /* (7) Access denied due to prohibited access or directory full */
    EXIST,                  /* (8) Access denied due to prohibited access */
    INVALID_OBJECT,         /* (9) The file/directory object is invalid */
    WRITE_PROTECTED,        /* (10) The physical drive is write protected */
    INVALID_DRIVE,          /* (11) The logical drive number is invalid */
    NOT_ENABLED,            /* (12) The volume has no work area */
    NO_FILESYSTEM,          /* (13) There is no valid FAT volume */
    MKFS_ABORTED,           /* (14) The f_mkfs() aborted due to any parameter error */
    TIMEOUT,                /* (15) Could not get a grant to access the volume within defined period */
    LOCKED,                 /* (16) The operation is rejected according to the file sharing policy */
    NOT_ENOUGH_CORE,        /* (17) LFN working buffer could not be allocated */
    TOO_MANY_OPEN_FILES,    /* (18) Number of open files > _FS_LOCK */
    INVALID_PARAMETER       /* (19) Given parameter is invalid */
} ;

enum class STA {
    OK          = 0,
    NOINIT		= 0x01,	/* Drive not initialized */
    NODISK		= 0x02,	/* No medium in the drive */
    PROTECT		= 0x04	/* Write protected */
};

// Формат получаемого от micro-sd ответа.
enum class MICRO_SD_ANSWER_TYPE {
    R1 = 1,
    R3 = 3,
    R7 = 7
};


struct microsd_spi_cfg_t {
    const pin*                      const cs;             // Вывод CS, подключенный к microsd.
          uint32_t                  init_spi_baudrate;    // Скорость во время инициализации.
          uint32_t                  spi_baudrate_job;     // Скорость во время работы.
          spi_master_8bit_base*     const p_spi;
    /*
#ifdef MICRO_SD_CARD_UART_DEBUG_LOG_OUT
    int                *uart_fd;            // Если включен режим вывода log-а по UART.
#endif*/
};

//#define DEBUG_OUT(uart_fd, string);            port_uart_tx(uart_fd, string, sizeof(string), portMAX_DELAY);
//#define DEBUG_OUT(uart_fd, string);

class microsd_spi {
public:
    constexpr microsd_spi( const microsd_spi_cfg_t* const cfg ) : cfg(cfg) {}
    void    reinit          ( void ) const;

    // Считать сектор: структура карты, указатель на первый байт, куда будут помещены данные.
    // Адрес внутри microsd. В зависимости от карты: либо первого байта, откуда считать (выравнивание по 512 байт), либо адрес
    // сектора (каждый сектор 512 байт).
    FRESULT write_sector ( uint32_t address, uint8_t *src ) const;

    // Записать по адресу address массив src длинной 512 байт.
    FRESULT read_sector ( uint8_t *dst, uint32_t address ) const;
    // По номеру сектора и типу SD решаем, какого типа адресация и возвращаем  адрес, который нужно передать в microsd.
    uint32_t get_address_for_sd_card ( uint32_t sector ) const;

    STA             microsd_card_get_card_info ( void ) const;
    EC_SD_RESULT    get_CSD ( uint8_t *src ) const;

private:
    EC_SD_RESULT    wait_byte_by_spi        ( uint32_t number_repetitions, uint8_t state ) const;
    void            delay_command_out       ( uint16_t  l ) const;
    void            out_command ( uint8_t command, uint32_t arg ) const;
    EC_SD_RESULT    read_r1 ( uint8_t& r1 ) const ;
    EC_SD_RESULT    read_r3 ( uint8_t& r1, uint32_t& r3 ) const;
    EC_SD_RESULT    read_r7 ( uint8_t& r1, uint32_t& r7 ) const;
    uint8_t         crc7 ( uint8_t *d, uint32_t l ) const;
    EC_SD_RESULT    out_ACMD_command ( uint8_t command, uint32_t arg ) const;
    EC_SD_RESULT    consider_answer( const MICRO_SD_ANSWER_TYPE type, uint8_t& r1, uint32_t& r_next ) const;
    void            reset ( void ) const;
    EC_SD_RESULT    wake ( void ) const;
    MICRO_SD_ANSWER_WRITE wait_answer_write ( uint32_t number_attempts ) const;
    MICRO_SD_TYPE   card_type_definition_and_init ( void ) const;

    const microsd_spi_cfg_t* const cfg;
    mutable MICRO_SD_TYPE             type_microsd = MICRO_SD_TYPE::ERROR;           // Тип microSD.

    mutable USER_OS_STATIC_MUTEX_BUFFER     mutex_buf = USER_OS_STATIC_MUTEX_BUFFER_INIT_VALUE;
    mutable USER_OS_STATIC_MUTEX            mutex = NULL;
};
