#pragma once

#include "mk_hardware_interfaces_spi.h"
#include "mk_hardware_interfaces_pin.h"
#include "user_os.h"

enum class EC_SD_RESULT {
    OK      = 0,        // 0: Successful
    ERROR   = 1,        // 1: R/W Error
    WRPRT   = 2,        // 2: Write Protected
    NOTRDY  = 3,        // 3: Not Ready
    PARERR  = 4         // 4: Invalid Parameter
};

enum class EC_MICRO_SD_TYPE {
    ERROR           = 0,
    SD_V2_BLOCK     = 1,
    SD_V2_BYTE      = 2,
    SD_V1           = 3,
    MMC_V3          = 4
};

struct microsd_spi_cfg_t {
    const pin_base*                 const cs;             // Вывод CS, подключенный к microsd.
          spi_master_8bit_base*     const p_spi_slow;
          spi_master_8bit_base*     const p_spi_fast;
};

// Результат ожидания чего-либо.
enum class EC_RES_WAITING {
    OK          =   0,
    TIMEOUT     =   1
};

class microsd_spi {
public:
    microsd_spi ( const microsd_spi_cfg_t* const cfg );

    //**********************************************************************
    // Метод:
    // 1. Распознает тип карты.
    // 2. Инициализирует ее в соответсвии с ее типом.
    //**********************************************************************
    EC_MICRO_SD_TYPE   initialize           ( void ) const;

    //**********************************************************************
    // Метод возвращает тип карты, определенный initialize.
    //**********************************************************************
    EC_MICRO_SD_TYPE   get_type             ( void ) const;

    //**********************************************************************
    // Основываясь на ранее определенном типе (методом initialize)
    // Производит пробуждение карты.
    // Возвраащет:
    // OK - в случае удачного пробуждения.
    // NOTRDY - если не удалось.
    // ERROR - ошибка интерфейса.
    //**********************************************************************
    EC_SD_RESULT        wake_up             ( void ) const;

    // Считать сектор: структура карты, указатель на первый байт, куда будут помещены данные.
    // Адрес внутри microsd. В зависимости от карты: либо первого байта, откуда считать (выравнивание по 512 байт), либо адрес
    // сектора (каждый сектор 512 байт).
    EC_SD_RESULT        read_sector         ( uint32_t sector, uint8_t *target_array ) const;

    // Записать по адресу address массив src длинной 512 байт.
    EC_SD_RESULT        write_sector        ( uint8_t *source_array, uint32_t sector ) const;


private:
    // Переключение CS.
    void            cs_low                          ( void ) const;       // CS = 0, GND.
    void            cs_high                         ( void ) const;       // CS = 1, VDD.

    // Передать count пустых байт (шлем 0xFF).
    void            send_empty_package              ( uint16_t count ) const;

    // Передача 1 пустого байта. Требуется после каждой команды для ОЧЕНЬ старых карт.
    void            send_wait_one_package           ( void ) const;

    // Пропускаем count приших байт.
    void            lose_package                    ( uint16_t count ) const;

    // Переводим micro-sd в режим SPI.
    void            init_spi_mode                   ( void ) const;

    // Ждем от команды специального маркера.
    EC_RES_WAITING  wait_mark                       ( uint8_t mark ) const;

    // Сами отправляем маркер.
    void            send_mark                       ( uint8_t mark ) const;

    // Просто передача команды.
    void            send_cmd                        ( uint8_t cmd, uint32_t arg, uint8_t crc ) const;

    // Получаем адресс сектора (для аргумента команды чтения/записи).
    uint32_t        get_arg_address                 ( uint32_t sector ) const;

    // Отправить ACMD.
    EC_RES_WAITING  send_acmd ( uint8_t acmd, uint32_t arg, uint8_t crc ) const;


    // Ждать R1.
    EC_RES_WAITING  wait_r1               ( void ) const;
    EC_RES_WAITING  wait_r1               ( uint8_t* r1 ) const;

    // Принимаем R3 (регистр OCR).
    EC_RES_WAITING    wait_r3               ( uint32_t* r3 ) const;

    // Принимаем r7.
    EC_RES_WAITING    wait_r7               ( uint32_t* r7 ) const;

    const microsd_spi_cfg_t*    const cfg;

    mutable EC_MICRO_SD_TYPE        type_microsd = EC_MICRO_SD_TYPE::ERROR;           // Тип microSD.

    mutable USER_OS_STATIC_MUTEX_BUFFER     mutex_buf;
    mutable USER_OS_STATIC_MUTEX            mutex       = nullptr;
};
