﻿# Драйвер и утилита тестирования PCI устройств для Linux

## Драйвер (модуль ядра)

Драйвер регистрирует PCI устройства с `VENDOR_ID:DEVICE_ID`, которые указаны в заголовочном файле `hw_drv.h` в макросе `HW_DRV_PCI_DEVICES_IDS`.

Для каждого PCI устройства:

1. Регистрируется обработчик прерываний,  который просто выводит сообщение в dmesg. По умолчанию, происходит попытка регистрации MSI прерываний. Если указан параметр `no_msi=1`, то будет попытка регистрации Legacy прерываний. MSI прерывания не будут регистрироваться.

2. Выделяется физически непрерывная область памяти размером 2 Мбайт (по умолчанию). Размер области памяти задается параметром `mem_size=<размер>`. Сам размер указывается в килобайтах, т.е. 4 Мбайтам соответствует значение 4096.

   По умолчанию, эта область памяти может быть выделена в 64-х битном адресном пространстве (DMA маска устанавливается в 64 бита). Для выделения области памяти в 32-х битном адресном пространстве необходимо передать модулю парметр `dma_32=1`.

Вне зависимости от наличия PCI устройств на шине, драйвер выделяет одну область физически непрерываной области памяти.

Из утилиты данная область доступна по идентификатору устройства 0.

Для того чтобы собрать драйвер совсем без поддержки PCI устройств, в файле `hw_drv.h` необходимо установить макрос `HW_DRV_USE_PCI_DEVICES` в значение 0.

## Утилита

Запуск:
```
./hw_test <options>
```

Опции:

* `--dev (-d)`

  Идентификатор устройства (0 --- для локальной области памяти).

* `--bar (-b)`

  Идентификатор (номер) BAR'а устройства, с которым будет производиться работа.

* `--size (-s)`

  Размер данных в байтах. Конкретное назначение параметра зависит от команды (`--cmd`).

* `--offset (-o)`

  Смещение в байтах. Конкретное назначение параметра зависит от команды (`--cmd`).

* `--value (-v)`

  32-битное значение. Конкретное назначение параметра зависит от команды (`--cmd`).

* `--r_size (-r)`

  Размер блока чтения в байтах. Конкретное назначение параметра зависит от команды (`--cmd`).

* `--w_size (-w)`

  Размер блока записи в байтах. Конкретное назначение параметра зависит от команды (`--cmd`).

* `--memcpy (-m)`

  Тип (метод) копирования данных. Может принимать одно из следующих значений:

  * `default`

    Для копирования данных используются функции `copy_to_user()` и `copy_from_user()`.

  * `io`

    Для копирования данных используются функции `memcpy_toio()` и `memcpy_fromio()`.

  * `io8`

    Для копирования данных используются функции `iowrite8()` и `ioread8()`.

  * `io16`

    Для копирования данных используются функции `iowrite16()` и `ioread16()`.

  * `io32`

    Для копирования данных используются функции `iowrite32()` и `ioread32()`.

  * `io64`

    Для копирования данных используются функции `writeq()` и `readq()`.


* `--cmd (-c)`

  Выбор команды для выполнения. Доступны следующие команды:

  * `wr`

    Записывает заданный объём данных (параметр `--size`) по заданному смещению (параметр `--offset`) в заданный BAR устройства (параметры `--bar` и `--dev`).

    Весь объем записываемых данных заполняется 32-х битным значением, заданным в параметре `--value`.

    Запись данных на уровне ядра Linux производится методом, указанным в параметре `--memcpy`.

    Значение параметра `--w_size` игнорируется. На уровне пользовательского приложения запись данных производится всем объемом сразу.

  * `rd`

    Считывает заданный объём данных (параметр `--size`) с заданным смещением (параметр `--offset`) с заданного BAR-а устройства (параметры `--bar` и `--dev`).

    Чтение данных на уровне ядра Linux производится методом, указанным в параметре `--memcpy`.

    Значение параметра `--r_size` игнорируется. На уровне пользовательского приложения чтение данных производится всем объемом сразу.

    Шестнадцатеричный дамп считанных данных выводится в терминал.

  * `wrrd`

    Записывает `--size` байт 32-х битного счетчика в BAR с номером `--bar` устройства с идентификатором `--dev` со смещением, заданным в параметре `--offset`.

    Далее, записанный счетчик считывается обратно и выполняется проверка считанного счетчика на валидность.

    Запись/чтение данных на уровне ядра Linux производится методом, указанным в параметре `--memcpy`.

    Значения параметров `--w_size` и `--r_size` игнорируются. На уровне пользовательского приложения запись/чтение данных производится всем объемом сразу.

    Результаты валидации считанного счетчика выводятся в терминал.

  * `rw_test`

    Данная команда выполняет многократную запись/чтение заданного объема данных (параметр `--size`) по заданному смещению (параметр `--offset`) в заданном BAR'е устройства (параметры `--bar` и `--dev`).

    Объём данных задаётся параметром `--size` и может принимать только значения 4 (32-бит), 8 (64-бита) или 16 (128-бит) байт.

    Данные записываются/считываются блоками, размер которых задаётся параметрами `--w_size` и `--r_size`. Данное разделение записи/чтения на блоки происходит только на уровне пользовательского приложения теста. Запись/чтение данных на уровне ядра Linux производится методом, указанным в параметре `--memcpy`. Если параметры `--w_size` и/или `--r_size` не заданы или равны 0, то на уровне пользовательского приложения, запись/чтение всего объема данных будет производиться одним целым блоком.

    После каждой операции записи/чтения полного объема данных выполняется сравнение считанных данных с записанными. В случае, если считанные данные не соответствуют записанным, в терминал выводится соответствующее сообщение.

## Примеры

1. Пример вывода dmesg после запуска драйвера:

	```
	hw_test: Hardware test driver started
	hw_test [00]: Allocated 2097152 bytes of DMA memory
	hw_test [00]: Memory virtual base = 0xffff88022b600000, physical (bus) base = 0x000000022b600000
	                                                                             ^^^^^^^^^^^^^^^^^^^^
	                                                                             физ. адрес
	                                                                             локальной памяти
	                                                                             (не связно с PCI устройством)
	hw_test [00]: Device initialized
	hw_test: Founded PCI device (vendor = 0x104C, device = 0xB005)
	Setted 32-bit DMA bitmask
	  * BAR0: physical = 0xdfc00000, virtual = 0xffffc90000c7a000, length = 4096
	  * BAR1: -- DISABLED --
	  * BAR2: physical = 0xd2000000, virtual = 0xffffc90011380000, length = 1048576
	  * BAR3: -- DISABLED --
	  * BAR4: physical = 0xd0000000, virtual = 0xffffc90012e80000, length = 33554432
	  * BAR5: -- DISABLED --
	hw_test [01]: Device is supported MSI
	hw_test 0000:06:00.0: irq 63 for MSI/MSI-X
	hw_test [01]: MSI enabled
	hw_test [01]: Registered the IRQ 63 for device
	hw_test [01]: Allocated 2097152 bytes of DMA memory
	hw_test [01]: Memory virtual base = 0xffff8800b9800000, physical (bus) base = 0x00000000b9800000
	         ^^                                                                  ^^^^^^^^^^^^^^^^^^^^
	     идентификатор устрйоства                                                физ. адрес
	                                                                             локальной памяти
	                                                                             (PCI устройство 1)
	hw_test [01]: Device initialized
	hw_test: Created character device "hw_test"
	```

2. Записываем в BAR2 1-го устройства 1024 двойных слов (4096 байт) счетчика, затем считываем и валидируем счетчик:

    ```
    ./hw_test --dev=1 --cmd=wrrd --bar=2 --size=4096

    Writing counter to BAR2 of DEV1:
     Offset = 0 bytes
     Size   = 1024 DWORD's (4096 bytes)
    Reading counter from BAR2 of DEV1:
     Offset = 0 bytes
     Size   = 1024 DWORD's (4096 bytes)
    Checking counter...
    Readed counter is OK
    Success
    ```

3. Считываем из BAR2 1-го устройства 1024 двойных слов (4096 байт) счетчика и выводим дамп:

    ```
    ./hw_test --dev=1 --cmd=rd --bar=2 --size=4096

    Reading from BAR2 of DEV1:
     Offset = 0 bytes
     Size   = 1024 DWORD's (4096 bytes)
    Success

    Memory dump: 4096 bytes from 0000000000db2010:
    0x00000000: 00000000 01000000 02000000 03000000
    0x00000010: 04000000 05000000 06000000 07000000
    0x00000020: 08000000 09000000 0a000000 0b000000
    0x00000030: 0c000000 0d000000 0e000000 0f000000
    0x00000040: 10000000 11000000 12000000 13000000

                  ...................

    0x00000fb0: ec030000 ed030000 ee030000 ef030000
    0x00000fc0: f0030000 f1030000 f2030000 f3030000
    0x00000fd0: f4030000 f5030000 f6030000 f7030000
    0x00000fe0: f8030000 f9030000 fa030000 fb030000
    0x00000ff0: fc030000 fd030000 fe030000 ff030000
    ```
