#ifndef __ModBus_H
#define __ModBus_H

#include <board.h>
#include <stm32l4xx_hal.h>
#include <stdlib.h>
#include <finsh.h>
#include <rtthread.h>


uint16_t CRC16_Modbus(volatile uint8_t *buf, uint16_t len);




#endif


