#ifndef _DEBUG_H
#define _DEBUG_H

#include <stdio.h>
#include "rtc.h"

#define DEBUG

void printf_time(void);
void printf_buff(uint8_t *buff, uint8_t len, char end_char);

#endif
