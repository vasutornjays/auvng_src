#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include "../include/ftd2xx.h"

int InitUm232I2c();
u_int16_t ReadPROM(u_int8_t promAddressOffset);
u_int32_t ReadADC(u_int16_t dataAddress);