#include <stdio.h>
#include <stdlib.h>

#include "ble.h"

ble_packed_t _ble_pack_float(float value)
{
    ble_packed_t package;
    uint16_t tempValue;

    // multiply by 100 to get 2 digits mantissa and convert into uint16_t
    tempValue = (uint16_t)(value * 100);

    package.value[0] = tempValue;      // set  LSB of characteristic
    package.value[1] = tempValue >> 8; // set MSB of characteristic

    return package;
}

ble_packed_t _ble_pack_uint16(uint16_t value)
{
    ble_packed_t package;
    package.value[0] = value;      // set  LSB of characteristic
    package.value[1] = value >> 8; // set MSB of characteristic
    return package;
}
