/*
 * ap_types.h
 *
 *  Created on: Nov 19, 2025
 *      Author: xodrb
 */

#ifndef INC_APP_TYPES_H_
#define INC_APP_TYPES_H_


#include <stdint.h>

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} joystick_sample_t;

// nRF로 보내는 payload 예시 (원하는대로 정의)
typedef struct {
    int16_t tx_x;
    int16_t tx_y;
    int16_t tx_z;
} payload_t;

#endif /* INC_APP_TYPES_H_ */
