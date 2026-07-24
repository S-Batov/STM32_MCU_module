/**
 * @file GPIO_status.h
 * @brief Centralized structure to keep track of general
 * purpose analog and digital pins.
 */

#ifndef _GPIO_status_H_
#define _GPIO_status_H_

#include "main.h"

#define AIN_NUM  2
#define DIN_NUM  2
#define DOUT_NUM 2

#define ADC_MAX_VAL_12B 4095

/* --- Analog input structure ------------------------------------------------*/
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
	volatile uint32_t raw;		/* Raw ADC value 0-4095 */
	uint32_t max;		/* Calibrated  maximum expected value */
	uint32_t min;		/* Calibrated minimum expected value  */
	volatile float percentage;	/* ADC value as percentage of range 0.0 - 1.0 */
} Analog_IN;

extern Analog_IN AIN[AIN_NUM];

/* --- Digital input structure -----------------------------------------------*/
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} Digital_IN;

extern Digital_IN DIN[DIN_NUM];

/* --- Digital output structure ----------------------------------------------*/
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;

    GPIO_TypeDef *en_port;
    uint16_t en_pin;

    GPIO_TypeDef *fb_port;
    uint16_t fb_pin;
} Digital_OUT;

extern Digital_OUT DOUT[DOUT_NUM];

#endif /* _GPIO_status_H_ */
