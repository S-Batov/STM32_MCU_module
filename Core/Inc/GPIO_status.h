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

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
	uint32_t raw;
	uint32_t max;
	uint32_t min;
	float percentage;
} Analog_IN;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} Digital_IN;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;

    GPIO_TypeDef *en_port;
    uint16_t en_pin;

    GPIO_TypeDef *fb_port;
    uint16_t fb_pin;
} Digital_OUT;

Analog_IN AIN[AIN_NUM] =
{
	[0] = { AIN_1_GPIO_Port, /* port */
			AIN_1_Pin,       /* pin */
			0,               /* raw */
			ADC_MAX_VAL_12B, /* max */
			0,               /* min */
			0                /* percentage */
		},

	[1] = { AIN_2_GPIO_Port, /* port */
			AIN_2_Pin,       /* pin */
			0,               /* raw */
			ADC_MAX_VAL_12B, /* max */
			0,               /* min */
			0                /* percentage */
		}
};

Digital_IN DIN[DIN_NUM] =
{
	[0] = { DIN_1_GPIO_Port, DIN_1_Pin },
	[1] = { DIN_2_GPIO_Port, DIN_2_Pin }
};

Digital_OUT DOUT[DOUT_NUM] =
{
	[0] = { DOUT_1_GPIO_Port,    /* port */
			DOUT_1_Pin,          /* pin */
			DOUT_1_EN_GPIO_Port, /* en_port */
			DOUT_1_EN_Pin,       /* en_pin */
			DOUT_1_FB_GPIO_Port, /* fb_port */
			DOUT_1_FB_Pin        /* fb_pin */
		},

	[1] = { DOUT_2_GPIO_Port,    /* port */
			DOUT_2_Pin,          /* pin */
			DOUT_2_EN_GPIO_Port, /* en_port */
			DOUT_2_EN_Pin,       /* en_pin */
			DOUT_2_FB_GPIO_Port, /* fb_port */
			DOUT_2_FB_Pin        /* fb_pin */
		}
};

#endif /* _GPIO_status_H_ */
