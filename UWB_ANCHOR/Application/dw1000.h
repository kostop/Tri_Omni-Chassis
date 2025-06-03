#ifndef _DW1000_H_
#define _DW1000_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "dw1000.h"
#include "deca_device_api.h"
#include "deca_regs.h"

/*
    CS <-------------  PA4
    RST <------------- PB12          (PB9)
    WAKEUP <---------- PB13          (PA8)
*/
#define DW1000_CS_PORT			GPIOA
#define DW1000_CS_PIN			GPIO_PIN_4
#define DW1000_RST_PORT			GPIOB
#define DW1000_RST_PIN			GPIO_PIN_4
#define DW1000_WAKEUP_PORT		GPIOB
#define DW10000_WAKEUP_PIN		GPIO_PIN_3

/*
    IRQ -------------->  PB0      (PC9)
*/
#define DW1000_IRQn_TYPE		EXTI0_IRQn
#define DW1000_IRQ_PORT		  	GPIOB
#define DW1000_IRQ_PIN			GPIO_PIN_0

/*
    SPI Interface <---> SPI1
    SPI_CS <----------> PA4
    SPI_CLK <---------> PA5          (PA1)
    SPI_MISO <--------> PA6          (PA6)
    SPI_MOSI <--------> PA7          (PA12)
*/
extern SPI_HandleTypeDef hspi1;
#define DW1000_SPI_Handle hspi1

void reset_DW1000(void);
void spi_set_rate_low(void);
void spi_set_rate_high(void);

void dw_task(void const * argument);

#endif /* _DW1000_H_ */
