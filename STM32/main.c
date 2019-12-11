#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_dma.h"

#include "xprintf.h"
#include "oled_driver.h"

#define MAX_ALLOWED_HUMIDITY 40


/*
 * Structure for communication
 */
typedef struct {
    uint8_t cmd;
    uint8_t params[10];
    uint8_t active;
} uart_req_t;
static uart_req_t uart_req;


/*
 * Structure for keeping state of the system
 */
typedef struct $
{
	// light indoor
	uint8_t light_state;
	uint8_t light_action;

	// light outdoor
	uint8_t light_outside_state;
	uint8_t light_outside_action;

	// climat
	uint8_t temperature;
	uint8_t humidity;

	// ventilation
	uint8_t ventilation;
	uint8_t ventilation_action;

	// fire signalization
	uint8_t signalization_state;

	// door
	uint8_t door;
	uint8_t door_action;

	// fire sensor
	uint16_t adc_buffer[8];
	uint16_t adc_value;

} state_t;
static state_t state;


static state_init()
{
	state.light_state = 0;
	state.light_action = 0;

	state.light_outside_state = 0;
	state.light_outside_action = 0;

	state.temperature = 0;
	state.humidity = 0;

	state.ventilation = 0;
	state.ventilation_action = 0;

	state.door = 0;
	state.door_action = 0;

	state.signalization_state = 0;

	state.adc_value = 0;
}


/**
  * System Clock Configuration
  * The system Clock is configured as follow :
  *    System Clock source            = PLL (HSI/2)
  *    SYSCLK(Hz)                     = 48000000
  *    HCLK(Hz)                       = 48000000
  *    AHB Prescaler                  = 1
  *    APB1 Prescaler                 = 1
  *    HSI Frequency(Hz)              = 8000000
  *    PLLMUL                         = 12
  *    Flash Latency(WS)              = 1
  */
static void rcc_config()
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    SystemCoreClock = 48000000;
    return;
}


void gpio_config()
{	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	// light indoor
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
	// light outdoor 
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
	// ventilation 
	LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);

}


static void adc_config(void)
{
    /*
     * Setting ADC
     */
    /* Turn on ADC1 as peripheral */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
    /* Clock selection */
    LL_RCC_HSI14_Enable();
    LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_ASYNC);
    while (LL_RCC_HSI14_IsReady() != 1);
    /* ADC Calibration process */
    if (LL_ADC_IsEnabled(ADC1)) {
        LL_ADC_Disable(ADC1);
    }
    while (LL_ADC_IsEnabled(ADC1));
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));
    /* Turn on ADC */
    LL_ADC_Enable(ADC1);
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1,
                                         LL_ADC_SAMPLINGTIME_239CYCLES_5);
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_PRESERVED);
    /* DMA Setting */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1,
                                    LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1,
                            LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1,
                            LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1,
                                   LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 8);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&(ADC1->DR));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)state.adc_buffer);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    /* Enable interrupt */
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* Enable ADC conversion */
    LL_ADC_REG_StartConversion(ADC1);
    return;
}


void sound_config()
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, LL_GPIO_AF_2);
	/*
     * Setup timer to output compare mode
     */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	LL_TIM_SetPrescaler(TIM2, 59);
	LL_TIM_SetAutoReload(TIM2, 999);
	LL_TIM_OC_SetCompareCH1(TIM2, 500); 
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
	LL_TIM_EnableIT_CC1(TIM2);
	LL_TIM_EnableCounter(TIM2);

	/*
     * Setup NVIC
     */
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0);
} 

void servo_config()
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_1);
	/*
     * Setup timer to output compare mode
     */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	LL_TIM_SetPrescaler(TIM3, 959);
	LL_TIM_SetAutoReload(TIM3, 999);
	LL_TIM_OC_SetCompareCH1(TIM3, 125); 
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
	LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
	LL_TIM_EnableIT_CC1(TIM3);
	LL_TIM_EnableCounter(TIM3);

	/*
     * Setup NVIC
     */
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 0);

}

void TIM2_IRQHandler(void)
{
    //LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
    LL_TIM_ClearFlag_CC1(TIM2);
}

void TIM3_IRQHandler(void)
{
    LL_TIM_ClearFlag_CC1(TIM3);
}

void usart_config() 
{
	/*
	*  Seting usart pins
	*/
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	// USART1_TX
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_1);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
	// USART1_RX
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_1);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
	/*
	*  USART Set clock sorce
	*/
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
    /*
     * USART Setting
     */
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
    LL_USART_SetTransferBitOrder(USART1, LL_USART_BITORDER_LSBFIRST);
    LL_USART_SetBaudRate(USART1, SystemCoreClock,
                         LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_EnableIT_IDLE(USART1);
    LL_USART_EnableIT_RXNE(USART1);
    /*
     * USART turn on
     */
    LL_USART_Enable(USART1);
    while (!(LL_USART_IsActiveFlag_TEACK(USART1) &&
             LL_USART_IsActiveFlag_REACK(USART1)));
    /*
     * Turn on NVIC interrupt line
     */
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);
    return;
}


static void manage_requests(void) {
    uint8_t is_ok = 0;

    if (!uart_req.active)
        return;

    switch (uart_req.cmd) {
    case 'L': {
        state.light_state = uart_req.params[0] - '0';
        state.light_action = 1;
        break;
    }
    case 'O': {
    	state.light_outside_state = uart_req.params[0] - '0';
        state.light_outside_action = 1;
        break;
    }
    case 'S': {
    	state.signalization_state = uart_req.params[0] - '0';
    	break;
    }
    case 'V': {
        state.ventilation = uart_req.params[0] - '0';
        state.ventilation_action = 1;
        break;
    }
    case 'D': {
    	state.door = uart_req.params[0] - '0';
        state.door_action = 1;
        break;
    }
    case 'T': {
    	state.temperature = (uart_req.params[0] - '0') * 10 + (uart_req.params[1] - '0');
    	break;
    }
    case 'H': {
    	state.humidity = (uart_req.params[0] - '0') * 10 + (uart_req.params[1] - '0');
    	break;
    }
    default:
        is_ok = 0;
        break;
    }

    while (!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, is_ok + 0x30);

    uart_req.active = 0;
    return;
}


void USART1_IRQHandler(void)
{
    static uint8_t pos = 0;

    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        if (pos == 0) {
            uart_req.cmd = LL_USART_ReceiveData8(USART1);
        } else {
            uart_req.params[pos - 1] = LL_USART_ReceiveData8(USART1);
        }
        pos++;
    }
    if (LL_USART_IsActiveFlag_IDLE(USART1)) {
        pos = 0;
        uart_req.active = 1;
        LL_USART_ClearFlag_IDLE(USART1);
        manage_requests();
    }
    return;
}


void DMA1_Channel1_IRQHandler(void)
{
    uint16_t value = 0;
    for (int i = 0; i < 8; i++) {
        value += state.adc_buffer[i];
    }
    value /= 8;
    state.adc_value = value;

    LL_DMA_ClearFlag_TC1(DMA1);
}

/*
 * Set callback for out device
 */
static void printf_config(void)
{
    xdev_out(oled_putc);
    return;
}


int main(void)
{
	rcc_config();
	usart_config();
	gpio_config();
	sound_config();
	servo_config();
	adc_config();
	state_init();
	oled_config();
    printf_config();

    while (1)
    {   
    	// light inside
    	if (state.light_action) {
    		if (state.light_state == 1) {
				LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1);
    		} else {
    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1);
    		}
    		state.light_action = 0;
    	}

    	// light ouside
    	if (state.light_outside_action) {
    		if (state.light_outside_state == 1) {
				LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_2);
    		} else {
    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2);
    		}
    		state.light_outside_action = 0;
    	}

    	// ventilation
    	if (state.ventilation_action) {
    		if (state.ventilation == 2) {
    			if (state.humidity >= MAX_ALLOWED_HUMIDITY) {
    				LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0);
    			} else if (state.humidity <= MAX_ALLOWED_HUMIDITY - 5) {
    				LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
    			}
    		} else if (state.ventilation == 1) {
    			LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0);
    			state.ventilation_action = 0;
    		} else {
    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0);
    			state.ventilation_action = 0;
    		}
    	}

    	// door 
    	if (state.door_action) {
    		if (state.door == 1) {
				LL_TIM_OC_SetCompareCH1(TIM3, 25);
    		} else {
    			LL_TIM_OC_SetCompareCH1(TIM3, 125);
    		}
    		state.door_action = 0;
    	}

    	// fire signalization 
    	if (state.signalization_state && state.adc_value > 1000) {
    		// display
    		oled_clr(0);
    		oled_set_cursor(0, 0);

    		xprintf("\n\n\n       FIRE!!!    \n");

    		LL_TIM_OC_SetCompareCH1(TIM2, 500);
    	} else {
			// display
    		oled_clr(0);
    		oled_set_cursor(0, 0);

    		xprintf("\n    HUMIDITY    %d\n", state.humidity);
    		xprintf("\n    TEMPERATURE %d\n", state.temperature);
    		//xprintf("\n    VALUE       %d\n", state.adc_value);

    		LL_TIM_OC_SetCompareCH1(TIM2, 0);
    	}
    	oled_update();


    }
    return 0;
}
