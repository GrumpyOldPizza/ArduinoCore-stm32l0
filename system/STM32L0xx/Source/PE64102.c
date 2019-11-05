#include "PE64102.h"
#include "../../../system/STM32L0xx/Include/stm32l0_spi.h"
#include "../../../system/STM32L0xx/Include/stm32l0_gpio.h"
#include "armv6m_systick.h"


stm32l0_spi_t spi;

const stm32l0_spi_params_t params =
{
    STM32L0_SPI_INSTANCE_SPI2,
    2,
    STM32L0_DMA_CHANNEL_DMA1_CH6_SPI2_RX,
    STM32L0_DMA_CHANNEL_NONE,
    {
        STM32L0_GPIO_PIN_PB15_SPI2_MOSI,
        STM32L0_GPIO_PIN_PB14_SPI2_MISO,
        STM32L0_GPIO_PIN_PB13_SPI2_SCK,
        STM32L0_GPIO_PIN_NONE,
    },
};

static void delay_ms(uint32_t milliseconds)
{
    uint32_t start =  armv6m_systick_micros();  
    while((start - armv6m_systick_micros()) < (milliseconds * 1000))
    {} 
}

/*
 * @brief Sets value of digital capacitor inside a specified DTC chip
 * @param[in] serial_enable, needed to start communication with specific chip
 * @param[in] cap_value, 5 bit value that sets the value of DTC
 * @return 0 if setting failed, because cap_value was to high, other wise return 1
 * @note Automaticaly activates chip, typical consumption is 30 uA 
 */
uint8_t DTC_set( uint8_t serial_enable, uint8_t cap_value )
{

    // Configure serial enable as output and set it to low.
	stm32l0_gpio_pin_configure( serial_enable, (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_MEDIUM | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    __stm32l0_gpio_pin_write( serial_enable, 0) ;



    if ( 0b11111 < cap_value )
    {
        //Capacitor value out of bounds, do not send 
        return 0;
    }

    //Begin
    stm32l0_spi_create(&spi, &params);
    stm32l0_spi_enable(&spi);
    stm32l0_spi_release(&spi);
    stm32l0_spi_acquire(&spi, 4000000, 0); //4MHz, MSBFIRST, SPIMODE0

    delay_ms(1);

    // Set serial enable to high
    __stm32l0_gpio_pin_write( serial_enable, 1) ;
    
    //Transfer cap_value
    stm32l0_spi_data(&spi, cap_value);
    delay_ms(1);

    // Set serial enable to low
    __stm32l0_gpio_pin_write( serial_enable, 0) ;

    // End
    stm32l0_spi_release(&spi);
    return 1;
}


/*
 * @brief Puts DTC chip into low current standby mode where it consumes 20 uA.
 * @param[in] serial_enable, needed to start communication with specific chip
 * @return none
 */
void DTC_deactivate( uint8_t serial_enable )
{
    // Configure serial enable as output and set it to low.
	stm32l0_gpio_pin_configure( serial_enable, (STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_MEDIUM | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    __stm32l0_gpio_pin_write( serial_enable, 0) ;

    //Begin
    stm32l0_spi_create(&spi, &params);
    stm32l0_spi_enable(&spi);
    stm32l0_spi_release(&spi);
    stm32l0_spi_acquire(&spi, 4000000, 0); //4MHz, MSBFIRST, SPIMODE0

    delay_ms(1);

    // Set serial enable to high
    __stm32l0_gpio_pin_write( serial_enable, 1) ;
    
    //Transfer cap_value
    stm32l0_spi_data(&spi, 0b00100000);
    delay_ms(1);

    // Set serial enable to low
    __stm32l0_gpio_pin_write( serial_enable, 0) ;

    // End
    stm32l0_spi_release(&spi);
}
