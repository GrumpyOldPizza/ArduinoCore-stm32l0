/*!
 * \file      lgt92-board.c
 *
 * \brief     Target board RFM95 module driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include "utilities.h"
#include "radio.h"
#include "sx1276-board.h"
#include "stm32l0_rtc.h"

#if defined(STM32L072xx)

#define RADIO_RESET                          STM32L0_GPIO_PIN_PB0

#define RADIO_MOSI                           STM32L0_GPIO_PIN_PA7_SPI1_MOSI
#define RADIO_MISO                           STM32L0_GPIO_PIN_PA6_SPI1_MISO
#define RADIO_SCLK                           STM32L0_GPIO_PIN_PA5_SPI1_SCK
#define RADIO_NSS                            STM32L0_GPIO_PIN_PA15_SPI1_NSS

#define RADIO_DIO_0                          STM32L0_GPIO_PIN_PC13
#define RADIO_DIO_1                          STM32L0_GPIO_PIN_PB10
#define RADIO_DIO_2                          STM32L0_GPIO_PIN_PB11

static const stm32l0_spi_params_t RADIO_SPI_PARAMS = {
    STM32L0_SPI_INSTANCE_SPI1,
    0,
    STM32L0_DMA_CHANNEL_NONE,
    STM32L0_DMA_CHANNEL_NONE,
    {
        RADIO_MOSI,
        RADIO_MISO,
        RADIO_SCLK,
        STM32L0_GPIO_PIN_NONE,
    },
};

static stm32l0_spi_t RADIO_SPI;

static void (*RADIO_DONE_IRQ)(void);

void SWI_RADIO_IRQHandler(void)
{
    (*RADIO_DONE_IRQ)();
}

static void SX1276OnRadioDone( void )
{
    // ### CAPUTRE RTC here
    armv6m_pendsv_raise(ARMV6M_PENDSV_SWI_RADIO);
}

void SX1276Delay( uint32_t timeout )
{
    uint32_t now, start, end;

    now = stm32l0_rtc_clock_read();
    start = now;
    end = start + stm32l0_rtc_millis_to_ticks(timeout);

    do
    {
        now = stm32l0_rtc_clock_read();
    }
    while ((now - start) < (end - start));
}

void SX1276Reset( void )
{
    // Set RESET pin to 0
    stm32l0_gpio_pin_configure(RADIO_RESET, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_RESET, 0);

    // Wait 1 ms
    SX1276Delay( 1 );

    // Configure RESET as input
    stm32l0_gpio_pin_configure(RADIO_RESET, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));

    // Wait 6 ms
    SX1276Delay( 6 );

    SX1276Write( REG_OCP, ( RF_OCP_ON | RF_OCP_TRIM_120_MA ) );
    SX1276Write( REG_TCXO, ( SX1276Read( REG_TCXO ) & RF_TCXO_TCXOINPUT_MASK ) );

    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP );

    SX1276Release( );
}

void SX1276SetBoardTcxo( bool state )
{
}

void SX1276AntSwInit( void )
{
}

void SX1276AntSwDeInit( void )
{
}

void SX1276SetAntSw( uint8_t opMode, int8_t power )
{
}

void SX1276DioInit(  RadioModems_t modem, RadioState_t state, void (*dio0Irq)(void), void (*dio1Irq)(void), void (*dio2Irq)(void) )
{
    RADIO_DONE_IRQ = dio0Irq;
    
    stm32l0_gpio_pin_configure(RADIO_DIO_0, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(RADIO_DIO_1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(RADIO_DIO_2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));

    stm32l0_exti_attach(RADIO_DIO_0, (STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL | STM32L0_EXTI_CONTROL_EDGE_RISING), (stm32l0_exti_callback_t)SX1276OnRadioDone, NULL);

    if( ( modem == MODEM_FSK ) && ( state == RF_TX_RUNNING ) )
    {
        stm32l0_exti_attach(RADIO_DIO_1, (STM32L0_EXTI_CONTROL_PRIORITY_LOW | STM32L0_EXTI_CONTROL_EDGE_FALLING), (stm32l0_exti_callback_t)dio1Irq, NULL);
    }
    else
    {
        stm32l0_exti_attach(RADIO_DIO_1, (STM32L0_EXTI_CONTROL_PRIORITY_LOW | STM32L0_EXTI_CONTROL_EDGE_RISING), (stm32l0_exti_callback_t)dio1Irq, NULL);
    }

    if( modem == MODEM_FSK )
    {
        stm32l0_exti_attach(RADIO_DIO_2, (STM32L0_EXTI_CONTROL_PRIORITY_LOW | STM32L0_EXTI_CONTROL_EDGE_RISING), (stm32l0_exti_callback_t)dio2Irq, NULL);
    }
    else
    {
        stm32l0_exti_attach(RADIO_DIO_2, (STM32L0_EXTI_CONTROL_PRIORITY_CRITICAL | STM32L0_EXTI_CONTROL_EDGE_RISING), (stm32l0_exti_callback_t)dio2Irq, NULL);
    }
}

void SX1276DioDeInit( void )
{
    stm32l0_exti_detach(RADIO_DIO_0);
    stm32l0_exti_detach(RADIO_DIO_1);
    stm32l0_exti_detach(RADIO_DIO_2);
    
    stm32l0_gpio_pin_configure(RADIO_DIO_0, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(RADIO_DIO_1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(RADIO_DIO_2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig, paDac;

    if( power < -4 )
    {
        power = -4;
    }
    if( power > 15 )
    {
        power = 15;
    }


    if( power > 0 )
    {
        paConfig = ( RF_PACONFIG_PASELECT_RFO | ( 7 << 4 ) | ( power ) );
        paDac = RF_PADAC_20DBM_OFF;
    }
    else
    {
        paConfig = ( RF_PACONFIG_PASELECT_RFO | ( 0 << 4 ) | ( power + 4 ) );
        paDac = RF_PADAC_20DBM_OFF;
    }

    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, ( ( SX1276Read( REG_PADAC ) & RF_PADAC_20DBM_MASK ) | paDac ) );
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    if( (frequency < 400000000) || (frequency > 1020000000) )
    {
        return false;
    }

    return true;
}

uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    return 0;
}

void SX1276Acquire( void )
{
    if( RADIO_SPI.state != STM32L0_SPI_STATE_DATA )
    {
        stm32l0_spi_acquire(&RADIO_SPI, 8000000, 0);
    }
}

void SX1276Release( void )
{
    if( RADIO_SPI.state == STM32L0_SPI_STATE_DATA )
    {
        stm32l0_spi_release(&RADIO_SPI);
    }
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276Acquire( );

    stm32l0_gpio_pin_write(RADIO_NSS, 0);

    stm32l0_spi_data8(&RADIO_SPI, addr | 0x80);
    stm32l0_spi_data8(&RADIO_SPI, data);

    stm32l0_gpio_pin_write(RADIO_NSS, 1);
}

uint8_t SX1276Read( uint8_t addr )
{
    uint8_t data;

    SX1276Acquire( );

    stm32l0_gpio_pin_write(RADIO_NSS, 0);

    stm32l0_spi_data8(&RADIO_SPI, addr & ~0x80);
    data = stm32l0_spi_data8(&RADIO_SPI, 0xff);

    stm32l0_gpio_pin_write(RADIO_NSS, 1);

    return data;
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    SX1276Acquire( );

    stm32l0_gpio_pin_write(RADIO_NSS, 0);

    stm32l0_spi_data8(&RADIO_SPI, addr | 0x80);
    stm32l0_spi_data(&RADIO_SPI, buffer, NULL, size);

    stm32l0_gpio_pin_write(RADIO_NSS, 1);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    SX1276Acquire( );

    stm32l0_gpio_pin_write(RADIO_NSS, 0);

    stm32l0_spi_data8(&RADIO_SPI, addr & ~0x80);
    stm32l0_spi_data(&RADIO_SPI, NULL, buffer, size);

    stm32l0_gpio_pin_write(RADIO_NSS, 1);
}

void RFM95_Initialize( void )
{
    uint32_t datarate, primask;

    stm32l0_gpio_pin_configure(RADIO_NSS, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_NSS, 1);

    stm32l0_spi_create(&RADIO_SPI, &RADIO_SPI_PARAMS);
    stm32l0_spi_enable(&RADIO_SPI);

    SX1276Reset( );

    datarate = ( 16 * XTAL_FREQ ) / 2048;
    SX1276Write( REG_BITRATEMSB,  ( uint8_t )( datarate >> 12 ) );
    SX1276Write( REG_BITRATELSB,  ( uint8_t )( datarate >> 4  ) );
    SX1276Write( REG_BITRATEFRAC, ( uint8_t )( datarate >> 0  ) );

    SX1276Write( REG_PACONFIG, 0x00 );
    SX1276Write( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_TRANSMITTER );

    // Wait 25 ms
    SX1276Delay( 25 );

    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP );

    SX1276Release( );
}

#endif /* defined(STM32L072xx) */
