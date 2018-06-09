/*
 * \file      sx1272mb2das-board.c
 *
 * \brief     Target board SX1272MB2DAS shield driver implementation
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
#include "sx1272-board.h"

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1272Init,
    SX1272GetStatus,
    SX1272SetModem,
    SX1272SetChannel,
    SX1272IsChannelFree,
    SX1272Random,
    SX1272SetRxConfig,
    SX1272SetTxConfig,
    SX1272CheckRfFrequency,
    SX1272GetTimeOnAir,
    SX1272Send,
    SX1272SetSleep,
    SX1272SetStby,
    SX1272SetRx,
    SX1272StartCad,
    SX1272SetTxContinuousWave,
    SX1272ReadRssi,
    SX1272Write,
    SX1272Read,
    SX1272WriteBuffer,
    SX1272ReadBuffer,
    SX1272SetMaxPayloadLength,
    SX1272SetPublicNetwork,
    SX1272SetSyncWord,
    SX1272SetModulation,
    SX1272SetAfc,
    SX1272SetDcFree,
    SX1272SetCrcType,
    SX1272SetAddressFiltering,
    SX1272SetNodeAddress,
    SX1272SetBroadcastAddress,
    SX1272SetOokFloorThreshold,
    SX1272SetLnaBoost,
    SX1272SetClockRate,
    SX1272GetWakeupTime
};

/* NUCLEO-L053R8 & NUCLEO-L073RZ
 */
#define RADIO_RESET                          STM32L0_GPIO_PIN_PA0           // A0

#define RADIO_MOSI                           STM32L0_GPIO_PIN_PA7_SPI1_MOSI // D11
#define RADIO_MISO                           STM32L0_GPIO_PIN_PA6_SPI1_MISO // D12
#define RADIO_SCLK                           STM32L0_GPIO_PIN_PA5_SPI1_SCK  // D13
#define RADIO_NSS                            STM32L0_GPIO_PIN_PB6           // D10

#define RADIO_DIO_0                          STM32L0_GPIO_PIN_PA10          // D2
#define RADIO_DIO_1                          STM32L0_GPIO_PIN_PB3           // D3
#define RADIO_DIO_2                          STM32L0_GPIO_PIN_PB5           // D4
// #define RADIO_DIO_3                          STM32L0_GPIO_PIN_PB4           // D5

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

static void SX1272ExtiCallback( void *context )
{
    armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)context, NULL, 0);
}

void SX1272Reset( void )
{
    // Set RESET pin to 0
    stm32l0_gpio_pin_configure(RADIO_RESET, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_RESET, 0);

    // Wait 1 ms
    SX1272Delay( 1 );

    // Configure RESET as input
    stm32l0_gpio_pin_configure(RADIO_RESET, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));

    // Wait 6 ms
    SX1272Delay( 6 );

    SX1272Write( REG_OCP, ( RF_OCP_ON | RF_OCP_TRIM_090_MA ) );

    SX1272Write( REG_OPMODE, ( SX1272Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP );

    SX1272Release( );
}

void SX1272SetBoardTcxo( bool state )
{
}

void SX1272AntSwInit( void )
{
}

void SX1272AntSwDeInit( void )
{
}

void SX1272SetAntSw( uint8_t opMode )
{
}

void SX1272DioInit( void )
{
    stm32l0_gpio_pin_configure(RADIO_DIO_0, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(RADIO_DIO_1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(RADIO_DIO_2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    
    stm32l0_exti_attach(RADIO_DIO_0, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1272ExtiCallback, SX1272OnDio0Irq);
    stm32l0_exti_attach(RADIO_DIO_1, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1272ExtiCallback, SX1272OnDio1Irq);
    stm32l0_exti_attach(RADIO_DIO_2, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1272ExtiCallback, SX1272OnDio2Irq);
}

void SX1272DioDeInit( void )
{
    stm32l0_exti_detach(RADIO_DIO_0);
    stm32l0_exti_detach(RADIO_DIO_1);
    stm32l0_exti_detach(RADIO_DIO_2);
    
    stm32l0_gpio_pin_configure(RADIO_DIO_0, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(RADIO_DIO_1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(RADIO_DIO_2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
}

void SX1272SetDio1Edge( bool rising )
{
    stm32l0_exti_control(RADIO_DIO_1, (rising ? STM32L0_EXTI_CONTROL_EDGE_RISING : STM32L0_EXTI_CONTROL_EDGE_FALLING));
}

void SX1272SetRfTxPower( int8_t power )
{
    uint8_t paConfig, paDac;

    paConfig = RF_PACONFIG_PASELECT_RFO;
    paDac = RF_PADAC_20DBM_OFF;

    if( power < -1 )
    {
	power = -1;
    }
    if( power > 14 )
    {
	power = 14;
    }
    
    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( power + 1 );

    SX1272Write( REG_PACONFIG, paConfig );
    SX1272Write( REG_PADAC, paDac );
}

bool SX1272CheckRfFrequency( uint32_t frequency )
{
    if( (frequency < 862000000) || (frequency > 1020000000) )
    {
        return false;
    }

    return true;
}

uint32_t SX1272GetBoardTcxoWakeupTime( void )
{
    return 0;
}

void SX1272MB2DAS_Initialize( void )
{
    stm32l0_gpio_pin_configure(RADIO_NSS, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_NSS, 1);

    SX1272.RadioNss = RADIO_NSS;

    stm32l0_spi_create(&SX1272.RadioSpi, &RADIO_SPI_PARAMS);
    stm32l0_spi_enable(&SX1272.RadioSpi);

    SX1272Reset( );
}
