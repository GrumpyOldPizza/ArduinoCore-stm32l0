/*!
 * \file      cmwx1zzabz-board.c
 *
 * \brief     Target board CMWX1ZZABZ module driver implementation
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

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276SetSyncWord,
    SX1276SetModulation,
    SX1276SetAfc,
    SX1276SetDcFree,
    SX1276SetCrcType,
    SX1276SetAddressFiltering,
    SX1276SetNodeAddress,
    SX1276SetBroadcastAddress,
    SX1276SetOokFloorThreshold,
    SX1276SetLnaBoost,
    SX1276SetClockRate,
    SX1276GetWakeupTime
};

#define RADIO_RESET                          STM32L0_GPIO_PIN_PC0

#define RADIO_MOSI                           STM32L0_GPIO_PIN_PA7_SPI1_MOSI
#define RADIO_MISO                           STM32L0_GPIO_PIN_PA6_SPI1_MISO
#define RADIO_SCLK                           STM32L0_GPIO_PIN_PB3_SPI1_SCK
#define RADIO_NSS                            STM32L0_GPIO_PIN_PA15_SPI1_NSS

#define RADIO_DIO_0                          STM32L0_GPIO_PIN_PB4
#define RADIO_DIO_1                          STM32L0_GPIO_PIN_PB1
#define RADIO_DIO_2                          STM32L0_GPIO_PIN_PB0
//#define RADIO_DIO_3                          STM32L0_GPIO_PIN_PC13

//#define RADIO_TCXO_VCC                       STM32L0_GPIO_PIN_PH1

#define RADIO_ANT_SWITCH_RX                  STM32L0_GPIO_PIN_PA1
#define RADIO_ANT_SWITCH_TX_RFO              STM32L0_GPIO_PIN_PC2
#define RADIO_ANT_SWITCH_TX_BOOST            STM32L0_GPIO_PIN_PC1

#define BOARD_TCXO_WAKEUP_TIME               4

static uint8_t RADIO_TCXO_VCC;

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

static void SX1276ExtiCallback( void *context )
{
    armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)context, NULL, 0);
}

void SX1276Reset( void )
{
    if (RADIO_TCXO_VCC != STM32L0_GPIO_PIN_NONE)
    {
        SX1276SetBoardTcxo( true );

        SX1276Delay( BOARD_TCXO_WAKEUP_TIME );
    }

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
    SX1276Write( REG_TCXO, ( SX1276Read( REG_TCXO ) & RF_TCXO_TCXOINPUT_MASK ) | RF_TCXO_TCXOINPUT_ON );

    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP );

    SX1276Release( );

    if (RADIO_TCXO_VCC != STM32L0_GPIO_PIN_NONE)
    {
        SX1276Delay( 1 );

        SX1276SetBoardTcxo( false );
    }
}

void SX1276SetBoardTcxo( bool state )
{
    if (RADIO_TCXO_VCC != STM32L0_GPIO_PIN_NONE)
    {
        if( state == true )
        {
            stm32l0_gpio_pin_configure(RADIO_TCXO_VCC, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
            stm32l0_gpio_pin_write(RADIO_TCXO_VCC, 1);
        }
        else
        {
            stm32l0_gpio_pin_configure(RADIO_TCXO_VCC, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
        }
    }
}

void SX1276AntSwInit( void )
{
    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_RX,       (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_TX_RFO,   (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_TX_BOOST, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
}

void SX1276AntSwDeInit( void )
{
    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_RX,       (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_TX_RFO,   (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_TX_BOOST, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
}

void SX1276SetAntSw( uint8_t opMode )
{
    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_RX,       0);
    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_TX_RFO,   0);
    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_TX_BOOST, 0);

    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        if( SX1276.Settings.Power > 14 )
        {
            stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_TX_BOOST, 1);
        }
        else
        {
            stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_TX_RFO, 1);
        }
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
        stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_RX, 1);
        break;
    default:
        break;
    }
}

void SX1276DioInit( void )
{
    stm32l0_gpio_pin_configure(RADIO_DIO_0, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(RADIO_DIO_1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    stm32l0_gpio_pin_configure(RADIO_DIO_2, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
    
    stm32l0_exti_attach(RADIO_DIO_0, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1276ExtiCallback, SX1276OnDio0Irq);
    stm32l0_exti_attach(RADIO_DIO_1, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1276ExtiCallback, SX1276OnDio1Irq);
    stm32l0_exti_attach(RADIO_DIO_2, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1276ExtiCallback, SX1276OnDio2Irq);
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

void SX1276SetDio1Edge( bool rising )
{
    stm32l0_exti_control(RADIO_DIO_1, (rising ? STM32L0_EXTI_CONTROL_EDGE_RISING : STM32L0_EXTI_CONTROL_EDGE_FALLING));
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig, paDac;

    if ( power > 15 )
    {
	paConfig = RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
	paConfig = RF_PACONFIG_PASELECT_RFO;
    }

    paDac = RF_PADAC_20DBM_OFF;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( power - 5 );
            paDac = RF_PADAC_20DBM_ON;
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( power - 2 );
        }
    }
    else
    {
	if( power > 0 )
	{
	    if( power > 15 )
	    {
		power = 15;
	    }

            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 7 << 4 ) | ( power - 0 );
	}
	else
	{
	    if( power < -4 )
	    {
		power = -4;
	    }

            paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( 0 << 4 ) | ( power + 4 );
	}
    }

    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    if( (frequency < 862000000) || (frequency > 1020000000) )
    {
        return false;
    }

    return true;
}

uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    if( RADIO_TCXO_VCC != STM32L0_GPIO_PIN_NONE )
    {
        return BOARD_TCXO_WAKEUP_TIME;
    }

    return 0;
}

void CMWX1ZZABZ_Initialize( uint8_t pin_tcxo, uint16_t pin_stsafe )
{
    RADIO_TCXO_VCC = pin_tcxo;

    stm32l0_gpio_pin_configure(RADIO_NSS, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_NSS, 1);

    SX1276.RadioNss = RADIO_NSS;

    stm32l0_spi_create(&SX1276.RadioSpi, &RADIO_SPI_PARAMS);
    stm32l0_spi_enable(&SX1276.RadioSpi);

    SX1276Reset( );
}
