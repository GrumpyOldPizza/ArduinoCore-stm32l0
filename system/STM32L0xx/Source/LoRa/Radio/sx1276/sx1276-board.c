/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

#include "armv6m.h"
#include "stm32l0xx.h"

#include "radio.h"
#include "sx1276.h"
#include "sx1276-board.h"

#include "stm32l0_gpio.h"
#include "stm32l0_exti.h"
#include "stm32l0_spi.h"
#include "stm32l0_lptim.h"

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
    SX1276GetRadioWakeUpTime
};

static bool IsLowPower = true;
static bool IsAntSwLowPower = true;
static bool IsTCXO = false;

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

void CMWX1ZZABZ_Initialize( void )
{
    stm32l0_gpio_pin_configure(RADIO_NSS, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_NSS, 1);

    stm32l0_spi_create(&RADIO_SPI, &RADIO_SPI_PARAMS);
    stm32l0_spi_enable(&RADIO_SPI);

    SX1276SetTCXO( true );

    SX1276Reset( );

    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP );

    SX1276SetTCXO( false );

    SX1276SetLowPower( true );
}

static void SX1276IoIrqCallback( void *context )
{
    armv6m_pendsv_enqueue((armv6m_pendsv_routine_t)context, NULL, 0);
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
    stm32l0_exti_attach(RADIO_DIO_0, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1276IoIrqCallback, irqHandlers[0]);
    stm32l0_exti_attach(RADIO_DIO_1, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1276IoIrqCallback, irqHandlers[1]);
    stm32l0_exti_attach(RADIO_DIO_2, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1276IoIrqCallback, irqHandlers[2]);
#if defined(RADIO_DIO_3)
    stm32l0_exti_attach(RADIO_DIO_3, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1276IoIrqCallback, irqHandlers[3]);
#endif
#if defined(RADIO_DIO_4)
    stm32l0_exti_attach(RADIO_DIO_4, STM32L0_EXTI_CONTROL_EDGE_RISING, SX1276IoIrqCallback, irqHandlers[4]);
#endif
}

void SX1276Reset( void )
{
    // Set RESET pin to 0
    stm32l0_gpio_pin_configure(RADIO_RESET, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_RESET, 0);

    // Wait 1 ms
    armv6m_core_udelay(1000);

    // Configure RESET as input
    stm32l0_gpio_pin_configure(RADIO_RESET, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));

    // Wait 6 ms
    armv6m_core_udelay(6000);
}

void SX1276Delay( uint32_t timeout )
{
    stm32l0_lptim_start(((timeout * 32768 + 999) / 1000), NULL, NULL);

    while (!stm32l0_lptim_done());
    {
	__WFE();
    }
}

void SX1276SetLowPower( bool status )
{
    if (IsLowPower != status)
    {
	IsLowPower = status;
	
	if (status)
	{
	    stm32l0_spi_release(&RADIO_SPI);
	}
	else
	{
	    stm32l0_spi_acquire(&RADIO_SPI, 8000000, 0);
	}
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if (IsAntSwLowPower != status)
    {
        IsAntSwLowPower = status;

        if (status)
        {
	    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_RX,       0);
	    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_TX_BOOST, 0);
	    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_TX_RFO,   0);

	    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_RX,       (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
	    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_TX_BOOST, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
	    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_TX_RFO,   (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));

	    stm32l0_gpio_pin_configure(RADIO_DIO_0,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
	    stm32l0_gpio_pin_configure(RADIO_DIO_1,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
	    stm32l0_gpio_pin_configure(RADIO_DIO_2,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
#if defined(RADIO_DIO_3)
	    stm32l0_gpio_pin_configure(RADIO_DIO_3,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
#endif /* defined(RADIO_DIO_3) */
#if defined(RADIO_DIO_4)
	    stm32l0_gpio_pin_configure(RADIO_DIO_4,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
#endif /* defined(RADIO_DIO_4) */
        }
        else
        {
	    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_RX,       (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
	    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_TX_BOOST, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
	    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_TX_RFO,   (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));

	    stm32l0_gpio_pin_configure(RADIO_DIO_0,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
	    stm32l0_gpio_pin_configure(RADIO_DIO_1,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
	    stm32l0_gpio_pin_configure(RADIO_DIO_2,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
#if defined(RADIO_DIO_3)
	    stm32l0_gpio_pin_configure(RADIO_DIO_3,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
#endif /* defined(RADIO_DIO_3) */
#if defined(RADIO_DIO_4)
	    stm32l0_gpio_pin_configure(RADIO_DIO_4,               (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_INPUT));
#endif /* defined(RADIO_DIO_4) */
        }
    }
}

void SX1276SetAntSw( uint8_t opMode )
{
    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_RX,       0);
    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_TX_BOOST, 0);
    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_TX_RFO,   0);

    switch (opMode ){
    case RFLR_OPMODE_TRANSMITTER:
	if (SX1276Read(REG_PACONFIG) & RF_PACONFIG_PASELECT_PABOOST)
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

void SX1276SetTCXO( bool status )
{
    if (IsTCXO != status)
    {
	IsTCXO = status;
	
	if (status)
	{
	    stm32l0_lptim_stop();
	    
	    stm32l0_gpio_pin_configure(RADIO_TCXO_VCC, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_VERY_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
	    stm32l0_gpio_pin_write(RADIO_TCXO_VCC, 1);

	    /* wait for 4ms till TCXO is stable
	     */
	       
	    SX1276Delay( 4 );
	}
	else
	{
	    /* wait for 2ms to turn off TCXO
	     */

	    SX1276Delay( 2 );

	    stm32l0_gpio_pin_configure(RADIO_TCXO_VCC, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
	}
    }
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( power );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
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
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

uint8_t SX1276GetPaSelect( int8_t power )
{
    if( power > 14 )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    if (IsLowPower)
    {
	SX1276SetLowPower( false );
    }

    stm32l0_gpio_pin_write(RADIO_NSS, 0);

    stm32l0_spi_data(&RADIO_SPI, addr | 0x80);
    stm32l0_spi_data(&RADIO_SPI, data);

    stm32l0_gpio_pin_write(RADIO_NSS, 1);
}

uint8_t SX1276Read( uint8_t addr )
{
    uint8_t data;

    if (IsLowPower)
    {
	SX1276SetLowPower( false );
    }

    stm32l0_gpio_pin_write(RADIO_NSS, 0);

    stm32l0_spi_data(&RADIO_SPI, addr & ~0x80);
    data = stm32l0_spi_data(&RADIO_SPI, 0xff);

    stm32l0_gpio_pin_write(RADIO_NSS, 1);

    return data;
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    if (IsLowPower)
    {
	SX1276SetLowPower( false );
    }

    stm32l0_gpio_pin_write(RADIO_NSS, 0);

    stm32l0_spi_data(&RADIO_SPI, addr | 0x80);
    stm32l0_spi_transmit(&RADIO_SPI, buffer, size);

    stm32l0_gpio_pin_write(RADIO_NSS, 1);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    if (IsLowPower)
    {
	SX1276SetLowPower( false );
    }

    stm32l0_gpio_pin_write(RADIO_NSS, 0);

    stm32l0_spi_data(&RADIO_SPI, addr & ~0x80);
    stm32l0_spi_receive(&RADIO_SPI, buffer, size);

    stm32l0_gpio_pin_write(RADIO_NSS, 1);
}

uint32_t SX1276GetRadioWakeUpTime( void )
{
  // return RADIO_WAKEUP_TIME;
  return 4;
}
