/*!
 * \file      s76x-board.c
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
#include "stm32l0_rtc.h"

#if defined(STM32L072xx)

#define RADIO_RESET                          STM32L0_GPIO_PIN_PB10

#define RADIO_MOSI                           STM32L0_GPIO_PIN_PB15_SPI2_MOSI
#define RADIO_MISO                           STM32L0_GPIO_PIN_PB14_SPI2_MISO
#define RADIO_SCLK                           STM32L0_GPIO_PIN_PB13_SPI2_SCK
#define RADIO_NSS                            STM32L0_GPIO_PIN_PB12_SPI2_NSS

#define RADIO_DIO_0                          STM32L0_GPIO_PIN_PB11
#define RADIO_DIO_1                          STM32L0_GPIO_PIN_PC13
#define RADIO_DIO_2                          STM32L0_GPIO_PIN_PB9

#define RADIO_ANT_SWITCH_RX                  STM32L0_GPIO_PIN_PA1

#define BOARD_TCXO_WAKEUP_TIME               5

static const stm32l0_spi_params_t RADIO_SPI_PARAMS = {
    STM32L0_SPI_INSTANCE_SPI2,
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

static uint8_t RADIO_TCXO_VCC;

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
    // Do not enable TCXO input for S76S/S78S
    if (RADIO_TCXO_VCC != STM32L0_GPIO_PIN_NONE)
    {
        SX1276Write( REG_TCXO, ( SX1276Read( REG_TCXO ) & RF_TCXO_TCXOINPUT_MASK ) | RF_TCXO_TCXOINPUT_ON );
    } else {
        SX1276Write( REG_TCXO, ( SX1276Read( REG_TCXO ) & RF_TCXO_TCXOINPUT_MASK ) );
    }
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
    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_RX, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_LOW | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_RX, 1);
}

void SX1276AntSwDeInit( void )
{
    stm32l0_gpio_pin_configure(RADIO_ANT_SWITCH_RX, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));
}

void SX1276SetAntSw( uint8_t opMode, int8_t power )
{
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        stm32l0_gpio_pin_write(RADIO_ANT_SWITCH_RX, 0);
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
    if( power > 20 )
    {
        power = 20;
    }

    if( power > 15 )
    {
        if( power > 17 )
        {
            paConfig = ( RF_PACONFIG_PASELECT_PABOOST | ( power - 5 ) );
            paDac = RF_PADAC_20DBM_ON;
        }
        else
        {
            paConfig = ( RF_PACONFIG_PASELECT_PABOOST | ( power - 2 ) );
            paDac = RF_PADAC_20DBM_OFF;
        }
    }
    else
    {
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
    }

    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, ( ( SX1276Read( REG_PADAC ) & RF_PADAC_20DBM_MASK ) | paDac ) );
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

void S76x_Initialize( uint8_t pin_tcxo )
{
    uint32_t tim3_start, tim3_end, tim3_count, tim3_capture, tim3_ccr4;
    uint32_t tim21_start, tim21_end, tim21_count, tim21_capture, tim21_ccr1;
    uint32_t datarate, primask;

    RADIO_TCXO_VCC = pin_tcxo;

    stm32l0_gpio_pin_configure(RADIO_NSS, (STM32L0_GPIO_PARK_HIZ | STM32L0_GPIO_PUPD_NONE | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_OUTPUT));
    stm32l0_gpio_pin_write(RADIO_NSS, 1);

    stm32l0_spi_create(&RADIO_SPI, &RADIO_SPI_PARAMS);
    stm32l0_spi_enable(&RADIO_SPI);

    SX1276Reset( );

    if (RADIO_TCXO_VCC != STM32L0_GPIO_PIN_NONE)
    {
        SX1276SetBoardTcxo( true );

        SX1276Delay( BOARD_TCXO_WAKEUP_TIME );
    }

    datarate = ( 16 * XTAL_FREQ ) / 2048;
    SX1276Write( REG_BITRATEMSB,  ( uint8_t )( datarate >> 12 ) );
    SX1276Write( REG_BITRATELSB,  ( uint8_t )( datarate >> 4  ) );
    SX1276Write( REG_BITRATEFRAC, ( uint8_t )( datarate >> 0  ) );

    SX1276Write( REG_PACONFIG, 0x00 );
    SX1276Write( REG_PACKETCONFIG2, ( SX1276Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );
    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_TRANSMITTER );

    // Wait 25 ms
    SX1276Delay( 25 );

    stm32l0_gpio_pin_configure(RADIO_DIO_1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_PUPD_PULLDOWN | STM32L0_GPIO_OSPEED_HIGH | STM32L0_GPIO_OTYPE_PUSHPULL | STM32L0_GPIO_MODE_ALTERNATE));
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB1ENR;

    TIM3->CR1   = TIM_CR1_URS;
    TIM3->CR2   = 0;
    TIM3->DIER  = 0;
    TIM3->PSC   = 0;
    TIM3->ARR   = 0xffff;
    TIM3->EGR   = TIM_EGR_UG;
    
    TIM3->CCER  = 0;
    TIM3->CCMR2 = TIM_CCMR2_CC4S_0 | TIM_CCMR2_IC4F_0;
    TIM3->CCER  = TIM_CCER_CC4E;

    tim3_start   = 0;
    tim3_end     = 0;
    tim3_capture = 0;
    tim3_count   = 0;

    RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
    RCC->APB2ENR;

    TIM21->CR1   = TIM_CR1_URS;
    TIM21->CR2   = 0;
    TIM21->DIER  = 0;
    TIM21->PSC   = 0;
    TIM21->ARR   = 0xffff;
    TIM21->OR    = TIM21_OR_TI1_RMP_2; /* Select LSE as TI1 */
    TIM21->EGR   = TIM_EGR_UG;
    
    TIM21->CCER  = 0;
    TIM21->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1PSC_1 | TIM_CCMR1_IC1PSC_0; /* Count every 8th pulse */
    TIM21->CCER  = TIM_CCER_CC1E;

    tim21_start   = 0;
    tim21_end     = 0;
    tim21_capture = 0;
    tim21_count   = 0;

    primask = __get_PRIMASK();

    __disable_irq();

    TIM3->CR1 |= TIM_CR1_CEN;
    TIM3->SR = 0;

    TIM21->CR1 |= TIM_CR1_CEN;
    TIM21->SR = 0;

    do
    {
        if (TIM3->SR & TIM_SR_CC4IF)
        {
            TIM3->SR = ~TIM_SR_CC4IF;

            tim3_ccr4 = TIM3->CCR4 & 0xffff;

            if (tim3_ccr4 < (tim3_capture & 0x0000ffff))
            {
                tim3_capture = ((tim3_capture + 0x00010000) & 0xffff0000) | tim3_ccr4;
            }
            else
            {
                tim3_capture = (tim3_capture & 0xffff0000) | tim3_ccr4;
            }
            
            if (tim3_count == 0)
            {
                tim3_start = tim3_capture;
            }

            if (tim3_count <= 256)
            {
                tim3_end = tim3_capture;
                tim3_count++;
            }
        }

        if (TIM21->SR & TIM_SR_CC1IF)
        {
            TIM21->SR = ~TIM_SR_CC1IF;

            tim21_ccr1 = TIM21->CCR1 & 0xffff;

            if (tim21_ccr1 < (tim21_capture & 0x0000ffff))
            {
                tim21_capture = ((tim21_capture + 0x00010000) & 0xffff0000) | tim21_ccr1;
            }
            else
            {
                tim21_capture = (tim21_capture & 0xffff0000) | tim21_ccr1;
            }
            
            if (tim21_count == 0)
            {
                tim21_start = tim21_capture;
            }

            if (tim21_count <= 512)
            {
                tim21_end = tim21_capture;
                tim21_count++;
            }
        }
    }
    while ((tim3_count <= 256) || (tim21_count <= 512));

    TIM3->CR1 = 0;
    TIM21->CR1 = 0;

    __set_PRIMASK(primask);

    stm32l0_gpio_pin_configure(RADIO_DIO_1, (STM32L0_GPIO_PARK_NONE | STM32L0_GPIO_MODE_ANALOG));

    RCC->APB2RSTR |= RCC_APB2RSTR_TIM21RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM21RST;
    RCC->APB2ENR &= ~RCC_APB2ENR_TIM21EN;

    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
    RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;

    SX1276Write( REG_OPMODE, ( SX1276Read( REG_OPMODE ) & RF_OPMODE_MASK ) | RF_OPMODE_SLEEP );

    SX1276Release( );

    if (RADIO_TCXO_VCC != STM32L0_GPIO_PIN_NONE)
    {
        SX1276Delay( 1 );

        SX1276SetBoardTcxo( false );
    }

    if (tim3_start != tim3_end)
    {
        stm32l0_rtc_set_calibration( ((uint32_t)(((uint64_t)(tim21_end - tim21_start) << 20) / (uint32_t)(tim3_end - tim3_start)) - (1 << 20)) );
    }
}

#endif /* defined(STM32L072xx) */
