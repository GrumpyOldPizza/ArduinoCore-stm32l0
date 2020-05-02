TOOLS    = /opt/gcc-arm-none-eabi-4.8.3-2014q1
CPP      = $(TOOLS)/bin/arm-none-eabi-cpp
CC       = $(TOOLS)/bin/arm-none-eabi-gcc
CXX      = $(TOOLS)/bin/arm-none-eabi-g++
AS       = $(TOOLS)/bin/arm-none-eabi-as
AR       = $(TOOLS)/bin/arm-none-eabi-ar
LD       = $(TOOLS)/bin/arm-none-eabi-ld
CFLAGS   = -mcpu=cortex-m0plus -mthumb -c -g -Os $(WARNINGS) -std=c99 -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
ASFLAGS  = -c -g -x assembler-with-cpp $(EXTRAS) $(DEFINES) $(INCLUDES)
WARNINGS = -Wall -Wextra -Wno-unused-parameter
EXTRAS   = -march=armv6-m -mthumb -mabi=aapcs -mfloat-abi=soft
DEFINES  =
INCLUDES = \
	-I../../../system/CMSIS/Include \
	-I../../../system/CMSIS/Device/ST/STM32L0xx/Include \
	-I../../../system/STM32L0xx/Source/LoRa/Boards \
	-I../../../system/STM32L0xx/Source/LoRa/Crypto \
	-I../../../system/STM32L0xx/Source/LoRa/Mac \
	-I../../../system/STM32L0xx/Source/LoRa/Mac/region \
	-I../../../system/STM32L0xx/Source/LoRa/Radio \
	-I../../../system/STM32L0xx/Source/LoRa/Radio/sx1272 \
	-I../../../system/STM32L0xx/Source/LoRa/Radio/sx1276 \
	-I../../../system/STM32L0xx/Source/LoRa/System \
	-I../../../system/STM32L0xx/Source/LoRa/Utilities \
	-I../../../system/STM32L0xx/Source/USB/HAL/Inc \
	-I../../../system/STM32L0xx/Source/USB/Core/Inc \
	-I../../../system/STM32L0xx/Source/USB/Class/CDC/Inc \
	-I../../../system/STM32L0xx/Source/USB/Class/MSC/Inc \
	-I../../../system/STM32L0xx/Source/USB/Class/HID/Inc \
	-I../../../system/STM32L0xx/Source/USB \
	-I../../../system/STM32L0xx/Include \
	-I. 

SRCS = \
	./LoRa/Boards/cmwx1zzabz-board.c \
	./LoRa/Boards/sx1272mb2das-board.c \
	./LoRa/Boards/wmsgsm42-board.c \
	./LoRa/Crypto/aes.c \
	./LoRa/Crypto/cmac.c \
	./LoRa/Mac/LoRaMac.c \
	./LoRa/Mac/LoRaMacCrypto.c \
	./LoRa/Mac/region/RegionCommon.c \
	./LoRa/Mac/region/RegionAS923.c \
	./LoRa/Mac/region/RegionAU915.c \
	./LoRa/Mac/region/RegionCN470.c \
	./LoRa/Mac/region/RegionCN779.c \
	./LoRa/Mac/region/RegionEU433.c \
	./LoRa/Mac/region/RegionEU868.c \
	./LoRa/Mac/region/RegionIN865.c \
	./LoRa/Mac/region/RegionKR920.c \
	./LoRa/Mac/region/RegionUS915.c \
	./LoRa/Radio/radio.c \
	./LoRa/Radio/sx1272/sx1272.c \
	./LoRa/Radio/sx1276/sx1276.c \
	./LoRa/System/timer.c \
	./LoRa/Utilities/utilities.c \
	./USB/Class/CDC/Src/usbd_cdc.c \
	./USB/Class/MSC/Src/usbd_msc.c \
	./USB/Class/MSC/Src/usbd_msc_bot.c \
	./USB/Class/MSC/Src/usbd_msc_data.c \
	./USB/Class/MSC/Src/usbd_msc_scsi.c \
	./USB/Class/HID/Src/usbd_hid.c \
	./USB/Core/Src/usbd_core.c \
	./USB/Core/Src/usbd_ctlreq.c \
	./USB/Core/Src/usbd_ioreq.c \
	./USB/HAL/Src/stm32l0xx_hal_pcd.c \
	./USB/HAL/Src/stm32l0xx_hal_pcd_ex.c \
	./USB/usbd_cdc_msc.c \
	./USB/usbd_conf.c \
	./USB/usbd_desc.c \
	armv6m_atomic.S \
	armv6m_core.c \
	armv6m_event.c \
	armv6m_pendsv.c \
	armv6m_svcall.c \
	armv6m_systick.c \
	dosfs_core.c \
	dosfs_device.c \
	dosfs_sflash.c \
	dosfs_storage.c \
	stm32l0_adc.c \
	stm32l0_aes.c \
	stm32l0_comp.c \
	stm32l0_dac.c \
	stm32l0_dma.c \
	stm32l0_eeprom.c \
	stm32l0_exti.c \
	stm32l0_flash.c \
	stm32l0_gpio.c \
	stm32l0_i2c.c \
	stm32l0_iwdg.c \
	stm32l0_lptim.c \
	stm32l0_random.c \
	stm32l0_rtc.c \
	stm32l0_sdspi.c \
	stm32l0_servo.c \
	stm32l0_sfspi.c \
	stm32l0_spi.c \
	stm32l0_system.c \
	stm32l0_timer.c \
	stm32l0_uart.c \
	stm32l0_usbd_cdc.c \
	stm32l0_usbd_hid.c

SRCS_L052 = $(SRCS) startup_stm32l052xx.S
OBJS_L052 = $(patsubst %.c,_out/stm32l052xx/%.o,$(filter-out %.S,$(SRCS_L052))) $(patsubst %.S,_out/stm32l052xx/%.o,$(filter-out %.c,$(SRCS_L052)))

SRCS_L072 = $(SRCS) startup_stm32l072xx.S
OBJS_L072 = $(patsubst %.c,_out/stm32l072xx/%.o,$(filter-out %.S,$(SRCS_L072))) $(patsubst %.S,_out/stm32l072xx/%.o,$(filter-out %.c,$(SRCS_L072)))

SRCS_L082 = $(SRCS) startup_stm32l082xx.S
OBJS_L082 = $(patsubst %.c,_out/stm32l082xx/%.o,$(filter-out %.S,$(SRCS_L082))) $(patsubst %.S,_out/stm32l082xx/%.o,$(filter-out %.c,$(SRCS_L082)))

all:: libstm32l052xx.a libstm32l072xx.a libstm32l082xx.a

libstm32l052xx.a:: $(OBJS_L052)
	$(AR) rcs $@ $^

_out/stm32l052xx/%.o: %.c
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32L052xx $(CFLAGS) -c $< -o $@

_out/stm32l052xx/%.o: %.S
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32L052xx $(ASFLAGS) -c $< -o $@

libstm32l072xx.a:: $(OBJS_L072)
	$(AR) rcs $@ $^

_out/stm32l072xx/%.o: %.c
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32L072xx $(CFLAGS) -c $< -o $@

_out/stm32l072xx/%.o: %.S
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32L072xx $(ASFLAGS) -c $< -o $@

libstm32l082xx.a:: $(OBJS_L082)
	$(AR) rcs $@ $^

_out/stm32l082xx/%.o: %.c
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32L082xx $(CFLAGS) -c $< -o $@

_out/stm32l082xx/%.o: %.S
	-@mkdir -p $(dir $@)
	$(CC) -DSTM32L082xx $(ASFLAGS) -c $< -o $@

clean::
	rm -rf libstm32l052xx.a libstm32l072xx.a libstm32l082xx.a _out

-include $(OBJS_L052:.o=.d)
-include $(OBJS_L072:.o=.d)
-include $(OBJS_L082:.o=.d)

