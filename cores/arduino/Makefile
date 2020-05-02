TOOLS    = /opt/gcc-arm-none-eabi-4.8.3-2014q1
CPP      = $(TOOLS)/bin/arm-none-eabi-cpp
CC       = $(TOOLS)/bin/arm-none-eabi-gcc
CXX      = $(TOOLS)/bin/arm-none-eabi-g++
AS       = $(TOOLS)/bin/arm-none-eabi-as
AR       = $(TOOLS)/bin/arm-none-eabi-ar
LD       = $(TOOLS)/bin/arm-none-eabi-ld
CFLAGS   = -mcpu=cortex-m0plus -mthumb -c -g -Os $(WARNINGS) -std=gnu11 -ffunction-sections -fdata-sections -nostdlib -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
CXXFLAGS = -mcpu=cortex-m0plus -mthumb -c -g -Os $(WARNINGS) -std=gnu++11 -ffunction-sections -fdata-sections -fno-threadsafe-statics -nostdlib -fno-rtti -fno-exceptions -MMD $(EXTRAS) $(DEFINES) $(INCLUDES)
ASFLAGS  = -c -g -x assembler-with-cpp $(EXTRAS) $(DEFINES) $(INCLUDES)
LDFLAGS = -g -Os -mcpu=cortex-m0plus -mthumb -mabi=aapcs -mfloat-abi=soft -Wl,--gc-sections,--no-undefined -T../../variants/Cricket-L082CZ/linker_scripts/STM32L082CZ_FLASH.ld --specs=nano.specs
WARNINGS = -Wall -Wextra -Wno-unused-parameter
EXTRAS   = -DSTM32L082xx -march=armv6-m -mthumb -mabi=aapcs -mfloat-abi=soft
DEFINES  = -D_SYSTEM_CORE_CLOCK_=32000000 -DDOSFS_SDCARD=0 -DDOSFS_SFLASH=1 -DARDUINO_MAKEFILE
INCLUDES = \
	-I../../system/CMSIS/Include \
	-I../../system/CMSIS/Device/ST/STM32L0xx/Include \
	-I../../system/STM32L0xx/Include \
	-I../../variants/Cricket-L082CZ \
	-I../../libraries/CayenneLPP/src \
	-I../../libraries/DOSFS/src \
	-I../../libraries/EEPROM/src \
	-I../../libraries/FskRadio/src \
	-I../../libraries/GNSS/src \
	-I../../libraries/GNSS/src/utility \
	-I../../libraries/LoRaRadio/src \
	-I../../libraries/LoRaWAN/src \
	-I../../libraries/RTC/src \
	-I../../libraries/SPI/src \
	-I../../libraries/STM32L0/src \
	-I../../libraries/TimerMillis/src \
	-I../../libraries/Wire/src \
	-I.
SRCS = \
	../../libraries/CayenneLPP/src/CayenneLPP.cpp \
	../../libraries/DOSFS/src/DOSFS.cpp \
	../../libraries/FskRadio/src/FskRadio.cpp \
	../../libraries/GNSS/src/GNSS.cpp \
	../../libraries/GNSS/src/utility/gnss_core.c \
	../../libraries/LoRaRadio/src/LoRaRadio.cpp \
	../../libraries/LoRaWAN/src/LoRaWAN.cpp \
	../../libraries/RTC/src/RTC.cpp \
	../../libraries/SPI/src/SPI.cpp \
	../../libraries/STM32L0/src/STM32L0.cpp \
	../../libraries/TimerMillis/src/TimerMillis.cpp \
	../../libraries/Wire/src/Wire.cpp \
	../../variants/Cricket-L082CZ/variant.cpp \
	avr/dtostrf.c \
	avr/eeprom.c \
	avr/fdevopen.c \
	Callback.cpp \
	CDC.cpp \
	FS.cpp \
	HardwareSerial.cpp \
	HardwareSerial1.cpp \
	HardwareSerial2.cpp \
	HardwareSerial3.cpp \
	HID.cpp \
	IPAddress.cpp \
	Print.cpp \
	RingBuffer.cpp \
	Stream.cpp \
	Tone.cpp \
	Uart.cpp \
	USBCore.cpp \
	WInterrupts.c \
	WMath.cpp \
	WString.cpp \
	abi.cpp \
	delay.c \
	hooks.c \
	itoa.c \
	main.c \
	new.cpp \
	syscalls_stm32l0.c \
	wiring.c \
	wiring_analog.c \
	wiring_digital.c \
	wiring_pulse.c \
	wiring_shift.c \

OBJS = \
	../../libraries/CayenneLPP/src/CayenneLPP.o \
	../../libraries/DOSFS/src/DOSFS.o \
	../../libraries/FskRadio/src/FskRadio.o \
	../../libraries/GNSS/src/GNSS.o \
	../../libraries/GNSS/src/utility/gnss_core.o \
	../../libraries/LoRaRadio/src/LoRaRadio.o \
	../../libraries/LoRaWAN/src/LoRaWAN.o \
	../../libraries/RTC/src/RTC.o \
	../../libraries/SPI/src/SPI.o \
	../../libraries/STM32L0/src/STM32L0.o \
	../../libraries/TimerMillis/src/TimerMillis.o \
	../../libraries/Wire/src/Wire.o \
	../../variants/Cricket-L082CZ/variant.o \
	avr/dtostrf.o \
	avr/eeprom.o \
	avr/fdevopen.o \
	Callback.o \
	CDC.o \
	FS.o \
	HardwareSerial.o \
	HardwareSerial1.o \
	HardwareSerial2.o \
	HardwareSerial3.o \
	HID.o \
	IPAddress.o \
	Print.o \
	RingBuffer.o \
	Stream.o \
	Tone.o \
	Uart.o \
	USBCore.o \
	WInterrupts.o \
	WMath.o \
	WString.o \
	abi.o \
	delay.o \
	hooks.o \
	itoa.o \
	main.o \
	new.o \
	syscalls_stm32l0.o \
	wiring.o \
	wiring_analog.o \
	wiring_digital.o \
	wiring_pulse.o \
	wiring_shift.o \

all:: flash.bin

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(ASFLAGS) -c $< -o $@

flash.elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -L../../system/STM32L0xx/Lib -lstm32l082xx -lm -lc -lgcc -lstm32l082xx -o flash.elf

flash.bin:: flash.elf
	arm-none-eabi-objcopy -O binary flash.elf flash.bin

clean::
	rm -f flash.bin flash.elf *~ *.o *.d $(OBJS)

-include $(OBJS:.o=.d)
