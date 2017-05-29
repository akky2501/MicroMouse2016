SHELL = /bin/sh
TARGET_ARCH   = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mthumb-interwork
INCLUDE_DIRS  = -I ../stm32plus/lib/include \
				-I ../stm32plus/lib/include/stl \
				-I ../stm32plus/lib/fwlib/f4/stdperiph/inc \
				-I ../stm32plus/lib/fwlib/f4/cmsis/Device/ST/STM32F4xx/Include \
				-I ../stm32plus/lib
STARTUP_DIR = ./system/f407_168_8/
BOARD_OPTS = -DHSI_VALUE=\(\(uint32_t\)16000000\)
FIRMWARE_OPTS = -DSTM32PLUS_F405 -DSTM32PLUS_BUILD -DARM_MATH_CM4
COMPILE_OPTS  = -MMD -std=c++11 -O0 -g3 -ffunction-sections -fpermissive -fdata-sections -fsigned-char -fno-rtti -fno-exceptions -Wall -fmessage-length=0 -pipe -fno-threadsafe-statics -Wno-misleading-indentation $(INCLUDE_DIRS) $(BOARD_OPTS) $(FIRMWARE_OPTS)
TARGETS = $(patsubst %.c,%.o,$(wildcard *.c)) \
		  $(patsubst %.cpp,%.o,$(wildcard *.cpp)) \
		  $(STARTUP_DIR)Startup.o \
		  $(STARTUP_DIR)System.o \
		  $(STARTUP_DIR)../LibraryHacks.o
DEPENDS = $(patsubst %cpp,%.d,$(wildcard *.cpp)) \
          $(patsubst %c,%.d,$(wildcard *.c))

CC      = arm-none-eabi-g++
CXX     = $(CC)
AS      = $(CC)
LD      = $(CC)
AR      = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
CFLAGS  = $(COMPILE_OPTS)
CXXFLAGS= $(COMPILE_OPTS) -include stdint.h
ASFLAGS = -x assembler-with-cpp -c $(TARGET_ARCH) $(COMPILE_OPTS) 
LDFLAGS = -Wl,--gc-sections,-Map=bin/main.map,-cref -T $(STARTUP_DIR)Linker.ld $(INCLUDE_DIRS) -lstm32plus-debug-f405-16000000i-hard -L ../stm32plus/lib/build/debug-f405-16000000i-hard

.PHONY: all

all: bin/main.hex

bin/main.hex: $(TARGETS) ../stm32plus/lib/build/debug-f405-16000000i-hard/libstm32plus-debug-f405-16000000i-hard.a
	$(LD) $(LDFLAGS) $(TARGET_ARCH) $^ -o bin/main.elf 
	$(OBJCOPY) -O ihex bin/main.elf bin/main.hex

$(STARTUP_DIR)Startup.o:
	$(AS) -o $(STARTUP_DIR)Startup.o $(ASFLAGS) $(STARTUP_DIR)Startup.asm

clean:
	rm -rf *.o *.s bin/* system/*.o $(STARTUP_DIR)*.o
	rm -rf *.d *.s system/*.d $(STARTUP_DIR)*.d

-include $(DEPENDS)
