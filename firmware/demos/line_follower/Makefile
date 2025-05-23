LibsPath 		= ../../
LibsCommonPATH 	= $(LibsPath)/libs/LibsCommon
LibsDevicePATH 	= $(LibsPath)/libs/LibsDevice
LibsDriversPATH = $(LibsPath)/libs/LibsDrivers
LibsControlPATH = $(LibsPath)/libs/LibsControl

AppPATH 		= ./

GCCPREFIX	=arm-none-eabi-
OUTPUT	 	=bin/build

TARGET 		= -mcpu=cortex-m7 -mthumb -mfloat-abi=softfp -fsingle-precision-constant -DSTM32F746xx

LDSCRIPT 	= $(LibsDevicePATH)/ldscript.ld

INC+= -include $(LibsDevicePATH)/stm32f7xx_conf.h
INC+= -include $(LibsDevicePATH)/device/stm32f7xx.h
INC+= -I $(LibsCommonPATH)/
INC+= -I $(LibsDevicePATH)/
INC+= -I $(LibsDevicePATH)/device/ 
INC+= -I $(LibsDevicePATH)/CMSIS/ 
INC+= -I $(LibsDevicePATH)/STM32F7xx_StdPeriph_Driver/inc
INC+= -I $(LibsDriversPATH)/
INC+= -I $(LibsControlPATH)/	
INC+= -I $(AppPATH)

C_SRCS = $(shell find  $(LibsCommonPATH) -name 	'*.c')
C_SRCS+= $(shell find  $(LibsDevicePATH) -name 	'*.c')
C_SRCS+= $(shell find  $(LibsDriversPATH) -name '*.c')
C_SRCS+= $(shell find  $(LibsControlPATH) -name '*.c')
C_SRCS+= $(shell find  $(AppPATH) -name '*.c')


CXX_SRCS+= $(shell find $(LibsCommonPATH) -name '*.cpp')
CXX_SRCS+= $(shell find $(LibsDevicePATH) -name '*.cpp')
CXX_SRCS+= $(shell find $(LibsDriversPATH) -name '*.cpp')
CXX_SRCS+= $(shell find $(LibsControlPATH) -name '*.cpp')
CXX_SRCS+= $(shell find $(AppPATH) -name '*.cpp')



CC 		= $(GCCPREFIX)gcc
CXX 	= $(GCCPREFIX)g++
LD 		= $(GCCPREFIX)g++
OBJCOPY = $(GCCPREFIX)objcopy
OBJDUMP = $(GCCPREFIX)objdump
SIZE 	= $(GCCPREFIX)size


LDFLAGS = $(TARGET) -Wl,--gc-sections

CCFLAGS = $(TARGET) $(INC) -c -O2 -Os -s -Wall -Wextra -std=c99    -fdata-sections -ffunction-sections   -Wl,--gc-sections 
CXXFLAGS= $(TARGET) $(INC) -c -O2 -s -funroll-loops -Wall -Wextra -pedantic -std=c++14 -fno-rtti  -fno-exceptions -fdata-sections -ffunction-sections  -Wl,--gc-sections 
LDFLAGS = -T$(LDSCRIPT) $(TARGET)  -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs


C_OBJS 		= $(C_SRCS:.c=.o)
CXX_OBJS 	= $(CXX_SRCS:.cpp=.o)


all: $(OUTPUT)
	$(OBJDUMP) --disassemble $<.elf > $(OUTPUT).asm
	$(OBJCOPY) -O ihex $<.elf $(OUTPUT).hex
	$(OBJCOPY) -O binary $<.elf $(OUTPUT).bin
	$(SIZE) $(OUTPUT).elf

$(OUTPUT): $(C_OBJS) $(CXX_OBJS) 
	$(LD) -o $@.elf $(C_OBJS) $(CXX_OBJS) $(LDFLAGS)


%.o: %.c
	$(CC) $(CCFLAGS) $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@




CLEAN_FILES = $(C_OBJS)
CLEAN_FILES+= $(CXX_OBJS)
CLEAN_FILES+= $(OUTPUT).*


clean:
	rm -f $(CLEAN_FILES)
