################################
# stm32 uart example Makefile
################################
 
PROGRAM = raspilot
 
ENTRY_FILE = startup
 
ENTRY_CFILE = $(ENTRY_FILE).c
ENTRY_OFILE = $(ENTRY_FILE).o
 
STDLIB_DIR = /home/pi/arminarm/src/CMSIS_StdPeriph/common

RCC_CFILE = $(STDLIB_DIR)/STM32F10x_StdPeriph_Driver_v3.6.1/src/stm32f10x_rcc.c
RCC_OFILE = stm32f10x_rcc.o
 
GPIO_CFILE = $(STDLIB_DIR)/STM32F10x_StdPeriph_Driver_v3.6.1/src/stm32f10x_gpio.c
GPIO_OFILE = stm32f10x_gpio.o
 
USART_CFILE = $(STDLIB_DIR)/STM32F10x_StdPeriph_Driver_v3.6.1/src/stm32f10x_usart.c
USART_OFILE = stm32f10x_usart.o

TIM_CFILE = $(STDLIB_DIR)/STM32F10x_StdPeriph_Driver_v3.6.1/src/stm32f10x_tim.c
TIM_OFILE = stm32f10x_tim.o
 
ALL_OFILES = main.o $(ENTRY_OFILE) $(RCC_OFILE) $(GPIO_OFILE) $(USART_OFILE) $(TIM_OFILE)
 
TARGET_BIN = $(PROGRAM).bin
TARGET_ELF = $(PROGRAM).elf
 
CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
CP = arm-none-eabi-objcopy
 
FAMILY = STM32F10X_MD
 
DEFINES = -D$(FAMILY)\
          -DUSE_STDPERIPH_DRIVER
           
INCLUDES = -I$(STDLIB_DIR)/ \
			-I$(STDLIB_DIR)/CMSIS_v3.6.1/Include/ \
			-I$(STDLIB_DIR)/CMSIS_v3.6.1/Device/ST/STM32F10x/Include/ \
			-I$(STDLIB_DIR)/STM32F10x_StdPeriph_Driver_v3.6.1/inc/
 
CFLAGS  = -c -march=armv7-m -mcpu=cortex-m3 -mthumb \
          -fno-common -nostdlib -fno-builtin -ffreestanding \
          -Wall -O0 -g  \
          $(DEFINES)
           
LFLAGS  = -nostartfiles -Tlink.lds
CPFLAGS = -Obinary
 
.PHONY: all clean write
 
all: $(TARGET_BIN)
 
$(TARGET_BIN): $(TARGET_ELF)
	$(CP) $(CPFLAGS) $< $@
     
$(TARGET_ELF): $(ALL_OFILES)
	$(LD) $(LFLAGS) $(ALL_OFILES) -o $@ 
     
main.o: main.c
	$(CC) $(INCLUDES) $(CFLAGS) main.c -o $@
 
$(ENTRY_OFILE): $(ENTRY_CFILE)
	$(CC) $(INCLUDES) $(CFLAGS) $(ENTRY_CFILE) -o $@
     
$(RCC_OFILE): $(RCC_CFILE)
	$(CC) $(INCLUDES) $(CFLAGS) $(RCC_CFILE) -o $@
 
$(GPIO_OFILE): $(GPIO_CFILE)
	$(CC) $(INCLUDES) $(CFLAGS) $(GPIO_CFILE) -o $@
 
$(USART_OFILE): $(USART_CFILE)
	$(CC) $(INCLUDES) $(CFLAGS) $(USART_CFILE) -o $@
     
$(TIM_OFILE): $(TIM_CFILE)
	$(CC) $(INCLUDES) $(CFLAGS) $(TIM_CFILE) -o $@

clean:
	rm -rf *.o *.elf *.bin
 
write: 
	stm32loader.py -ewv -p /dev/ttyAMA0 $(PROGRAM).bin
