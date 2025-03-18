TARGET = gpt_test
MCU = cortex-m7
FPU = -mfloat-abi=hard -mfpu=fpv5-d16
LDSCRIPT = ram.ld
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS = -mcpu=$(MCU) $(FPU) -mthumb -O0 -nostdlib -ffreestanding -g
LDFLAGS = -T$(LDSCRIPT) -nostartfiles -Wl,--gc-sections -Wl,-Map=$(TARGET).map


SRCS = main.c
OBJS = $(SRCS:.c=.o)

all: $(TARGET).elf $(TARGET).bin

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f $(TARGET).elf $(TARGET).bin $(OBJS)
