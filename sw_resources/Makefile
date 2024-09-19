SW_RESOURCES=../sw_resources

# Compiler configurations
MCU=atxmega16a4u
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) -Os
LDFLAGS=-Wl,-gc-sections -Wl,-relax
CC=avr-gcc

# Name of the target executable
TARGET=main

# Source files in this directory
SRCS=$(wildcard *.c)

# Source files in the sw_resources directory
SRCS_COMMON=$(wildcard $(SW_RESOURCES)/*.c)

# Header files
HEADERS=$(wildcard *.h) 
HEADERS_COMMON=$(wildcard $(SW_RESOURCES)/*.h)

# Object files that need to be compiled (destination is current directory, so remove leading path on common files)
OBJECT_FILES=$(SRCS:.c=.o) $(patsubst $(SW_RESOURCES)/%.c, %.o, $(SRCS_COMMON))


all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.elf

# Compile source files in this directory
%.o: ./%.c $(HEADERS) $(HEADERS_COMMON)
	$(CC) $(CFLAGS) -c $< -I$(SW_RESOURCES)

# Compile source files in the sw_resources directory
%.o: $(SW_RESOURCES)/%.c $(HEADERS_COMMON)
	$(CC) $(CFLAGS) -c $< -I$(SW_RESOURCES)

# Link object files to ELF file
$(TARGET).elf: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

# Convert ELF file to HEX file
$(TARGET).hex: $(TARGET).elf
	avr-objcopy -R .eeprom -O ihex $< $@

program: $(TARGET).hex
	avrdude -v -c flip2 -p $(MCU) -U application:w:$(TARGET).hex:i
