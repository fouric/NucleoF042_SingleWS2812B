# IMPORTANT!!!
# You must configure the LIBSPEC variable for your own system.  The setting below reflects
# where the Cortex M0 libraries from gcc are installed on *my* system.  *Your* system
# will be different
# Tell the linker where to find the libraries -> important: use thumb versions
# email me if this causes a problem!
ARM_TOOLCHAIN_VERSION=$(shell ls -1 /usr/lib/gcc/arm-none-eabi | head -n 1)
LIBSPEC=-L /usr/lib/gcc/arm-none-eabi/$(ARM_TOOLCHAIN_VERSION)/armv6-m

# Specify the compiler to use
CC=arm-none-eabi-gcc
# Specify the assembler to use
AS=arm-none-eabi-as
# Specity the linker to use
LD=arm-none-eabi-ld

CCFLAGS=-mcpu=cortex-m0 -mthumb -g -Wall

# List the object files involved in this project
OBJS=	init.o \
	serial.o \
	spi.o \
	main.o \
  config.o

# The default 'target' (output) is main.elf and it depends on the object files being there.
# These object files are linked together to create main.elf
main.elf : $(OBJS)
	$(LD) $(OBJS) $(LIBSPEC) -lgcc -T linker_script.ld --cref -Map main.map -nostartfiles -o main.elf
# convert binary elf file to hex to help the stm32flash utility
	objcopy -O ihex main.elf main.hex
# The object file main.o depends on main.c.  main.c is compiled to make main.o
main.o: main.c
	$(CC) -c $(CCFLAGS) main.c -o main.o

init.o: init.c
	$(CC) -c $(CCFLAGS) init.c -o init.o

serial.o: serial.c
	$(CC) -c $(CCFLAGS) serial.c -o serial.o

spi.o: spi.c
	$(CC) -c $(CCFLAGS) spi.c -o spi.o

config.o: config.c
	$(CC) -c $(CCFLAGS) config.c -o config.o

# if someone types in 'make clean' then remove all object files and executables
# associated wit this project
clean:
	rm -f $(OBJS)
	rm -f main.elf
