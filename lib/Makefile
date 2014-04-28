# MCU name
MCU =  atmega328p

# Processor frequency.
#     This will define a symbol, F_CPU, in all source code files equal to the 
#     processor frequency. You can then use this symbol in your source code to 
#     calculate timings. Do NOT tack on a 'UL' at the end, this will be done
#     automatically to create a 32-bit value in your source code.
F_CPU = 16000000

# Target file name (without extension).
TARGET = libnrf24l01p

# List C source files here. (C dependencies are automatically generated.)
SRC = radio.c spi.c net.c

# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)UL

#---------------- Compiler Options ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = $(CDEFS)
CFLAGS += -O$(OPT)
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -Wall -Wstrict-prototypes
CFLAGS += -Wa,-adhlns=$(<:.c=.lst)
CFLAGS += -std=gnu99
CFLAGS += -flto

#============================================================================


# Define programs and commands.
CC = avr-gcc
AR = avr-ar
REMOVE = rm -f

# Define all object files.
OBJ = $(SRC:.c=.o)

# Define all listing files.
LST = $(SRC:.c=.lst)

# Compiler flags to generate dependency files.
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)

# Default target.
all: $(TARGET).a

# Make an archive
.PRECIOUS : $(OBJ)
%.a: $(OBJ)
	$(AR) rcs $@ $^

# Compile: create object files from C source files.
%.o : %.c
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 


# Target: clean project.
clean: 
	$(REMOVE) $(TARGET).a
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) .dep/*

# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# Listing of phony targets.
.PHONY : all clean



