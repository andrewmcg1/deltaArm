
CC=gcc
CFLAGS= -lrobotcontrol -lm

includeDir = ./include/
srcDir = ./src/

DEPS= rc_pilot_serial.h deltaarm.h servo.h
OBJ= main.o rc_pilot_serial.o deltaarm.o servo.o

%.o: %.c $(DEPS)
	@echo "Compiling $@..."


deltaArm: $(OBJ)
	@echo "Compiling deltaArm..."
	@echo "Compilation done!"

clean:
	rm -f