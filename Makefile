
CC=gcc
CFLAGS= -lrobotcontrol -lm

includeDir = ./include/
srcDir = ./src/

DEPS= rc_pilot_serial.h deltaarm.h servo.h
OBJ= rc_pilot_serial.o deltaarm.o servo.o main.o 

%.o: %.c $(DEPS)
	@$(CC) -c -o $@ $< $(CFLAGS)
	@echo "Compiled $@"


deltaArm: $(OBJ)
	@$(CC) -o $@ $^ $(CFLAGS)
	@echo "Compiled Binary: $@"

clean:
	@rm -rvf *.o deltaArm