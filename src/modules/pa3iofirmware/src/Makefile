# src Makefile
include ../Makefile.common

PATH_HARDWARE=hardware
PATH_MAIN=main
CFLAGSlib+=-Iinclude

all: src

src: mkdir objs app.a

mkdir:
	mkdir -p release

objs:
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/main.o -c $(PATH_MAIN)/main.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/startup.o -c $(PATH_MAIN)/startup.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/timer_tick.o -c $(PATH_HARDWARE)/timer.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/pwm_out.o -c $(PATH_HARDWARE)/pwmout.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/led_light.o -c $(PATH_HARDWARE)/led.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/serial_port.o -c $(PATH_HARDWARE)/uart1.c

app.a:
	$(AR) cr $(PATH_TARGET)/app.a $(PATH_TARGET)/*.o
	
clean:
	rm -f $(PATH_TARGET)/*
	
