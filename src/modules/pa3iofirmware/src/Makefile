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
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/timer.o -c $(PATH_HARDWARE)/timer.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/pwmout.o -c $(PATH_HARDWARE)/pwmout.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/led.o -c $(PATH_HARDWARE)/led.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/uart1.o -c $(PATH_HARDWARE)/uart1.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/uart2.o -c $(PATH_HARDWARE)/uart2.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/sbus.o -c $(PATH_HARDWARE)/sbus.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/battery.o -c $(PATH_HARDWARE)/battery.c
	$(CC) $(CFLAGSlib) -o $(PATH_TARGET)/protocol.o -c $(PATH_HARDWARE)/protocol.c

app.a:
	$(AR) cr $(PATH_TARGET)/app.a $(PATH_TARGET)/*.o
	
clean:
	rm -f $(PATH_TARGET)/*
	
