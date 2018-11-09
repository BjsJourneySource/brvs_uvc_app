CROSS_COMPILE=arm-buildroot-linux-uclibcgnueabi-
CC=$(CROSS_COMPILE)gcc
AS=$(CROSS_COMPILE)as
LD=$(CROSS_COMPILE)gcc
STRIP=$(CROSS_COMPILE)strip

CFLAGS = -g 
CFLAGS += -I/usr/src/linux-$(shell uname -r)/include/generated/uapi -I/usr/src/linux-$(shell uname -r)/include/uapi
CFLAGS += -I/usr/src/linux-$(shell uname -r)/include
CFLAGS += -lpthread

OBJS = brvs_uvc_app.o

all: brvs_uvc_app

brvs_uvc_app: $(OBJS)
	@$(CC) $(CFLAGS) $(OBJS) -o $@

brvs_uvc_app.o: brvs_uvc_app.c
	@$(CC) $(CFLAGS) -c -o $@ $<

clean:
	-rm -f *.o brvs_uvc_app


