WRAPPER=libfeetech.so

ifeq (,$(TOOLCHAIN_PREFIX))
$(error TOOLCHAIN_PREFIX is not set)
endif

ifeq (,$(CFLAGS))
$(error CFLAGS is not set)
endif

ifeq (,$(LDFLAGS))
$(error LDFLAGS is not set)
endif

CC = $(TOOLCHAIN_PREFIX)gcc

CFLAGS += -I$(SYSROOT)/usr/include
CFLAGS += -I$(TPU_SDK_DIR)/include
CFLAGS += -fPIC

LDFLAGS += -L$(SYSROOT)/lib
LDFLAGS += -L$(SYSROOT)/usr/lib
LDFLAGS += -shared

SOURCE = $(wildcard *.c)
OBJS = $(patsubst %.c,%.o,$(SOURCE))

all: $(WRAPPER)

$(WRAPPER): feetech.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@rm *.o -rf
	@rm $(OBJS) -rf
	@rm $(TARGET) $(WRAPPER)

