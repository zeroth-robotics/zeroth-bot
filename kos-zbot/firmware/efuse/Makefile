TARGETS = efuse efuse_read_serial

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

LDFLAGS += -L$(SYSROOT)/lib
LDFLAGS += -L$(SYSROOT)/usr/lib
# LDFLAGS += -L$(TPU_SDK_DIR)/lib

SOURCES = efuse.c efuse_read_serial.c
OBJS = $(patsubst %.c,%.o,$(SOURCES))

all: $(TARGETS)

efuse: efuse.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

efuse_read_serial: efuse_read_serial.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

.PHONY: clean
clean:
	@rm -f *.o
	@rm -f $(OBJS)
	@rm -f $(TARGETS)
