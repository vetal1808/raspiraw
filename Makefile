CROSS_COMPILE ?=

CC	?= gcc
CC	:= $(CROSS_COMPILE)$(CC)
CFLAGS	?= -I/opt/vc/include -pipe -W -Wall -Wextra -g -O0 -MD
LDFLAGS	?=
LIBS	:= -L/opt/vc/lib -lrt -lbcm_host -lvcos -lmmal_core -lmmal_util -lmmal_vc_client -lvcsm -lpthread

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: raspiraw

OBJS := raspiraw.o RaspiCLI.o
raspiraw: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o *.d
	-rm -f raspiraw
	-rm -rf $(TEST_DIR)

# ---------------------------------------------------------------------------
# Tests on a mock of the firmware libraries and of the sensor (tools/test).
# They need only the VideoCore headers, so they run on any Linux machine:
#
#   make test                              headers in /opt/vc/include
#   make test USERLAND=/path/to/userland   headers from a checkout of
#                                          https://github.com/raspberrypi/userland
#   make test-diff BASELINE=<git rev>      compare behaviour with an older revision
# ---------------------------------------------------------------------------
USERLAND ?=
TEST_DIR := build-test
TEST_SAN ?= -fsanitize=address,undefined -fno-omit-frame-pointer
TEST_CFLAGS := -g -O0 -w $(TEST_SAN)

ifeq ($(USERLAND),)
TEST_INC := -I/opt/vc/include -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux
TEST_VCSM_H := /opt/vc/include/interface/vcsm/user-vcsm.h
else
# A source checkout keeps user-vcsm.h elsewhere than the installed headers do
TEST_INC := -I$(TEST_DIR)/inc -I$(USERLAND) -I$(USERLAND)/host_applications/linux/libs/bcm_host/include \
	    -I$(USERLAND)/interface/vcos/pthreads -I$(USERLAND)/interface/vmcs_host/linux
TEST_VCSM_H := $(TEST_DIR)/inc/interface/vcsm/user-vcsm.h

$(TEST_VCSM_H): $(USERLAND)/host_applications/linux/libs/sm/user-vcsm.h
	mkdir -p $(dir $@)
	cp $< $@
endif

TEST_SRCS := raspiraw.c RaspiCLI.c RaspiCLI.h raw_header.h $(wildcard *_modes.h)

$(TEST_DIR)/mock_mmal.o: tools/test/mock_mmal.c $(TEST_VCSM_H)
	mkdir -p $(TEST_DIR)
	$(CC) $(TEST_INC) $(TEST_CFLAGS) -c -o $@ $<

$(TEST_DIR)/raspiraw_mock: $(TEST_SRCS) $(TEST_DIR)/mock_mmal.o
	$(CC) $(TEST_INC) $(TEST_CFLAGS) -o $@ raspiraw.c RaspiCLI.c $(TEST_DIR)/mock_mmal.o -lpthread -lm

# raspiraw.c with main() renamed, so that its functions can be called from a test
$(TEST_DIR)/pixtest: tools/test/pixtest.c $(TEST_SRCS) $(TEST_DIR)/mock_mmal.o
	$(CC) $(TEST_INC) $(TEST_CFLAGS) -Dmain=raspiraw_main -c -o $(TEST_DIR)/raspiraw_nomain.o raspiraw.c
	$(CC) $(TEST_INC) $(TEST_CFLAGS) -o $@ tools/test/pixtest.c $(TEST_DIR)/raspiraw_nomain.o RaspiCLI.c \
		$(TEST_DIR)/mock_mmal.o -lpthread -lm

test: $(TEST_DIR)/raspiraw_mock $(TEST_DIR)/pixtest
	python3 tools/test/run_tests.py --raspiraw $(TEST_DIR)/raspiraw_mock --pixtest $(TEST_DIR)/pixtest

# The baseline revision is built with today's mock
test-diff: $(TEST_DIR)/raspiraw_mock
	@test -n "$(BASELINE)" || { echo "usage: make test-diff BASELINE=<git rev>"; exit 2; }
	rm -rf $(TEST_DIR)/baseline && mkdir -p $(TEST_DIR)/baseline
	git archive $(BASELINE) | tar -x -C $(TEST_DIR)/baseline
	cd $(TEST_DIR)/baseline && $(CC) $(patsubst -I$(TEST_DIR)/%,-I$(CURDIR)/$(TEST_DIR)/%,$(TEST_INC)) $(TEST_CFLAGS) \
		-o ../raspiraw_baseline raspiraw.c RaspiCLI.c ../mock_mmal.o -lpthread -lm
	python3 tools/test/difftest.py $(TEST_DIR)/raspiraw_baseline $(TEST_DIR)/raspiraw_mock

.PHONY: all clean test test-diff

-include $(OBJS:.o=.d)
