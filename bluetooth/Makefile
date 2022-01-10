# Makefile for Raspberry Pi
BTSTACK_ROOT ?= ../..

CORE += \
	btstack_chipset_bcm.c \
	btstack_chipset_bcm_download_firmware.c \
	btstack_control_raspi.c \
	btstack_link_key_db_tlv.c \
	btstack_run_loop_posix.c \
	btstack_tlv_posix.c \
	btstack_uart_posix.c \
	btstack_slip.c \
	hci_dump_posix_fs.c \
	hci_transport_h4.c \
	hci_transport_h5.c \
	le_device_db_tlv.c \
	main_pigun.c \
	main_pigun_core.c \
	main_pigun_hid.c \
	wav_util.c 					\
	btstack_stdin_posix.c \
	raspi_get_model.c \
	rijndael.c


VPATH += ${BTSTACK_ROOT}/src
VPATH += ${BTSTACK_ROOT}/src/ble
VPATH += ${BTSTACK_ROOT}/src/ble/gatt-service
VPATH += ${BTSTACK_ROOT}/src/classic
VPATH += ${BTSTACK_ROOT}/src/mesh
VPATH += ${BTSTACK_ROOT}/example
VPATH += ${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/srce
VPATH += ${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/srce
VPATH += ${BTSTACK_ROOT}/3rd-party/hxcmod-player
VPATH += ${BTSTACK_ROOT}/3rd-party/hxcmod-player
VPATH += ${BTSTACK_ROOT}/3rd-party/hxcmod-player/mods
VPATH += ${BTSTACK_ROOT}/3rd-party/lwip/core/src/core/
VPATH += ${BTSTACK_ROOT}/3rd-party/lwip/core/src/core/ipv4
VPATH += ${BTSTACK_ROOT}/3rd-party/lwip/core/src/core/ipv6
VPATH += ${BTSTACK_ROOT}/3rd-party/lwip/core/src/netif
VPATH += ${BTSTACK_ROOT}/3rd-party/lwip/core/src/apps/http
VPATH += ${BTSTACK_ROOT}/3rd-party/lwip/dhcp-server
VPATH += ${BTSTACK_ROOT}/3rd-party/md5
VPATH += ${BTSTACK_ROOT}/3rd-party/micro-ecc
VPATH += ${BTSTACK_ROOT}/3rd-party/yxml
VPATH += ${BTSTACK_ROOT}/platform/lwip
VPATH += ${BTSTACK_ROOT}/platform/lwip/port

CFLAGS += -I.
CFLAGS += -I${BTSTACK_ROOT}/src/ble
CFLAGS += -I${BTSTACK_ROOT}/src/classic
CFLAGS += -I${BTSTACK_ROOT}/src/mesh
CFLAGS += -I${BTSTACK_ROOT}/src
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/include
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/include
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/hxcmod-player
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/lwip/core/src/include/
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/lwip/dhcp-server
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/md5
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/micro-ecc
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/yxml
CFLAGS += -I${BTSTACK_ROOT}/platform/lwip
CFLAGS += -I${BTSTACK_ROOT}/platform/lwip/port

# for CVSD/SBC PLC
LDFLAGS += -lm

CORE += \
	btstack_memory.c            \
	btstack_linked_list.c	    \
	btstack_memory_pool.c       \
	btstack_run_loop.c		    \
	btstack_util.c 	            \

COMMON += \
	ad_parser.c                 \
	hci.c			            \
	hci_cmd.c		            \
	hci_dump.c		            \
	l2cap.c			            \
	l2cap_signaling.c	        \
	btstack_audio.c             \
	btstack_tlv.c               \
	btstack_crypto.c            \
	uECC.c                      \
	sm.c                        \

CLASSIC ?= \
	sdp_util.c	                \
	gatt_sdp.c                  \
	spp_server.c  				\
	rfcomm.c	                \
	bnep.c	                    \
	sdp_server.c			    \
	device_id_server.c          \

SDP_CLIENT += \
	sdp_client.o		        \
	sdp_client_rfcomm.o		    \



# .o for .c
CORE_OBJ    = $(CORE:.c=.o)
COMMON_OBJ  = $(COMMON:.c=.o)
CLASSIC_OBJ  = $(CLASSIC:.c=.o)


# use (cross)compiler for Raspi
CC = arm-linux-gnueabihf-gcc

CFLAGS  += -g -Wall -Werror \
	-I$(BTSTACK_ROOT)/platform/embedded \
	-I$(BTSTACK_ROOT)/platform/posix \
	-I$(BTSTACK_ROOT)/chipset/bcm \
	-I${BTSTACK_ROOT}/3rd-party/tinydir \
    -I${BTSTACK_ROOT}/3rd-party/rijndael



# add 'real time' lib for clock_gettime
LDFLAGS += -lrt -lpthread

VPATH += ${BTSTACK_ROOT}/3rd-party/rijndael
VPATH += ${BTSTACK_ROOT}/platform/posix
VPATH += ${BTSTACK_ROOT}/platform/embedded
VPATH += ${BTSTACK_ROOT}/chipset/bcm	




all: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} btstack_ring_buffer.o hid_device.o btstack_hid_parser.o main_pigun_core.o main_pigun_hid.o main_pigun.o
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o pigun.exe