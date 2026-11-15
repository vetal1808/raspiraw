/*
Copyright (c) 2015, Raspberry Pi Foundation
Copyright (c) 2015, Dave Stevenson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define VERSION_STRING "0.0.3"

#define _GNU_SOURCE
#include <ctype.h>
#include <fcntl.h>
#include <inttypes.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#define I2C_SLAVE_FORCE 0x0706
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

#include "interface/vcsm/user-vcsm.h"

#include "RaspiCLI.h"

#include <sys/ioctl.h>

#include "raw_header.h"

#define DEFAULT_I2C_DEVICE 10
#define ALT_I2C_DEVICE	   0

#define I2C_DEVICE_NAME_LEN 13 // "/dev/i2c-XXX"+NULL

// Pseudo register addresses that can be used in a sensor_regs table
#define SENSOR_REG_I2C_ADDR 0xFFFF // Switch to the I2C slave address given as data
#define SENSOR_REG_DELAY    0xFFFE // Sleep for the number of ms given as data

struct brcm_raw_header *brcm_header = NULL;

enum bayer_order
{
	// Carefully ordered so that an hflip is ^1,
	// and a vflip is ^2.
	BAYER_ORDER_BGGR,
	BAYER_ORDER_GBRG,
	BAYER_ORDER_GRBG,
	BAYER_ORDER_RGGB
};

struct sensor_regs
{
	uint16_t reg;
	uint16_t data;
};

struct mode_def
{
	struct sensor_regs *regs;
	int num_regs;
	int width;
	int height;
	MMAL_FOURCC_T encoding;
	enum bayer_order order;
	int native_bit_depth;
	uint8_t image_id;
	uint8_t data_lanes;
	int min_vts;
	int line_time_ns;
	uint32_t timing[5];
	uint32_t term[2];
	int black_level;

	int binning; /* Binning or skipping factor */
};

struct raspiraw_crop
{
	int hinc;
	int vinc;
	int width;
	int height;
	int left;
	int top;
};

struct sensor_def
{
	char *name;
	struct sensor_regs *common_init;
	int num_common_init;
	struct mode_def *modes;
	int num_modes;
	struct sensor_regs *stop;
	int num_stop_regs;

	uint8_t i2c_addr;   // Device I2C slave address
	int i2c_addressing; // Length of register address values
	int i2c_data_size;  // Length of register data to write

	//  Detecting the device
	int i2c_ident_length;	  // Length of I2C ID register
	uint16_t i2c_ident_reg;	  // ID register address
	uint16_t i2c_ident_value; // ID register value

	// Flip configuration
	uint16_t vflip_reg;		   // Register for VFlip
	int vflip_reg_bit;		   // Bit in that register for VFlip
	uint16_t hflip_reg;		   // Register for HFlip
	int hflip_reg_bit;		   // Bit in that register for HFlip
	int flips_dont_change_bayer_order; // Some sensors do not change the
					   // Bayer order by adjusting X/Y
					   // starts to compensate.

	uint16_t exposure_reg;
	int exposure_reg_num_bits;

	uint16_t vts_reg;
	int vts_reg_num_bits;

	uint16_t gain_reg;
	int gain_reg_num_bits;

	int (*set_crop)(const struct sensor_def *, struct mode_def *, const struct raspiraw_crop *cfg);
};

// The process first loads the cleaned up dump of the registers
// than updates the known registers to the proper values
// based on: http://www.seeedstudio.com/wiki/images/3/3c/Ov5647_full.pdf
enum operation
{
	EQUAL, // Set bit to value
	SET,   // Set bit
	CLEAR, // Clear bit
	XOR    // Xor bit
};

void modReg(struct mode_def *mode, uint16_t reg, int startBit, int endBit, int value, enum operation op);

#define NUM_ELEMENTS(a) (sizeof(a) / sizeof(a[0]))

#include "adv7282m_modes.h"
#include "imx219_modes.h"
#include "imx477_modes.h"
#include "ov5647_modes.h"

const struct sensor_def *sensors[] = { &ov5647, &imx219, &adv7282, &imx477, NULL };

enum
{
	CommandHelp,
	CommandMode,
	CommandHFlip,
	CommandVFlip,
	CommandExposure,
	CommandGain,
	CommandOutput,
	CommandWriteHeader,
	CommandTimeout,
	CommandSaveRate,
	CommandBitDepth,
	CommandCameraNum,
	CommandExposureus,
	CommandI2cBus,
	CommandAwbGains,
	CommandRegs,
	CommandHinc,
	CommandVinc,
	CommandFps,
	CommandWidth,
	CommandHeight,
	CommandLeft,
	CommandTop,
	CommandVts,
	CommandLine,
	CommandWriteHeader0,
	CommandWriteHeaderG,
	CommandWriteTimestamps,
	CommandWriteEmpty,
	CommandDecodeMetadata,
	CommandAwb,
	CommandNoPreview,
	CommandProcessing,
	CommandPreview,
	CommandFullScreen,
	CommandOpacity,
	CommandProcessingYUV,
	CommandOutputYUV,
};

static COMMAND_LIST cmdline_commands[] = {
	// clang-format off
	{ CommandHelp,            "-help",           "?",    "This help information", 0 },
	{ CommandMode,            "-mode",           "md",   "Set sensor mode <mode>", 1 },
	{ CommandHFlip,           "-hflip",          "hf",   "Set horizontal flip", 0 },
	{ CommandVFlip,           "-vflip",          "vf",   "Set vertical flip", 0 },
	{ CommandExposure,        "-ss",             "e",    "Set the sensor exposure time (not calibrated units)", 0 },
	{ CommandGain,            "-gain",           "g",    "Set the sensor gain code (not calibrated units)", 0 },
	{ CommandOutput,          "-output",         "o",    "Set the output filename", 0 },
	{ CommandWriteHeader,     "-header",         "hd",   "Write the BRCM header to the output file", 0 },
	{ CommandTimeout,         "-timeout",        "t",    "Time (in ms) before shutting down (if not specified, set to 5s)", 1 },
	{ CommandSaveRate,        "-saverate",       "sr",   "Save every Nth frame", 1 },
	{ CommandBitDepth,        "-bitdepth",       "b",    "Set output raw bit depth (8, 10, 12 or 16, if not specified, set to sensor native)", 1 },
	{ CommandCameraNum,       "-cameranum",      "c",    "Set camera number to use (0=CAM0, 1=CAM1).", 1 },
	{ CommandExposureus,      "-expus",          "eus",  "Set the sensor exposure time in micro seconds.", -1 },
	{ CommandI2cBus,          "-i2c",            "y",    "Set the I2C bus to use.", -1 },
	{ CommandAwbGains,        "-awbgains",       "awbg", "Set the AWB gains to use.", 1 },
	{ CommandRegs,            "-regs",           "r",    "Change (current mode) regs", 0 },
	{ CommandHinc,            "-hinc",           "hi",   "Set horizontal odd/even inc reg", -1 },
	{ CommandVinc,            "-vinc",           "vi",   "Set vertical odd/even inc reg", -1 },
	{ CommandFps,             "-fps",            "f",    "Set framerate regs", -1 },
	{ CommandWidth,           "-width",          "w",    "Set current mode width", -1 },
	{ CommandHeight,          "-height",         "h",    "Set current mode height", -1 },
	{ CommandLeft,            "-left",           "lt",   "Set current mode left", -1 },
	{ CommandTop,             "-top",            "tp",   "Set current mode top", -1 },
	{ CommandWriteHeader0,    "-header0",        "hd0",  "Sets filename to write the BRCM header to", 0 },
	{ CommandWriteHeaderG,    "-headerg",        "hdg",  "Sets filename to write the .pgm header to", 0 },
	{ CommandWriteTimestamps, "-tstamps",        "ts",   "Sets filename to write timestamps to", 0 },
	{ CommandWriteEmpty,      "-empty",          "emp",  "Write empty output files", 0 },
	{ CommandDecodeMetadata,  "-metadata",       "m",    "Decode register metadata", 0 },
	{ CommandAwb,             "-awb",            "awb",  "Use a simple grey-world AWB algorithm", 0 },
	{ CommandNoPreview,       "-nopreview",      "n",    "Do not send the stream to the display", 0 },
	{ CommandProcessing,      "-processing",     "P",    "Pass images into an image processing function", 0 },
	{ CommandPreview,         "-preview",        "p",    "Preview window settings <'x,y,w,h'>", 1 },
	{ CommandFullScreen,      "-fullscreen",     "fs",   "Fullscreen preview mode", 0 },
	{ CommandOpacity,         "-opacity",        "op",   "Preview window opacity (0-255)", 1 },
	{ CommandProcessingYUV,   "-processing_yuv", "PY",   "Pass processed YUV images into an image processing function", 0 },
	{ CommandOutputYUV,       "-output_yuv", "oY", "Set the output filename for YUV data", 0 },
	// clang-format on
};

static int cmdline_commands_size = sizeof(cmdline_commands) / sizeof(cmdline_commands[0]);

typedef struct pts_node
{
	int idx;
	int64_t pts;
	struct pts_node *nxt;
} * PTS_NODE_T;

#define DEFAULT_PREVIEW_LAYER 3

typedef struct raspiraw_params
{
	struct raspiraw_crop crop;
	int mode;
	int hflip;
	int vflip;
	int exposure;
	int gain;
	char *output;
	int capture;
	int write_header;
	int timeout;
	int saverate;
	int bit_depth;
	int camera_num;
	int exposure_us;
	int i2c_bus;
	double awb_gains_r;
	double awb_gains_b;
	char *regs;
	double fps;
	char *write_header0;
	char *write_headerg;
	char *write_timestamps;
	int write_empty;
	PTS_NODE_T ptsa;
	PTS_NODE_T ptso;
	int decodemetadata;
	int awb;
	int no_preview;
	int processing;
	int fullscreen; // 0 is use previewRect, non-zero to use full screen
	int opacity;	// Opacity of window - 0 = transparent, 255 = opaque
	MMAL_RECT_T preview_window; // Destination rectangle for the preview window.
	int capture_yuv;
	char *output_yuv;
	int processing_yuv;
} RASPIRAW_PARAMS_T;

typedef struct
{
	RASPIRAW_PARAMS_T *cfg;

	MMAL_POOL_T *rawcam_pool;
	MMAL_PORT_T *rawcam_output;

	MMAL_POOL_T *isp_ip_pool;
	MMAL_PORT_T *isp_ip;

	MMAL_QUEUE_T *awb_queue;
	int awb_thread_quit;
	MMAL_PARAMETER_AWB_GAINS_T wb_gains;

	MMAL_QUEUE_T *processing_queue;
	int processing_thread_quit;
} RASPIRAW_CALLBACK_T;

typedef struct
{
	RASPIRAW_PARAMS_T *cfg;

	MMAL_POOL_T *isp_op_pool;
	MMAL_PORT_T *isp_output;

	MMAL_POOL_T *vr_ip_pool;
	MMAL_PORT_T *vr_ip;

	MMAL_QUEUE_T *processing_yuv_queue;
	int processing_yuv_thread_quit;
} RASPIRAW_ISP_CALLBACK_T;

typedef struct
{
	MMAL_COMPONENT_T *rawcam;
	MMAL_COMPONENT_T *isp;
	MMAL_COMPONENT_T *render;

	MMAL_POOL_T *rawcam_pool;

	VCOS_THREAD_T processing_yuv_thread;
	int processing_yuv_thread_started;
} RASPIRAW_PIPELINE_T;

/* ------------------------------------------------------------------------
 * Sensor register table manipulation
 * ------------------------------------------------------------------------ */

void modRegBit(struct mode_def *mode, uint16_t reg, int bit, int value, enum operation op)
{
	int i = 0;
	uint16_t val;
	while (i < mode->num_regs && mode->regs[i].reg != reg)
		i++;
	if (i == mode->num_regs)
	{
		vcos_log_error("Reg: %04X not found!\n", reg);
		return;
	}
	val = mode->regs[i].data;

	switch (op)
	{
	case EQUAL:
		val = val & ~(1 << bit);
		val = val | (value << bit);
		break;
	case SET:
		val = val | (1 << bit);
		break;
	case CLEAR:
		val = val & ~(1 << bit);
		break;
	case XOR:
		val = val ^ (value << bit);
		break;
	}
	mode->regs[i].data = val;
}

void modReg(struct mode_def *mode, uint16_t reg, int startBit, int endBit, int value, enum operation op)
{
	int i;
	for (i = startBit; i <= endBit; i++)
	{
		modRegBit(mode, reg, i, value >> i & 1, op);
	}
}

static int value_fits(int value, int num_bits)
{
	return value >= 0 && value < (1 << num_bits);
}

/* Spread a num_bits wide value, MSB first, over consecutive 8 bit registers starting at reg. */
static void set_multibyte_reg(struct mode_def *mode, uint16_t reg, int num_bits, int value, const char *name)
{
	int i, shift = VCOS_ALIGN_DOWN(num_bits - 1, 8);
	int num_regs = (num_bits + 7) >> 3;

	for (i = 0; i < num_regs; i++, shift -= 8)
	{
		uint8_t val = (value >> shift) & 0xFF;
		modReg(mode, reg + i, 0, 7, val, EQUAL);
		vcos_log_error("Set %s %04X to %02X", name, reg + i, val);
	}
}

void update_regs(const struct sensor_def *sensor, struct mode_def *mode, int hflip, int vflip, int exposure, int gain)
{
	if (sensor->vflip_reg)
	{
		modRegBit(mode, sensor->vflip_reg, sensor->vflip_reg_bit, vflip, XOR);
		if (vflip && !sensor->flips_dont_change_bayer_order)
			mode->order ^= 2;
	}

	if (sensor->hflip_reg)
	{
		modRegBit(mode, sensor->hflip_reg, sensor->hflip_reg_bit, hflip, XOR);
		if (hflip && !sensor->flips_dont_change_bayer_order)
			mode->order ^= 1;
	}

	if (sensor->exposure_reg && exposure != -1)
	{
		if (value_fits(exposure, sensor->exposure_reg_num_bits))
			set_multibyte_reg(mode, sensor->exposure_reg, sensor->exposure_reg_num_bits, exposure,
					  "exposure");
		else
			vcos_log_error("Invalid exposure:%d, exposure range is 0 to %u!\n", exposure,
				       (1 << sensor->exposure_reg_num_bits) - 1);
	}

	if (sensor->vts_reg && exposure != -1 && exposure >= mode->min_vts)
	{
		if (value_fits(exposure, sensor->vts_reg_num_bits))
			set_multibyte_reg(mode, sensor->vts_reg, sensor->vts_reg_num_bits, exposure, "vts");
		else
			vcos_log_error("Invalid exposure:%d, vts range is 0 to %u!\n", exposure,
				       (1 << sensor->vts_reg_num_bits) - 1);
	}

	if (sensor->gain_reg && gain != -1)
	{
		if (value_fits(gain, sensor->gain_reg_num_bits))
			set_multibyte_reg(mode, sensor->gain_reg, sensor->gain_reg_num_bits, gain, "gain");
		else
			vcos_log_error("Invalid gain:%d, gain range is 0 to %u\n", gain,
				       (1 << sensor->gain_reg_num_bits) - 1);
	}
}

/* ------------------------------------------------------------------------
 * I2C access to the sensor
 * ------------------------------------------------------------------------ */

static int i2c_rd(int fd, uint8_t i2c_addr, uint16_t reg, uint8_t *values, uint32_t n, const struct sensor_def *sensor)
{
	int err;
	uint8_t buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2] = {
		{
		    .addr = i2c_addr,
		    .flags = 0,
		    .len = 2,
		    .buf = buf,
		},
		{
		    .addr = i2c_addr,
		    .flags = I2C_M_RD,
		    .len = n,
		    .buf = values,
		},
	};

	if (sensor->i2c_addressing == 1)
	{
		msgs[0].len = 1;
	}
	msgset.msgs = msgs;
	msgset.nmsgs = 2;

	err = ioctl(fd, I2C_RDWR, &msgset);
	if (err != (int)msgset.nmsgs)
		return -1;

	return 0;
}

const struct sensor_def *probe_sensor(int fd)
{
	const struct sensor_def **sensor_list;

	for (sensor_list = &sensors[0]; *sensor_list != NULL; sensor_list++)
	{
		const struct sensor_def *sensor = *sensor_list;
		uint16_t reg = 0;

		vcos_log_error("Probing sensor %s on addr %02X", sensor->name, sensor->i2c_addr);
		if (sensor->i2c_ident_length > 2)
			continue;
		if (i2c_rd(fd, sensor->i2c_addr, sensor->i2c_ident_reg, (uint8_t *)&reg, sensor->i2c_ident_length,
			   sensor))
			continue;
		if (reg == sensor->i2c_ident_value)
		{
			vcos_log_error("Found sensor %s at address %02X", sensor->name, sensor->i2c_addr);
			return sensor;
		}
	}
	return NULL;
}

/* Returns non-zero if the register was written. */
static int write_reg(int fd, const struct sensor_def *sensor, uint16_t reg, uint16_t data)
{
	unsigned char msg[4];
	int len = 0;

	if (sensor->i2c_addressing != 1)
		msg[len++] = reg >> 8;
	msg[len++] = reg & 0xFF;
	if (sensor->i2c_data_size == 2)
		msg[len++] = data >> 8;
	msg[len++] = data & 0xFF;

	return write(fd, msg, len) == len;
}

void send_regs(int fd, const struct sensor_def *sensor, const struct sensor_regs *regs, int num_regs)
{
	int i;
	for (i = 0; i < num_regs; i++)
	{
		if (regs[i].reg == SENSOR_REG_I2C_ADDR)
		{
			if (ioctl(fd, I2C_SLAVE_FORCE, regs[i].data) < 0)
			{
				vcos_log_error("Failed to set I2C address to %02X", regs[i].data);
			}
		}
		else if (regs[i].reg == SENSOR_REG_DELAY)
		{
			vcos_sleep(regs[i].data);
		}
		else if (!write_reg(fd, sensor, regs[i].reg, regs[i].data))
		{
			vcos_log_error("Failed to write register index %d (%04X val %02X)", i, regs[i].reg,
				       regs[i].data);
		}
	}
}

void start_camera_streaming(const struct sensor_def *sensor, struct mode_def *mode, int fd)
{
	if (ioctl(fd, I2C_SLAVE_FORCE, sensor->i2c_addr) < 0)
	{
		vcos_log_error("Failed to set I2C address");
		return;
	}
	if (sensor->common_init)
		send_regs(fd, sensor, sensor->common_init, sensor->num_common_init);
	send_regs(fd, sensor, mode->regs, mode->num_regs);
	vcos_log_error("Now streaming...");
}

void stop_camera_streaming(const struct sensor_def *sensor, int fd)
{
	if (ioctl(fd, I2C_SLAVE_FORCE, sensor->i2c_addr) < 0)
	{
		vcos_log_error("Failed to set I2C address");
		return;
	}
	send_regs(fd, sensor, sensor->stop, sensor->num_stop_regs);
}

static int open_i2c_bus(int bus, char *name, size_t name_len)
{
	snprintf(name, name_len, "/dev/i2c-%d", bus);
	return open(name, O_RDWR);
}

/* Opens the requested I2C bus, or the default one (falling back to the alternate) if bus is -1. */
static int open_i2c_device(int bus)
{
	char name[I2C_DEVICE_NAME_LEN];
	int fd;

	if (bus != -1)
	{
		fd = open_i2c_bus(bus, name, sizeof(name));
	}
	else
	{
		fd = open_i2c_bus(DEFAULT_I2C_DEVICE, name, sizeof(name));
		if (fd < 0)
			fd = open_i2c_bus(ALT_I2C_DEVICE, name, sizeof(name));
	}

	if (fd < 0)
		printf("Failed to open I2C device %s\n", name);
	else
		printf("Using I2C device %s\n", name);
	return fd;
}

/* ------------------------------------------------------------------------
 * Format helpers
 * ------------------------------------------------------------------------ */

int encoding_to_bpp(uint32_t encoding)
{
	switch (encoding)
	{
	case MMAL_ENCODING_BAYER_SBGGR10P:
	case MMAL_ENCODING_BAYER_SGBRG10P:
	case MMAL_ENCODING_BAYER_SGRBG10P:
	case MMAL_ENCODING_BAYER_SRGGB10P:
		return 10;
	case MMAL_ENCODING_BAYER_SBGGR12P:
	case MMAL_ENCODING_BAYER_SGBRG12P:
	case MMAL_ENCODING_BAYER_SGRBG12P:
	case MMAL_ENCODING_BAYER_SRGGB12P:
		return 12;
	default:
		return 8;
	};
}

uint32_t order_and_bit_depth_to_encoding(enum bayer_order order, int bit_depth)
{
	// Indexed by enum bayer_order: BGGR, GBRG, GRBG, RGGB
	const uint32_t depth8[] = { MMAL_ENCODING_BAYER_SBGGR8, MMAL_ENCODING_BAYER_SGBRG8, MMAL_ENCODING_BAYER_SGRBG8,
				    MMAL_ENCODING_BAYER_SRGGB8 };
	const uint32_t depth10[] = { MMAL_ENCODING_BAYER_SBGGR10P, MMAL_ENCODING_BAYER_SGBRG10P,
				     MMAL_ENCODING_BAYER_SGRBG10P, MMAL_ENCODING_BAYER_SRGGB10P };
	const uint32_t depth12[] = { MMAL_ENCODING_BAYER_SBGGR12P, MMAL_ENCODING_BAYER_SGBRG12P,
				     MMAL_ENCODING_BAYER_SGRBG12P, MMAL_ENCODING_BAYER_SRGGB12P };
	const uint32_t depth16[] = { MMAL_ENCODING_BAYER_SBGGR16, MMAL_ENCODING_BAYER_SGBRG16,
				     MMAL_ENCODING_BAYER_SGRBG16, MMAL_ENCODING_BAYER_SRGGB16 };

	if (order < 0 || order > 3)
	{
		vcos_log_error("order out of range - %d", order);
		return 0;
	}

	switch (bit_depth)
	{
	case 8:
		return depth8[order];
	case 10:
		return depth10[order];
	case 12:
		return depth12[order];
	case 16:
		return depth16[order];
	}
	vcos_log_error("%d not one of the handled bit depths", bit_depth);
	return 0;
}

static int bit_depth_to_rx_unpack(int bit_depth)
{
	switch (bit_depth)
	{
	case 8:
		return MMAL_CAMERA_RX_CONFIG_UNPACK_8;
	case 10:
		return MMAL_CAMERA_RX_CONFIG_UNPACK_10;
	case 12:
		return MMAL_CAMERA_RX_CONFIG_UNPACK_12;
	case 14:
	case 16:
		return MMAL_CAMERA_RX_CONFIG_UNPACK_16;
	default:
		vcos_log_error("Unknown native bit depth %d", bit_depth);
		return MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
	}
}

static int bit_depth_to_rx_pack(int bit_depth)
{
	switch (bit_depth)
	{
	case 8:
		return MMAL_CAMERA_RX_CONFIG_PACK_8;
	case 10:
		return MMAL_CAMERA_RX_CONFIG_PACK_RAW10;
	case 12:
		return MMAL_CAMERA_RX_CONFIG_PACK_RAW12;
	case 14:
		return MMAL_CAMERA_RX_CONFIG_PACK_14;
	case 16:
		return MMAL_CAMERA_RX_CONFIG_PACK_16;
	default:
		vcos_log_error("Unknown output bit depth %d", bit_depth);
		return MMAL_CAMERA_RX_CONFIG_PACK_NONE;
	}
}

static int bayer_order_to_vc_image(enum bayer_order order)
{
	switch (order)
	{
	case BAYER_ORDER_BGGR:
		return VC_IMAGE_BAYER_BGGR;
	case BAYER_ORDER_GBRG:
		return VC_IMAGE_BAYER_GBRG;
	case BAYER_ORDER_GRBG:
		return VC_IMAGE_BAYER_GRBG;
	case BAYER_ORDER_RGGB:
		return VC_IMAGE_BAYER_RGGB;
	}
	return 0;
}

static int bit_depth_to_vc_image(int bit_depth)
{
	switch (bit_depth)
	{
	case 8:
		return VC_IMAGE_BAYER_RAW8;
	case 10:
		return VC_IMAGE_BAYER_RAW10;
	case 12:
		return VC_IMAGE_BAYER_RAW12;
	case 14:
		return VC_IMAGE_BAYER_RAW14;
	case 16:
		return VC_IMAGE_BAYER_RAW16;
	}
	return 0;
}

/* ------------------------------------------------------------------------
 * Frame output
 * ------------------------------------------------------------------------ */

/**
 * Allocates and generates a filename based on the
 * user-supplied pattern and the frame number.
 * On successful return, finalName points to a malloc()ed string
 * which must be freed externally.  (On failure, returns null that
 * doesn't need free()ing.)
 *
 * @param finalName pointer receives the generated filename
 * @param pattern sprintf pattern with %d to be replaced by frame
 * @param frame for timelapse, the frame number
 * @return Returns a MMAL_STATUS_T giving result of operation
 */
MMAL_STATUS_T create_filenames(char **finalName, const char *pattern, int frame)
{
	*finalName = NULL;
	if (0 > asprintf(finalName, pattern, frame))
	{
		// It may be some other error, but it is not worth getting it right
		return MMAL_ENOMEM;
	}
	return MMAL_SUCCESS;
}

/**
 * Writes one frame to the file named by pattern and frame number.
 * With write_empty set, the file is created but left empty.
 *
 * @return non-zero if the file was created
 */
static int write_frame_file(const char *pattern, int frame, const MMAL_BUFFER_HEADER_T *buffer, int write_header,
			    int write_empty)
{
	char *filename = NULL;
	FILE *file;

	if (create_filenames(&filename, pattern, frame) != MMAL_SUCCESS)
		return 0;

	file = fopen(filename, "wb");
	free(filename);
	if (!file)
		return 0;

	if (!write_empty)
	{
		if (write_header && brcm_header)
			fwrite(brcm_header, BRCM_RAW_HEADER_LENGTH, 1, file);
		fwrite(buffer->user_data, buffer->length, 1, file);
	}
	fclose(file);
	return 1;
}

/* Appends a frame timestamp to the list. Does nothing if timestamps are not being collected. */
static void record_timestamp(RASPIRAW_PARAMS_T *cfg, int idx, int64_t pts)
{
	if (!cfg->ptso) // also covers a failed malloc() of the previous node
		return;

	cfg->ptso->idx = idx;
	cfg->ptso->pts = pts;
	cfg->ptso->nxt = malloc(sizeof(*cfg->ptso->nxt));
	cfg->ptso = cfg->ptso->nxt;
}

static void write_timestamps(const RASPIRAW_PARAMS_T *cfg)
{
	int64_t old = 0;
	PTS_NODE_T aux;
	FILE *file = fopen(cfg->write_timestamps, "wb");

	if (!file)
		return;

	for (aux = cfg->ptsa; aux != cfg->ptso; aux = aux->nxt)
	{
		if (aux == cfg->ptsa)
			fprintf(file, ",%d,%" PRId64 "\n", aux->idx, aux->pts);
		else
			fprintf(file, "%" PRId64 ",%d,%" PRId64 "\n", aux->pts - old, aux->idx, aux->pts);
		old = aux->pts;
	}
	fclose(file);
}

static void free_timestamps(RASPIRAW_PARAMS_T *cfg)
{
	while (cfg->ptsa != cfg->ptso)
	{
		PTS_NODE_T aux = cfg->ptsa->nxt;
		free(cfg->ptsa);
		cfg->ptsa = aux;
	}
	free(cfg->ptso);
	cfg->ptsa = cfg->ptso = NULL;
}

static void write_file(const char *filename, const void *data, size_t length)
{
	FILE *file = fopen(filename, "wb");
	if (file)
	{
		fwrite(data, length, 1, file);
		fclose(file);
	}
}

static void write_pgm_header(const char *filename, const struct mode_def *mode)
{
	FILE *file = fopen(filename, "wb");
	if (file)
	{
		fprintf(file, "P5\n%d %d\n255\n", mode->width, mode->height);
		fclose(file);
	}
}

static void create_brcm_header(const struct mode_def *mode, int bit_depth)
{
	brcm_header = (struct brcm_raw_header *)calloc(1, BRCM_RAW_HEADER_LENGTH);
	if (!brcm_header)
		return;

	brcm_header->id = BRCM_ID_SIG;
	brcm_header->version = HEADER_VERSION;
	brcm_header->mode.width = mode->width;
	brcm_header->mode.height = mode->height;
	// FIXME: Ought to check that the sensor is producing Bayer rather than just assuming.
	brcm_header->mode.format = VC_IMAGE_BAYER;
	brcm_header->mode.bayer_order = bayer_order_to_vc_image(mode->order);
	brcm_header->mode.bayer_format = bit_depth_to_vc_image(bit_depth);
}

/* Prepares the BRCM header for the captured frames and writes the one-off header files. */
static void prepare_capture_headers(const RASPIRAW_PARAMS_T *cfg, const struct mode_def *mode)
{
	if (!cfg->capture)
		return;

	if (cfg->write_header || cfg->write_header0)
	{
		create_brcm_header(mode, cfg->bit_depth);
		if (brcm_header && cfg->write_header0)
			write_file(cfg->write_header0, brcm_header, BRCM_RAW_HEADER_LENGTH);
	}
	else if (cfg->write_headerg)
	{
		write_pgm_header(cfg->write_headerg, mode);
	}
}

void decodemetadataline(uint8_t *data, int bpp)
{
	int c = 1;
	uint8_t tag, dta;
	uint16_t reg = -1;

	if (data[0] != 0x0a)
	{
		vcos_log_error("Doesn't looks like register set %x!=0x0a", data[0]);
		return;
	}

	while (data[c] != 0x07)
	{
		tag = data[c++];
		if (bpp == 10 && (c % 5) == 4)
			c++;
		if (bpp == 12 && (c % 3) == 2)
			c++;
		dta = data[c++];

		if (tag == 0xaa)
			reg = (reg & 0x00ff) | (dta << 8);
		else if (tag == 0xa5)
			reg = (reg & 0xff00) | dta;
		else if (tag == 0x5a)
			vcos_log_error("Register 0x%04x = 0x%02x", reg++, dta);
		else if (tag == 0x55)
			vcos_log_error("Skip     0x%04x", reg++);
		else
			vcos_log_error("Metadata decode failed %x %x %x", reg, tag, dta);
	}
}

static void decode_metadata(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	int bpp = encoding_to_bpp(port->format->encoding);
	int pitch = mmal_encoding_width_to_stride(port->format->encoding, port->format->es->video.width);

	vcos_log_error("First metadata line");
	decodemetadataline(buffer->user_data, bpp);
	vcos_log_error("Second metadata line");
	decodemetadataline(buffer->user_data + pitch, bpp);
}

/* ------------------------------------------------------------------------
 * MMAL buffer callbacks
 * ------------------------------------------------------------------------ */

static int is_metadata_buffer(const MMAL_BUFFER_HEADER_T *buffer)
{
	return (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) != 0;
}

/* Sends a replica of buffer, sharing the same memory, to another port. */
static void forward_buffer(MMAL_PORT_T *port, MMAL_POOL_T *pool, MMAL_BUFFER_HEADER_T *buffer, const char *dest)
{
	MMAL_STATUS_T status;
	MMAL_BUFFER_HEADER_T *out = mmal_queue_get(pool->queue);

	if (!out)
		return;

	mmal_buffer_header_replicate(out, buffer);
	out->data = buffer->data;
	out->alloc_size = buffer->alloc_size;
	status = mmal_port_send_buffer(port, out);
	if (status != MMAL_SUCCESS)
		vcos_log_error("Failed to send buffer %p to %s - %d", buffer, dest, status);
}

/* Passes buffer to a worker thread, which is relied on to release the reference taken here. */
static void queue_buffer(MMAL_QUEUE_T *queue, MMAL_BUFFER_HEADER_T *buffer)
{
	mmal_buffer_header_acquire(buffer);
	mmal_queue_put(queue, buffer);
}

static void buffers_to_rawcam(RASPIRAW_CALLBACK_T *dev)
{
	MMAL_BUFFER_HEADER_T *buffer;

	while ((buffer = mmal_queue_get(dev->rawcam_pool->queue)) != NULL)
	{
		mmal_port_send_buffer(dev->rawcam_output, buffer);
	}
}

static void buffers_to_isp_op(RASPIRAW_ISP_CALLBACK_T *dev)
{
	MMAL_BUFFER_HEADER_T *buffer;

	while ((buffer = mmal_queue_get(dev->isp_op_pool->queue)) != NULL)
	{
		mmal_port_send_buffer(dev->isp_output, buffer);
	}
}

static void rawcam_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	static int count = 0;
	RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)port->userdata;
	RASPIRAW_PARAMS_T *cfg = dev->cfg;

	// Save every Nth frame. SD card access is too slow to do much more.
	if (cfg->capture && !is_metadata_buffer(buffer) && ((count++) % cfg->saverate) == 0)
	{
		if (write_frame_file(cfg->output, count, buffer, cfg->write_header, cfg->write_empty))
			record_timestamp(cfg, count, buffer->pts);
	}

	if (cfg->decodemetadata && is_metadata_buffer(buffer))
		decode_metadata(port, buffer);

	/* Pass the buffers off to any other MMAL sinks. */
	if (buffer->length && !is_metadata_buffer(buffer))
	{
		if (dev->isp_ip)
			forward_buffer(dev->isp_ip, dev->isp_ip_pool, buffer, "isp");
		if (dev->awb_queue)
			queue_buffer(dev->awb_queue, buffer);
		if (dev->processing_queue)
			queue_buffer(dev->processing_queue, buffer);
	}

	mmal_buffer_header_release(buffer);

	buffers_to_rawcam(dev);
}

static void yuv_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	static int count = 0;
	RASPIRAW_ISP_CALLBACK_T *yuv_cb = (RASPIRAW_ISP_CALLBACK_T *)port->userdata;
	RASPIRAW_PARAMS_T *cfg = yuv_cb->cfg;

	// Save every Nth frame. SD card access is too slow to do much more.
	if (cfg->capture_yuv && !is_metadata_buffer(buffer) && ((count++) % cfg->saverate) == 0)
		write_frame_file(cfg->output_yuv, count, buffer, 0, cfg->write_empty);

	/* Pass the buffers off to any other MMAL sinks. */
	if (buffer->length && !is_metadata_buffer(buffer))
	{
		if (yuv_cb->vr_ip)
			forward_buffer(yuv_cb->vr_ip, yuv_cb->vr_ip_pool, buffer, "video render");
		if (yuv_cb->processing_yuv_queue)
			queue_buffer(yuv_cb->processing_yuv_queue, buffer);
	}

	mmal_buffer_header_release(buffer);

	buffers_to_isp_op(yuv_cb);
}

static void isp_ip_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)port->userdata;

	mmal_buffer_header_release(buffer);
	buffers_to_rawcam(dev);
}

static void vr_ip_cb(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	RASPIRAW_ISP_CALLBACK_T *dev = (RASPIRAW_ISP_CALLBACK_T *)port->userdata;

	mmal_buffer_header_release(buffer);
	buffers_to_isp_op(dev);
}

/* ------------------------------------------------------------------------
 * AWB and processing threads
 * ------------------------------------------------------------------------ */

uint32_t get_pixel(int x, int y, uint32_t encoding, int stride, uint8_t *data)
{
	uint32_t val = 0;
	const uint8_t raw10_masks[4] = { 0x03, 0x0c, 0x30, 0xc0 };
	const uint8_t raw10_shifts[4] = { 0, 2, 4, 6 };
	const uint8_t raw12_masks[2] = { 0x0f, 0xf0 };
	const uint8_t raw12_shifts[2] = { 0, 4 };

	switch (encoding)
	{
	case MMAL_ENCODING_BAYER_SBGGR8:
	case MMAL_ENCODING_BAYER_SGBRG8:
	case MMAL_ENCODING_BAYER_SGRBG8:
	case MMAL_ENCODING_BAYER_SRGGB8:
		val = data[x + (y * stride)];
		break;

	case MMAL_ENCODING_BAYER_SBGGR10P:
	case MMAL_ENCODING_BAYER_SGBRG10P:
	case MMAL_ENCODING_BAYER_SGRBG10P:
	case MMAL_ENCODING_BAYER_SRGGB10P:
		// 4 pixels in 5 bytes: the 8 MSBs of each, then a byte with the 4 pairs of LSBs
		val = data[x + (x / 4) + (y * stride)] << 2;
		val |= (data[(x / 4) * 5 + 4 + (y * stride)] & raw10_masks[x & 3]) >> raw10_shifts[x & 3];
		break;

	case MMAL_ENCODING_BAYER_SBGGR12P:
	case MMAL_ENCODING_BAYER_SGBRG12P:
	case MMAL_ENCODING_BAYER_SGRBG12P:
	case MMAL_ENCODING_BAYER_SRGGB12P:
		// 2 pixels in 3 bytes: the 8 MSBs of each, then a byte with the 2 nibbles of LSBs
		val = data[x + (x / 2) + (y * stride)] << 4;
		val |= (data[(x / 2) * 3 + 2 + (y * stride)] & raw12_masks[x & 1]) >> raw12_shifts[x & 1];
		break;

	case MMAL_ENCODING_BAYER_SBGGR16:
	case MMAL_ENCODING_BAYER_SGBRG16:
	case MMAL_ENCODING_BAYER_SGRBG16:
	case MMAL_ENCODING_BAYER_SRGGB16:
	{
		/* CHECK ME: are the MSB and LSB the right way around? */
		uint16_t *data16 = (uint16_t *)data;
		val = data16[x + (y * stride / 2)];
		break;
	}
	}
	return val;
}

enum channels
{
	RED,
	GREEN1,
	GREEN2,
	BLUE
};

uint64_t get_channel(enum channels chan, uint32_t encoding, uint64_t *sums)
{
	const int bggr[4] = { BLUE, GREEN1, GREEN2, RED };
	const int rggb[4] = { RED, GREEN1, GREEN2, BLUE };
	const int gbrg[4] = { GREEN1, BLUE, RED, GREEN2 };
	const int grbg[4] = { GREEN1, RED, BLUE, GREEN2 };

	switch (encoding)
	{
	case MMAL_ENCODING_BAYER_SBGGR8:
	case MMAL_ENCODING_BAYER_SBGGR10P:
	case MMAL_ENCODING_BAYER_SBGGR12P:
	case MMAL_ENCODING_BAYER_SBGGR16:
		return sums[bggr[chan]];

	case MMAL_ENCODING_BAYER_SRGGB8:
	case MMAL_ENCODING_BAYER_SRGGB10P:
	case MMAL_ENCODING_BAYER_SRGGB12P:
	case MMAL_ENCODING_BAYER_SRGGB16:
		return sums[rggb[chan]];

	case MMAL_ENCODING_BAYER_SGBRG8:
	case MMAL_ENCODING_BAYER_SGBRG10P:
	case MMAL_ENCODING_BAYER_SGBRG12P:
	case MMAL_ENCODING_BAYER_SGBRG16:
		return sums[gbrg[chan]];

	case MMAL_ENCODING_BAYER_SGRBG8:
	case MMAL_ENCODING_BAYER_SGRBG10P:
	case MMAL_ENCODING_BAYER_SGRBG12P:
	case MMAL_ENCODING_BAYER_SGRBG16:
		return sums[grbg[chan]];

	default:
		return 0;
	}
}

static void run_awb_calcs(RASPIRAW_CALLBACK_T *dev, MMAL_BUFFER_HEADER_T *buffer)
{
	int x, y, count = 0;
	uint64_t sums[4] = { 0 }; // Sums for each of the 4 channels
	int r_ave, g_ave, b_ave;
	uint32_t encoding = dev->rawcam_output->format->encoding;
	int stride = mmal_encoding_width_to_stride(encoding, dev->rawcam_output->format->es->video.width);

	if (!dev->wb_gains.hdr.id)
	{
		dev->wb_gains.hdr.id = MMAL_PARAMETER_CUSTOM_AWB_GAINS;
		dev->wb_gains.hdr.size = sizeof(dev->wb_gains);
		dev->wb_gains.r_gain.num = 256;
		dev->wb_gains.r_gain.den = 256;
		dev->wb_gains.b_gain.num = 256;
		dev->wb_gains.b_gain.den = 256;
	}

	for (x = 8; x < dev->rawcam_output->format->es->video.crop.width; x += 16)
	{
		for (y = 8; y < dev->rawcam_output->format->es->video.crop.height; y += 16)
		{
			sums[0] += get_pixel(x, y, encoding, stride, buffer->user_data);
			sums[1] += get_pixel(x + 1, y, encoding, stride, buffer->user_data);
			sums[2] += get_pixel(x, y + 1, encoding, stride, buffer->user_data);
			sums[3] += get_pixel(x + 1, y + 1, encoding, stride, buffer->user_data);
			count++;
		}
	}

	r_ave = get_channel(RED, encoding, sums) / vcos_max(count, 1);
	g_ave = (get_channel(GREEN1, encoding, sums) + get_channel(GREEN2, encoding, sums)) / vcos_max(count * 2, 1);
	b_ave = get_channel(BLUE, encoding, sums) / vcos_max(count, 1);

	/* FIXME: Do a moving average filter for smoother changes in values */
	/* +1 to protect against div by 0 */
	dev->wb_gains.r_gain.num = ((g_ave << 8) / (r_ave + 1));
	dev->wb_gains.b_gain.num = ((g_ave << 8) / (b_ave + 1));

	if (dev->isp_ip)
		mmal_port_parameter_set(dev->isp_ip, &dev->wb_gains.hdr);
}

static void *awb_thread_task(void *arg)
{
	RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)arg;
	MMAL_BUFFER_HEADER_T *buffer;

	while (!dev->awb_thread_quit)
	{
		// Being lazy and using a timed wait instead of setting up a
		// mechanism for skipping this when destroying the thread
		buffer = mmal_queue_timedwait(dev->awb_queue, 1000);
		if (!buffer)
			continue;

		// If more buffers are in the queue, loop so we're working on the latest one
		if (!mmal_queue_length(dev->awb_queue))
			run_awb_calcs(dev, buffer);

		mmal_buffer_header_release(buffer);
		buffers_to_rawcam(dev);
	}

	return NULL;
}

static void *processing_thread_task(void *arg)
{
	RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)arg;
	MMAL_BUFFER_HEADER_T *buffer;

	while (!dev->processing_thread_quit)
	{
		// Being lazy and using a timed wait instead of setting up a
		// mechanism for skipping this when destroying the thread
		buffer = mmal_queue_timedwait(dev->processing_queue, 1000);
		if (!buffer)
			continue;

		// If more buffers are in the queue, loop so we're working on the latest one
		if (!mmal_queue_length(dev->processing_queue))
		{
			// DO SOME FORM OF PROCESSING ON THE RAW BAYER DATA HERE
			// buffer->user_data points to the data, with
			// buffer->length being the length of the data.
		}

		mmal_buffer_header_release(buffer);
		buffers_to_rawcam(dev);
	}

	return NULL;
}

static void *processing_yuv_thread_task(void *arg)
{
	RASPIRAW_ISP_CALLBACK_T *yuv_cb = (RASPIRAW_ISP_CALLBACK_T *)arg;
	MMAL_BUFFER_HEADER_T *buffer;

	while (!yuv_cb->processing_yuv_thread_quit)
	{
		// Being lazy and using a timed wait instead of setting up a
		// mechanism for skipping this when destroying the thread
		buffer = mmal_queue_timedwait(yuv_cb->processing_yuv_queue, 1000);
		if (!buffer)
			continue;

		// If more buffers are in the queue, loop so we're working on the latest one
		if (!mmal_queue_length(yuv_cb->processing_yuv_queue))
		{
			// DO SOME FORM OF PROCESSING ON THE YUV DATA HERE
			// buffer->user_data points to the data, with
			// buffer->length being the length of the data.
		}

		mmal_buffer_header_release(buffer);
		buffers_to_isp_op(yuv_cb);
	}

	return NULL;
}

/**
 * Creates the queue feeding a worker thread, then the thread itself.
 * The queue is created first so that the thread never sees a NULL queue.
 *
 * @return 0 on success, non-0 otherwise
 */
static int start_worker(VCOS_THREAD_T *thread, const char *name, void *(*entry)(void *), void *arg,
			MMAL_QUEUE_T **queue)
{
	printf("Setup %s thread\n", name);

	*queue = mmal_queue_create();
	if (!*queue)
	{
		printf("Failed to create %s queue\n", name);
		return -1;
	}
	if (vcos_thread_create(thread, name, NULL, entry, arg) != VCOS_SUCCESS)
	{
		printf("Failed to create %s thread\n", name);
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------------
 * Command line
 * ------------------------------------------------------------------------ */

static void set_default_params(RASPIRAW_PARAMS_T *cfg)
{
	memset(cfg, 0, sizeof(*cfg));

	cfg->exposure = -1;
	cfg->gain = -1;
	cfg->timeout = 5000;
	cfg->saverate = 20;
	cfg->bit_depth = -1;
	cfg->camera_num = -1;
	cfg->exposure_us = -1;
	cfg->i2c_bus = -1;
	cfg->fps = -1;
	cfg->crop.hinc = -1;
	cfg->crop.vinc = -1;
	cfg->crop.width = -1;
	cfg->crop.height = -1;
	cfg->crop.left = -1;
	cfg->crop.top = -1;
	cfg->opacity = 255;
	cfg->fullscreen = 1;
}

/*
 * Option value parsers. They all return non-zero on success, and treat a
 * missing value (arg == NULL) as a failure.
 */

static int parse_int(const char *arg, int *value)
{
	return arg && sscanf(arg, "%d", value) == 1;
}

static int parse_double(const char *arg, double *value)
{
	return arg && sscanf(arg, "%lf", value) == 1;
}

/* Exactly two hex digits */
static int parse_hex_byte(const char *arg, int *value)
{
	unsigned int byte;

	if (!arg || strlen(arg) != 2 || sscanf(arg, "%x", &byte) != 1)
		return 0;
	*value = byte;
	return 1;
}

static int parse_string(const char *arg, char **value)
{
	if (!arg)
		return 0;
	*value = strdup(arg);
	vcos_assert(*value);
	return *value != NULL;
}

/* The pattern is passed to sprintf with the frame number, so any %<char> must be either %% or %d. */
static int is_valid_filename_pattern(const char *pattern)
{
	const char *percent = pattern;

	while ((percent = strchr(percent, '%')) != NULL)
	{
		int digits = 0;
		percent++;
		while (isdigit((unsigned char)*percent))
		{
			percent++;
			digits++;
		}
		if (!((*percent == '%' && !digits) || *percent == 'd'))
			return 0;
		percent++;
	}
	return 1;
}

static int parse_filename_pattern(const char *arg, char **value)
{
	if (!arg || !arg[0])
		return 0;
	if (!is_valid_filename_pattern(arg))
	{
		fprintf(stderr, "Filename contains %% characters, but not %%d or %%%% - sorry, will fail\n");
		return 0;
	}
	*value = strdup(arg);
	if (!*value)
	{
		fprintf(stderr, "internal error - allocation fail\n");
		return 0;
	}
	return 1;
}

static int parse_camera_num(const char *arg, int *camera_num)
{
	if (!parse_int(arg, camera_num))
		return 0;
	if (*camera_num != 0 && *camera_num != 1)
	{
		fprintf(stderr, "Invalid camera number specified (%d). It should be 0 or 1.\n", *camera_num);
		return 0;
	}
	return 1;
}

static int parse_awb_gains(const char *arg, RASPIRAW_PARAMS_T *cfg)
{
	double r, b;

	if (!arg || sscanf(arg, "%lf,%lf", &r, &b) != 2 || r > 8.0 || b > 8.0)
		return 0;
	cfg->awb_gains_r = r;
	cfg->awb_gains_b = b;
	return 1;
}

static int parse_preview_window(const char *arg, RASPIRAW_PARAMS_T *cfg)
{
	MMAL_RECT_T *win = &cfg->preview_window;

	if (!arg)
		return 0;
	// If we fail to get the window parameters, revert to full screen
	cfg->fullscreen = sscanf(arg, "%d,%d,%d,%d", &win->x, &win->y, &win->width, &win->height) != 4;
	return 1;
}

static int parse_timestamps_file(const char *arg, RASPIRAW_PARAMS_T *cfg)
{
	if (!parse_string(arg, &cfg->write_timestamps))
		return 0;
	cfg->ptsa = malloc(sizeof(*cfg->ptsa));
	cfg->ptso = cfg->ptsa;
	return 1;
}

/**
 * Handles the options that take no value.
 *
 * @return non-0 if command_id was such an option, 0 otherwise
 */
static int apply_flag_option(RASPIRAW_PARAMS_T *cfg, int command_id)
{
	switch (command_id)
	{
	case CommandHFlip:
		cfg->hflip = 1;
		return 1;
	case CommandVFlip:
		cfg->vflip = 1;
		return 1;
	case CommandWriteHeader:
		cfg->write_header = 1;
		return 1;
	case CommandWriteEmpty:
		cfg->write_empty = 1;
		return 1;
	case CommandDecodeMetadata:
		cfg->decodemetadata = 1;
		return 1;
	case CommandAwb:
		vcos_log_error("Let's do AWB");
		cfg->awb = 1;
		return 1;
	case CommandNoPreview:
		cfg->no_preview = 1;
		return 1;
	case CommandProcessing:
		cfg->processing = 1;
		return 1;
	case CommandProcessingYUV:
		cfg->processing_yuv = 1;
		return 1;
	case CommandFullScreen: // overrides the preview window rect
		cfg->fullscreen = 1;
		return 1;
	default:
		return 0;
	}
}

/**
 * Handles the options that take a value.
 *
 * @param arg The option value, NULL if it is missing
 * @return non-0 if the option and its value are valid, 0 otherwise
 */
static int apply_value_option(RASPIRAW_PARAMS_T *cfg, int command_id, const char *arg)
{
	switch (command_id)
	{
	case CommandMode:
		return parse_int(arg, &cfg->mode);
	case CommandExposure:
		return parse_int(arg, &cfg->exposure);
	case CommandExposureus:
		return parse_int(arg, &cfg->exposure_us);
	case CommandGain:
		return parse_int(arg, &cfg->gain);
	case CommandTimeout: // Time to run for in milliseconds
		return parse_int(arg, &cfg->timeout);
	case CommandSaveRate:
		return parse_int(arg, &cfg->saverate) && cfg->saverate > 0;
	case CommandBitDepth:
		return parse_int(arg, &cfg->bit_depth);
	case CommandCameraNum:
		return parse_camera_num(arg, &cfg->camera_num);
	case CommandI2cBus:
		return parse_int(arg, &cfg->i2c_bus);
	case CommandAwbGains:
		return parse_awb_gains(arg, cfg);
	case CommandRegs:
		return parse_string(arg, &cfg->regs);
	case CommandFps:
		return parse_double(arg, &cfg->fps);
	case CommandHinc:
		return parse_hex_byte(arg, &cfg->crop.hinc);
	case CommandVinc:
		return parse_hex_byte(arg, &cfg->crop.vinc);
	case CommandWidth:
		return parse_int(arg, &cfg->crop.width);
	case CommandHeight:
		return parse_int(arg, &cfg->crop.height);
	case CommandLeft:
		return parse_int(arg, &cfg->crop.left);
	case CommandTop:
		return parse_int(arg, &cfg->crop.top);
	case CommandOutput:
		cfg->capture = parse_filename_pattern(arg, &cfg->output);
		return cfg->capture;
	case CommandOutputYUV:
		cfg->capture_yuv = parse_filename_pattern(arg, &cfg->output_yuv);
		return cfg->capture_yuv;
	case CommandWriteHeader0:
		return parse_string(arg, &cfg->write_header0);
	case CommandWriteHeaderG:
		return parse_string(arg, &cfg->write_headerg);
	case CommandWriteTimestamps:
		return parse_timestamps_file(arg, cfg);
	case CommandPreview:
		return parse_preview_window(arg, cfg);
	case CommandOpacity:
		return parse_int(arg, &cfg->opacity);
	default:
		return 0;
	}
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param cfg Pointer to state structure to assign any discovered parameters to
 * @return non-0 if failed for some reason, 0 otherwise
 */
static int parse_cmdline(int argc, char **argv, RASPIRAW_PARAMS_T *cfg)
{
	// We are looking for --<something> or -<abbreviation of something>
	int i;

	for (i = 1; i < argc; i++)
	{
		int command_id, num_parameters;

		if (argv[i][0] != '-')
			break;

		command_id =
		    raspicli_get_command_id(cmdline_commands, cmdline_commands_size, &argv[i][1], &num_parameters);

		if (command_id == CommandHelp)
		{
			raspicli_display_help(cmdline_commands, cmdline_commands_size);
			// exit straight away if help requested
			return -1;
		}

		if (apply_flag_option(cfg, command_id))
			continue;

		// argv[argc] is NULL, so a missing value is seen as NULL by the parsers
		if (!apply_value_option(cfg, command_id, argv[i + 1]))
			break;
		i++; // skip the value
	}

	if (i < argc)
	{
		fprintf(stderr, "Invalid command line option (%s)\n", argv[i]);
		return 1;
	}

	return 0;
}

/* ------------------------------------------------------------------------
 * Sensor mode setup
 * ------------------------------------------------------------------------ */

/**
 * Applies the user's register changes to the mode's register table.
 * Format is "<reg>,<bytes>[;<reg>,<bytes>...]", with reg as 4 hex digits
 * and bytes as pairs of hex digits written to consecutive registers,
 * e.g. "380A,0040;3802,78". The string is modified by the parsing.
 */
static void apply_reg_overrides(struct mode_def *mode, char *regs)
{
	unsigned int r, b;
	char *p, *q;

	for (p = strtok(regs, ";"); p; p = strtok(NULL, ";"))
	{
		vcos_assert(strlen(p) > 6);
		vcos_assert(p[4] == ',');
		vcos_assert(strlen(p) % 2);
		p[4] = '\0';
		sscanf(p, "%4x", &r);

		for (q = p + 5; *q; q += 2, r++)
		{
			vcos_assert(isxdigit((unsigned char)q[0]));
			vcos_assert(isxdigit((unsigned char)q[1]));

			sscanf(q, "%2x", &b);
			vcos_log_error("%04x: %02x", r, b);

			modReg(mode, r, 0, 7, b, EQUAL);
		}
	}
}

static int crop_requested(const struct raspiraw_crop *crop)
{
	return crop->hinc >= 0 || crop->vinc >= 0 || crop->width > 0 || crop->height > 0 || crop->top >= 0 ||
	       crop->left >= 0;
}

static int apply_crop(const struct sensor_def *sensor, struct mode_def *mode, const struct raspiraw_crop *crop)
{
	if (!sensor->set_crop)
	{
		vcos_log_error("This sensor does not currently support manual cropping settings. Aborting");
		return -1;
	}
	if (sensor->set_crop(sensor, mode, crop))
	{
		vcos_log_error("Failed setting manual crops. Aborting");
		return -1;
	}
	return 0;
}

static void apply_fps(const struct sensor_def *sensor, struct mode_def *mode, double fps)
{
	int vts = 1000000000 / (mode->line_time_ns * fps);

	modReg(mode, sensor->vts_reg + 0, 0, 7, vts >> 8, EQUAL);
	modReg(mode, sensor->vts_reg + 1, 0, 7, vts & 0xFF, EQUAL);
}

/**
 * Applies all the command line settings to the mode's register table,
 * and resolves the output bit depth and exposure in cfg.
 *
 * @return 0 on success, non-0 otherwise
 */
static int configure_sensor_mode(const struct sensor_def *sensor, struct mode_def *mode, RASPIRAW_PARAMS_T *cfg)
{
	if (cfg->regs)
		apply_reg_overrides(mode, cfg->regs);

	if (crop_requested(&cfg->crop) && apply_crop(sensor, mode, &cfg->crop))
		return -1;

	if (cfg->fps > 0)
		apply_fps(sensor, mode, cfg->fps);

	if (cfg->bit_depth == -1)
		cfg->bit_depth = mode->native_bit_depth;

	if (cfg->write_headerg && (cfg->bit_depth != mode->native_bit_depth))
	{
		// needs change after fix for https://github.com/6by9/raspiraw/issues/2
		vcos_log_error("--headerG supported for native bit depth only");
		return -1;
	}

	if (cfg->exposure_us != -1)
	{
		cfg->exposure = ((int64_t)cfg->exposure_us * 1000) / mode->line_time_ns;
		vcos_log_error("Setting exposure to %d from time %dus", cfg->exposure, cfg->exposure_us);
	}

	update_regs(sensor, mode, cfg->hflip, cfg->vflip, cfg->exposure, cfg->gain);
	return 0;
}

/* ------------------------------------------------------------------------
 * MMAL pipeline: rawcam -> isp -> video render
 * ------------------------------------------------------------------------ */

static MMAL_STATUS_T create_component(const char *name, MMAL_COMPONENT_T **component)
{
	MMAL_STATUS_T status = mmal_component_create(name, component);
	if (status != MMAL_SUCCESS)
		vcos_log_error("Failed to create %s", name);
	return status;
}

static MMAL_STATUS_T enable_component(MMAL_COMPONENT_T *component)
{
	MMAL_STATUS_T status = mmal_component_enable(component);
	if (status != MMAL_SUCCESS)
		vcos_log_error("Failed to enable %s", component->name);
	return status;
}

static void disable_component(MMAL_COMPONENT_T *component)
{
	if (component && mmal_component_disable(component) != MMAL_SUCCESS)
		vcos_log_error("Failed to disable %s", component->name);
}

static MMAL_STATUS_T enable_zero_copy(MMAL_PORT_T *port)
{
	MMAL_STATUS_T status = mmal_port_parameter_set_boolean(port, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if (status != MMAL_SUCCESS)
		vcos_log_error("Failed to set zero copy on %s", port->name);
	return status;
}

static MMAL_STATUS_T create_components(RASPIRAW_PIPELINE_T *pipeline)
{
	MMAL_STATUS_T status;

	status = create_component("vc.ril.rawcam", &pipeline->rawcam);
	if (status == MMAL_SUCCESS)
		status = enable_zero_copy(pipeline->rawcam->output[0]);

	if (status == MMAL_SUCCESS)
		status = create_component("vc.ril.isp", &pipeline->isp);
	if (status == MMAL_SUCCESS)
		status = enable_zero_copy(pipeline->isp->input[0]);
	if (status == MMAL_SUCCESS)
		status = enable_zero_copy(pipeline->isp->output[0]);

	if (status == MMAL_SUCCESS)
		status = create_component(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &pipeline->render);
	if (status == MMAL_SUCCESS)
		status = enable_zero_copy(pipeline->render->input[0]);

	return status;
}

/* Sets the CSI-2 receiver unpacking/packing, number of data lanes and image id. */
static MMAL_STATUS_T configure_rx(MMAL_PORT_T *output, const struct mode_def *mode, int bit_depth)
{
	MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg = { { MMAL_PARAMETER_CAMERA_RX_CONFIG, sizeof(rx_cfg) } };
	MMAL_STATUS_T status;

	status = mmal_port_parameter_get(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to get cfg");
		return status;
	}

	if (mode->encoding || bit_depth == mode->native_bit_depth)
	{
		rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
		rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_NONE;
	}
	else
	{
		rx_cfg.unpack = bit_depth_to_rx_unpack(mode->native_bit_depth);
		rx_cfg.pack = bit_depth_to_rx_pack(bit_depth);
	}
	vcos_log_error("Set unpack to %d, pack to %d", rx_cfg.unpack, rx_cfg.pack);

	if (mode->data_lanes)
		rx_cfg.data_lanes = mode->data_lanes;
	if (mode->image_id)
		rx_cfg.image_id = mode->image_id;

	status = mmal_port_parameter_set(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
		vcos_log_error("Failed to set cfg");
	return status;
}

/* Overrides the CSI-2 receiver timings with any that the mode specifies. */
static MMAL_STATUS_T configure_rx_timing(MMAL_PORT_T *output, const struct mode_def *mode)
{
	MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing = { { MMAL_PARAMETER_CAMERA_RX_TIMING, sizeof(rx_timing) } };
	MMAL_STATUS_T status;

	status = mmal_port_parameter_get(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to get timing");
		return status;
	}

	if (mode->timing[0])
		rx_timing.timing1 = mode->timing[0];
	if (mode->timing[1])
		rx_timing.timing2 = mode->timing[1];
	if (mode->timing[2])
		rx_timing.timing3 = mode->timing[2];
	if (mode->timing[3])
		rx_timing.timing4 = mode->timing[3];
	if (mode->timing[4])
		rx_timing.timing5 = mode->timing[4];
	if (mode->term[0])
		rx_timing.term1 = mode->term[0];
	if (mode->term[1])
		rx_timing.term2 = mode->term[1];
	vcos_log_error("Timing %u/%u, %u/%u/%u, %u/%u", rx_timing.timing1, rx_timing.timing2, rx_timing.timing3,
		       rx_timing.timing4, rx_timing.timing5, rx_timing.term1, rx_timing.term2);

	status = mmal_port_parameter_set(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
		vcos_log_error("Failed to set timing");
	return status;
}

static MMAL_STATUS_T configure_rawcam(MMAL_PORT_T *output, const struct mode_def *mode, const RASPIRAW_PARAMS_T *cfg)
{
	MMAL_STATUS_T status;

	status = configure_rx(output, mode, cfg->bit_depth);
	if (status == MMAL_SUCCESS)
		status = configure_rx_timing(output, mode);

	if (status == MMAL_SUCCESS && cfg->camera_num != -1)
	{
		vcos_log_error("Set camera_num to %d", cfg->camera_num);
		status = mmal_port_parameter_set_int32(output, MMAL_PARAMETER_CAMERA_NUM, cfg->camera_num);
		if (status != MMAL_SUCCESS)
			vcos_log_error("Failed to set camera_num");
	}
	return status;
}

static MMAL_STATUS_T set_rawcam_format(MMAL_PORT_T *output, const struct mode_def *mode, uint32_t encoding)
{
	MMAL_STATUS_T status;

	output->format->es->video.crop.width = mode->width;
	output->format->es->video.crop.height = mode->height;
	output->format->es->video.width = VCOS_ALIGN_UP(mode->width, 16);
	output->format->es->video.height = VCOS_ALIGN_UP(mode->height, 16);
	output->format->encoding = encoding;

	status = mmal_port_format_commit(output);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed port_format_commit");
		return status;
	}

	output->buffer_size = output->buffer_size_recommended;
	output->buffer_num = output->buffer_num_recommended;
	return MMAL_SUCCESS;
}

/* The isp input takes whatever rawcam produces. */
static MMAL_STATUS_T set_isp_input_format(MMAL_PORT_T *isp_input, MMAL_PORT_T *rawcam_output)
{
	MMAL_STATUS_T status;

	status = mmal_format_full_copy(isp_input->format, rawcam_output->format);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to copy port format");
		return status;
	}
	status = mmal_port_format_commit(isp_input);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to commit port format on isp input");
		return status;
	}

	isp_input->buffer_num = rawcam_output->buffer_num;

	if (isp_input->buffer_size != rawcam_output->buffer_size)
	{
		vcos_log_error("rawcam output and isp input are different sizes - this could end badly");
		vcos_log_error("rawcam %u, isp %u", rawcam_output->buffer_size, isp_input->buffer_size);
	}
	return MMAL_SUCCESS;
}

/* The isp outputs I420, scaled down until it is small enough for the display. */
static MMAL_STATUS_T set_isp_output_format(MMAL_PORT_T *isp_output, const struct mode_def *mode)
{
	MMAL_VIDEO_FORMAT_T *video = &isp_output->format->es->video;
	MMAL_STATUS_T status;

	video->crop.width = mode->width;
	video->crop.height = mode->height;
	while (video->crop.width > 1920)
	{
		// Display can only go up to a certain resolution before underflowing
		video->crop.width /= 2;
		video->crop.height /= 2;
	}
	video->width = VCOS_ALIGN_UP(video->crop.width, 32);
	video->height = VCOS_ALIGN_UP(video->crop.height, 16);
	isp_output->format->encoding = MMAL_ENCODING_I420;
	isp_output->buffer_num = 6; // Go for 6 output buffers to give some slack

	status = mmal_port_format_commit(isp_output);
	if (status != MMAL_SUCCESS)
		vcos_log_error("Failed to commit port format on isp output");
	return status;
}

/* Sets the isp black level and white balance gains. Failures here are not fatal. */
static void set_isp_tuning(MMAL_PORT_T *isp_input, const struct mode_def *mode, const RASPIRAW_PARAMS_T *cfg)
{
	if (mode->black_level)
	{
		if (mmal_port_parameter_set_uint32(isp_input, MMAL_PARAMETER_BLACK_LEVEL, mode->black_level) !=
		    MMAL_SUCCESS)
			vcos_log_error("Failed to set black level - try updating firmware");
	}

	if (cfg->awb_gains_r && cfg->awb_gains_b)
	{
		MMAL_PARAMETER_AWB_GAINS_T param = { { MMAL_PARAMETER_CUSTOM_AWB_GAINS, sizeof(param) },
						     { 0, 0 },
						     { 0, 0 } };

		param.r_gain.num = (unsigned int)(cfg->awb_gains_r * 65536);
		param.b_gain.num = (unsigned int)(cfg->awb_gains_b * 65536);
		param.r_gain.den = param.b_gain.den = 65536;
		if (mmal_port_parameter_set(isp_input, &param.hdr) != MMAL_SUCCESS)
			vcos_log_error("Failed to set white balance");
	}
}

static MMAL_STATUS_T configure_isp(MMAL_COMPONENT_T *isp, MMAL_PORT_T *rawcam_output, const struct mode_def *mode,
				   const RASPIRAW_PARAMS_T *cfg)
{
	MMAL_STATUS_T status;

	status = set_isp_input_format(isp->input[0], rawcam_output);
	if (status == MMAL_SUCCESS)
		status = set_isp_output_format(isp->output[0], mode);
	if (status == MMAL_SUCCESS)
		set_isp_tuning(isp->input[0], mode, cfg);
	return status;
}

/* Backs a buffer header with VCSM shared memory that both the GPU and we can access. */
static void attach_vcsm_memory(MMAL_BUFFER_HEADER_T *buffer, uint32_t size)
{
	unsigned int vcsm_handle = vcsm_malloc_cache(size, VCSM_CACHE_TYPE_HOST, "mmal_vc_port buffer");
	unsigned int vc_handle = vcsm_vc_hdl_from_hdl(vcsm_handle);
	uint8_t *mem = (uint8_t *)vcsm_lock(vcsm_handle);

	if (!mem || !vc_handle)
	{
		vcos_log_error("could not allocate %i bytes of shared memory (handle %x)", (int)size, vcsm_handle);
		if (mem)
			vcsm_unlock_hdl(vcsm_handle);
		if (vcsm_handle)
			vcsm_free(vcsm_handle);
		return;
	}

	buffer->data = (void *)vc_handle;
	buffer->alloc_size = size;
	buffer->user_data = mem;
}

/**
 * Creates a pool of buffer headers for a port, backed by VCSM shared memory.
 * The pool is created manually so that we have control over the mem handles,
 * not letting MMAL core handle the magic.
 */
static MMAL_POOL_T *create_vcsm_pool(MMAL_PORT_T *port)
{
	MMAL_POOL_T *pool;
	unsigned int i;

	vcos_log_error("Create pool of %u buffers of size %u", port->buffer_num, port->buffer_size);
	pool = mmal_port_pool_create(port, port->buffer_num, 0);
	if (!pool)
		return NULL;

	for (i = 0; i < port->buffer_num; i++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);
		if (!buffer)
		{
			vcos_log_error("Where did my buffer go?");
			continue;
		}
		attach_vcsm_memory(buffer, port->buffer_size);
		mmal_buffer_header_release(buffer);
	}
	return pool;
}

/* Creates the rawcam output pool, and the pool of headers used to replicate its buffers to the isp. */
static int setup_rawcam_buffers(RASPIRAW_PIPELINE_T *pipeline, RASPIRAW_CALLBACK_T *dev)
{
	MMAL_PORT_T *output = pipeline->rawcam->output[0];
	MMAL_PORT_T *isp_input = pipeline->isp->input[0];

	pipeline->rawcam_pool = create_vcsm_pool(output);
	if (!pipeline->rawcam_pool)
	{
		vcos_log_error("Failed to create pool");
		return -1;
	}
	dev->rawcam_output = output;
	dev->rawcam_pool = pipeline->rawcam_pool;

	vcos_log_error("Create pool of %u buffers of size %u", isp_input->buffer_num, isp_input->buffer_size);
	dev->isp_ip_pool = mmal_port_pool_create(isp_input, isp_input->buffer_num, 0);
	if (!dev->isp_ip_pool)
	{
		vcos_log_error("Failed to create isp_ip_pool");
		return -1;
	}
	dev->isp_ip = isp_input;

	isp_input->userdata = (struct MMAL_PORT_USERDATA_T *)dev;
	if (mmal_port_enable(isp_input, isp_ip_cb) != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable isp ip port");
		return -1;
	}
	return 0;
}

/* Creates the pool for the YUV images coming out of the isp. */
static int setup_isp_output_buffers(RASPIRAW_PIPELINE_T *pipeline, RASPIRAW_ISP_CALLBACK_T *yuv_cb)
{
	MMAL_PORT_T *isp_output = pipeline->isp->output[0];
	MMAL_POOL_T *yuv_pool;

	yuv_pool = create_vcsm_pool(isp_output);
	if (!yuv_pool)
	{
		vcos_log_error("Failed to create yuv pool");
		return -1;
	}

	isp_output->userdata = (struct MMAL_PORT_USERDATA_T *)yuv_cb;
	if (mmal_port_enable(isp_output, yuv_callback) != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable isp op port");
		return -1;
	}
	yuv_cb->isp_output = isp_output;
	yuv_cb->isp_op_pool = yuv_pool;
	return 0;
}

static void set_display_region(MMAL_PORT_T *render_input, const RASPIRAW_PARAMS_T *cfg)
{
	MMAL_DISPLAYREGION_T param;
	MMAL_STATUS_T status;

	memset(&param, 0, sizeof(param));
	param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
	param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);

	param.set = MMAL_DISPLAY_SET_LAYER;
	param.layer = DEFAULT_PREVIEW_LAYER;

	param.set |= MMAL_DISPLAY_SET_ALPHA;
	param.alpha = cfg->opacity;

	if (cfg->fullscreen)
	{
		param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
		param.fullscreen = 1;
	}
	else
	{
		param.set |= (MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_FULLSCREEN);
		param.fullscreen = 0;
		param.dest_rect = cfg->preview_window;
	}

	status = mmal_port_parameter_set(render_input, &param.hdr);
	if (status != MMAL_SUCCESS && status != MMAL_ENOSYS)
		vcos_log_error("unable to set preview port parameters (%u)", status);
}

/* Connects the video render to the isp output to show the stream on the display. */
static int setup_preview(RASPIRAW_PIPELINE_T *pipeline, RASPIRAW_ISP_CALLBACK_T *yuv_cb, const RASPIRAW_PARAMS_T *cfg)
{
	MMAL_PORT_T *isp_output = pipeline->isp->output[0];
	MMAL_PORT_T *render_input = pipeline->render->input[0];

	set_display_region(render_input, cfg);

	if (mmal_format_full_copy(render_input->format, isp_output->format) != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to copy port format - isp to render");
		return -1;
	}
	if (mmal_port_format_commit(render_input) != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to commit port format on render input");
		return -1;
	}

	render_input->buffer_num = isp_output->buffer_num;

	yuv_cb->vr_ip_pool = mmal_port_pool_create(render_input, render_input->buffer_num, 0);
	if (!yuv_cb->vr_ip_pool)
	{
		vcos_log_error("Failed to create vr_ip_pool");
		return -1;
	}
	yuv_cb->vr_ip = render_input;

	render_input->userdata = (struct MMAL_PORT_USERDATA_T *)yuv_cb;
	if (mmal_port_enable(render_input, vr_ip_cb) != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable vr ip port");
		return -1;
	}
	return 0;
}

/**
 * Creates and configures the whole rawcam -> isp -> render pipeline,
 * leaving it ready to receive frames as soon as the sensor starts streaming.
 *
 * @return 0 on success, non-0 otherwise. In both cases the pipeline has to be
 *         released with destroy_pipeline().
 */
static int setup_pipeline(RASPIRAW_PIPELINE_T *pipeline, RASPIRAW_CALLBACK_T *dev, RASPIRAW_ISP_CALLBACK_T *yuv_cb,
			  const struct mode_def *mode, uint32_t encoding)
{
	const RASPIRAW_PARAMS_T *cfg = dev->cfg;
	MMAL_PORT_T *output;

	if (create_components(pipeline) != MMAL_SUCCESS)
		return -1;
	output = pipeline->rawcam->output[0];

	if (configure_rawcam(output, mode, cfg) != MMAL_SUCCESS)
		return -1;

	if (enable_component(pipeline->rawcam) != MMAL_SUCCESS || enable_component(pipeline->isp) != MMAL_SUCCESS ||
	    enable_component(pipeline->render) != MMAL_SUCCESS)
		return -1;

	if (set_rawcam_format(output, mode, encoding) != MMAL_SUCCESS)
		return -1;

	prepare_capture_headers(cfg, mode);

	if (configure_isp(pipeline->isp, output, mode, cfg) != MMAL_SUCCESS)
		return -1;

	if (setup_rawcam_buffers(pipeline, dev))
		return -1;
	if (setup_isp_output_buffers(pipeline, yuv_cb))
		return -1;

	if (!cfg->no_preview && setup_preview(pipeline, yuv_cb, cfg))
		return -1;

	if (cfg->processing_yuv)
	{
		if (start_worker(&pipeline->processing_yuv_thread, "processing-yuv", processing_yuv_thread_task, yuv_cb,
				 &yuv_cb->processing_yuv_queue))
			return -1;
		pipeline->processing_yuv_thread_started = 1;
	}

	output->userdata = (struct MMAL_PORT_USERDATA_T *)dev;
	if (mmal_port_enable(output, rawcam_callback) != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable port");
		return -1;
	}

	buffers_to_rawcam(dev);
	buffers_to_isp_op(yuv_cb);
	return 0;
}

/* Safe to call on a partially set up pipeline. */
static void destroy_pipeline(RASPIRAW_PIPELINE_T *pipeline)
{
	if (pipeline->rawcam_pool)
		mmal_port_pool_destroy(pipeline->rawcam->output[0], pipeline->rawcam_pool);

	free(brcm_header);
	brcm_header = NULL;

	disable_component(pipeline->render);
	disable_component(pipeline->isp);
	disable_component(pipeline->rawcam);

	if (pipeline->rawcam)
		mmal_component_destroy(pipeline->rawcam);
	if (pipeline->isp)
		mmal_component_destroy(pipeline->isp);
	if (pipeline->render)
		mmal_component_destroy(pipeline->render);
}

/* Streams from the sensor for the configured time. */
static int run_capture(RASPIRAW_PIPELINE_T *pipeline, const RASPIRAW_PARAMS_T *cfg, const struct sensor_def *sensor,
		       struct mode_def *mode, int i2c_fd)
{
	start_camera_streaming(sensor, mode, i2c_fd);

	vcos_sleep(cfg->timeout);

	stop_camera_streaming(sensor, i2c_fd);

	if (cfg->capture && mmal_port_disable(pipeline->rawcam->output[0]) != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to disable port");
		return -1;
	}
	return 0;
}

static uint32_t select_encoding(const struct mode_def *mode, int bit_depth)
{
	uint32_t encoding = mode->encoding;

	if (!encoding)
		encoding = order_and_bit_depth_to_encoding(mode->order, bit_depth);
	if (!encoding)
		vcos_log_error("Failed to map bitdepth %d and order %d into encoding\n", bit_depth, mode->order);
	else
		vcos_log_error("Encoding %08X", encoding);
	return encoding;
}

int main(int argc, char **argv)
{
	RASPIRAW_PARAMS_T cfg;
	RASPIRAW_CALLBACK_T dev = { .cfg = &cfg };
	RASPIRAW_ISP_CALLBACK_T yuv_cb = { .cfg = &cfg };
	RASPIRAW_PIPELINE_T pipeline = { 0 };
	const struct sensor_def *sensor;
	struct mode_def *sensor_mode;
	VCOS_THREAD_T awb_thread;
	VCOS_THREAD_T processing_thread;
	uint32_t encoding;
	int i2c_fd;
	int ret;

	set_default_params(&cfg);

	bcm_host_init();
	vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

	if (argc == 1)
	{
		fprintf(stdout, "\n%s Camera App %s\n\n", basename(argv[0]), VERSION_STRING);

		raspicli_display_help(cmdline_commands, cmdline_commands_size);
		exit(-1);
	}

	// Parse the command line and put options in to our status structure
	if (parse_cmdline(argc, argv, &cfg))
	{
		exit(-1);
	}

	i2c_fd = open_i2c_device(cfg.i2c_bus);
	if (i2c_fd < 0)
		return -1;

	sensor = probe_sensor(i2c_fd);
	if (!sensor)
	{
		vcos_log_error("No sensor found. Aborting");
		return -1;
	}

	if (cfg.mode < 0 || cfg.mode >= sensor->num_modes)
	{
		vcos_log_error("Invalid mode %d - aborting", cfg.mode);
		return -2;
	}
	sensor_mode = &sensor->modes[cfg.mode];

	if (configure_sensor_mode(sensor, sensor_mode, &cfg))
		return -1;

	encoding = select_encoding(sensor_mode, cfg.bit_depth);
	if (!encoding)
		return -3;

	if (cfg.awb)
	{
		if (start_worker(&awb_thread, "awb", awb_thread_task, &dev, &dev.awb_queue))
			return -4;
	}
	else
		printf("No AWB\n");

	if (cfg.processing)
	{
		if (start_worker(&processing_thread, "processing", processing_thread_task, &dev,
				 &dev.processing_queue))
			return -4;
	}

	ret = setup_pipeline(&pipeline, &dev, &yuv_cb, sensor_mode, encoding);
	if (!ret)
		ret = run_capture(&pipeline, &cfg, sensor, sensor_mode, i2c_fd);

	destroy_pipeline(&pipeline);

	if (cfg.awb)
	{
		dev.awb_thread_quit = 1;
		vcos_thread_join(&awb_thread, NULL);
	}

	if (cfg.processing)
	{
		dev.processing_thread_quit = 1;
		vcos_thread_join(&processing_thread, NULL);
	}

	if (pipeline.processing_yuv_thread_started)
	{
		yuv_cb.processing_yuv_thread_quit = 1;
		vcos_thread_join(&pipeline.processing_yuv_thread, NULL);
	}

	if (cfg.write_timestamps)
	{
		write_timestamps(&cfg);
		free_timestamps(&cfg);
	}
	close(i2c_fd);

	return ret;
}
