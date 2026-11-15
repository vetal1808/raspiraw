/*
 * Mock of the VideoCore userland libraries (bcm_host, vcos, mmal, vcsm) and of
 * the sensor's I2C bus, so that raspiraw can be linked and run on a machine
 * without the firmware camera stack (e.g. a Pi 5).
 *
 * Environment:
 *   MOCK_SENSOR     ov5647 (default) | imx219 | imx477 | none
 *   MOCK_I2C_BUSES  comma separated list of existing bus numbers (default "10")
 *   MOCK_FPS        frame rate of the fake sensor (default 30)
 *   MOCK_METADATA   1 = deliver an embedded-data buffer before every frame
 *   MOCK_FAIL       checkpoint to fail, e.g. create:vc.ril.isp, commit:isp:in:0,
 *                   enable:rawcam:out:0, pool:isp:in:0, cenable:vc.ril.isp
 *   MOCK_LOG        file receiving the deterministic event log (I2C, params, ...)
 *   MOCK_NO_FLUSH   1 = do not hand unused buffers back when a port is disabled
 *   MOCK_FRAME_FILE frame from a real sensor to use as image: 16 bit little endian,
 *                   10 significant bits in the MSBs, no padding (rpicam-raw --mode W:H:10:U)
 *   MOCK_AWB_VERBOSE 1 = print every set of AWB gains to stderr, not only the first
 *
 * Problems the mock itself detects are printed as "MOCK ERROR: ...".
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/vcsm/user-vcsm.h"

/* ------------------------------------------------------------------ log */

static FILE *mock_log;
static pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;
static int mock_errors;

static struct
{
	long frames_made, frames_dropped, metadata_made, rawcam_sent, isp_in, isp_out, render_in;
	long flushed, send_disabled;
	long vcsm_live;
} stats;

static void mlog(const char *fmt, ...)
{
	va_list ap;
	if (!mock_log)
		return;
	pthread_mutex_lock(&log_lock);
	va_start(ap, fmt);
	vfprintf(mock_log, fmt, ap);
	va_end(ap);
	fputc('\n', mock_log);
	fflush(mock_log);
	pthread_mutex_unlock(&log_lock);
}

static void merror(const char *fmt, ...)
{
	va_list ap;
	char msg[256];
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);
	__sync_fetch_and_add(&mock_errors, 1);
	fprintf(stderr, "MOCK ERROR: %s\n", msg);
	mlog("MOCKERROR %s", msg);
}

static int should_fail(const char *what, const char *name)
{
	const char *f = getenv("MOCK_FAIL");
	char key[128];
	if (!f)
		return 0;
	snprintf(key, sizeof(key), "%s:%s", what, name);
	return strcmp(f, key) == 0;
}

static void summary(void)
{
	mlog("SUMMARY frames_made=%ld dropped=%ld metadata=%ld rawcam_sent=%ld isp_in=%ld isp_out=%ld render_in=%ld "
	     "flushed=%ld send_disabled=%ld vcsm_live=%ld mock_errors=%d",
	     stats.frames_made, stats.frames_dropped, stats.metadata_made, stats.rawcam_sent, stats.isp_in,
	     stats.isp_out, stats.render_in, stats.flushed, stats.send_disabled, stats.vcsm_live, mock_errors);
}

/* ------------------------------------------------------- bcm_host / vcos */

VCOS_LOG_CAT_T mmal_log_category;

void bcm_host_init(void)
{
	const char *path = getenv("MOCK_LOG");
	if (path && !mock_log)
	{
		mock_log = fopen(path, "w");
		atexit(summary);
	}
}

void vcos_log_register(const char *name, VCOS_LOG_CAT_T *category)
{
	(void)name;
	category->level = VCOS_LOG_ERROR;
}

void vcos_log_impl(const VCOS_LOG_CAT_T *cat, VCOS_LOG_LEVEL_T level, const char *fmt, ...)
{
	va_list ap;
	(void)cat;
	(void)level;
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
}

void vcos_pthreads_logging_assert(const char *file, const char *func, unsigned int line, const char *fmt, ...)
{
	va_list ap;
	fprintf(stderr, "ASSERT %s:%u %s: ", file, line, func);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
	abort();
}

VCOS_STATUS_T vcos_thread_create(VCOS_THREAD_T *thread, const char *name, VCOS_THREAD_ATTR_T *attrs,
				 VCOS_THREAD_ENTRY_FN_T entry, void *arg)
{
	(void)name;
	(void)attrs;
	memset(thread, 0, sizeof(*thread));
	if (pthread_create(&thread->thread, NULL, entry, arg))
		return VCOS_ENOMEM;
	return VCOS_SUCCESS;
}

void vcos_thread_join(VCOS_THREAD_T *thread, void **pdata)
{
	pthread_join(thread->thread, pdata);
}

/* ----------------------------------------------------------------- vcsm */

#define MAX_VCSM 256
#define VC_HANDLE_BASE 0x4000
static struct
{
	void *mem;
	unsigned int size;
} vcsm_tab[MAX_VCSM];
static pthread_mutex_t vcsm_mutex = PTHREAD_MUTEX_INITIALIZER;

unsigned int vcsm_malloc_cache(unsigned int size, VCSM_CACHE_TYPE_T cache, const char *name)
{
	unsigned int i, h = 0;
	(void)cache;
	(void)name;
	pthread_mutex_lock(&vcsm_mutex);
	for (i = 0; i < MAX_VCSM && !h; i++)
	{
		if (!vcsm_tab[i].mem)
		{
			vcsm_tab[i].mem = calloc(1, size ? size : 1);
			vcsm_tab[i].size = size;
			h = i + 1;
			stats.vcsm_live++;
		}
	}
	pthread_mutex_unlock(&vcsm_mutex);
	return h;
}

unsigned int vcsm_vc_hdl_from_hdl(unsigned int handle)
{
	return handle ? VC_HANDLE_BASE + handle : 0;
}

void *vcsm_lock(unsigned int handle)
{
	if (!handle || handle > MAX_VCSM)
		return NULL;
	return vcsm_tab[handle - 1].mem;
}

int vcsm_unlock_hdl(unsigned int handle)
{
	(void)handle;
	return 0;
}

void vcsm_free(unsigned int handle)
{
	if (!handle || handle > MAX_VCSM)
		return;
	pthread_mutex_lock(&vcsm_mutex);
	free(vcsm_tab[handle - 1].mem);
	vcsm_tab[handle - 1].mem = NULL;
	stats.vcsm_live--;
	pthread_mutex_unlock(&vcsm_mutex);
}

/* What the "GPU" does with the opaque handle found in buffer->data. */
static uint8_t *vc_handle_to_mem(void *data, unsigned int *size)
{
	uintptr_t h = (uintptr_t)data;
	if (h <= VC_HANDLE_BASE || h > VC_HANDLE_BASE + MAX_VCSM || !vcsm_tab[h - VC_HANDLE_BASE - 1].mem)
		return NULL;
	*size = vcsm_tab[h - VC_HANDLE_BASE - 1].size;
	return vcsm_tab[h - VC_HANDLE_BASE - 1].mem;
}

/* ---------------------------------------------------------------- queue */

struct MMAL_QUEUE_T
{
	pthread_mutex_t lock;
	pthread_cond_t cond;
	MMAL_BUFFER_HEADER_T *first, **last;
	unsigned int length;
};

MMAL_QUEUE_T *mmal_queue_create(void)
{
	MMAL_QUEUE_T *q = calloc(1, sizeof(*q));
	pthread_condattr_t attr;
	if (!q)
		return NULL;
	pthread_mutex_init(&q->lock, NULL);
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	pthread_cond_init(&q->cond, &attr);
	q->last = &q->first;
	return q;
}

void mmal_queue_put(MMAL_QUEUE_T *q, MMAL_BUFFER_HEADER_T *buffer)
{
	pthread_mutex_lock(&q->lock);
	buffer->next = NULL;
	*q->last = buffer;
	q->last = &buffer->next;
	q->length++;
	pthread_cond_signal(&q->cond);
	pthread_mutex_unlock(&q->lock);
}

static MMAL_BUFFER_HEADER_T *queue_pop_locked(MMAL_QUEUE_T *q)
{
	MMAL_BUFFER_HEADER_T *b = q->first;
	if (b)
	{
		q->first = b->next;
		if (!q->first)
			q->last = &q->first;
		q->length--;
		b->next = NULL;
	}
	return b;
}

MMAL_BUFFER_HEADER_T *mmal_queue_get(MMAL_QUEUE_T *q)
{
	MMAL_BUFFER_HEADER_T *b;
	pthread_mutex_lock(&q->lock);
	b = queue_pop_locked(q);
	pthread_mutex_unlock(&q->lock);
	return b;
}

MMAL_BUFFER_HEADER_T *mmal_queue_timedwait(MMAL_QUEUE_T *q, VCOS_UNSIGNED timeout)
{
	MMAL_BUFFER_HEADER_T *b;
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	ts.tv_sec += timeout / 1000;
	ts.tv_nsec += (timeout % 1000) * 1000000L;
	if (ts.tv_nsec >= 1000000000L)
	{
		ts.tv_sec++;
		ts.tv_nsec -= 1000000000L;
	}
	pthread_mutex_lock(&q->lock);
	while (!q->first)
	{
		if (pthread_cond_timedwait(&q->cond, &q->lock, &ts))
			break;
	}
	b = queue_pop_locked(q);
	pthread_mutex_unlock(&q->lock);
	return b;
}

unsigned int mmal_queue_length(MMAL_QUEUE_T *q)
{
	return q->length;
}

/* ------------------------------------------------------- buffer headers */

struct MMAL_BUFFER_HEADER_PRIVATE_T
{
	int refcount;
	MMAL_POOL_T *pool;
	MMAL_BUFFER_HEADER_T *reference;
	MMAL_BUFFER_HEADER_TYPE_SPECIFIC_T type;
};

void mmal_buffer_header_acquire(MMAL_BUFFER_HEADER_T *header)
{
	__sync_fetch_and_add(&header->priv->refcount, 1);
}

void mmal_buffer_header_release(MMAL_BUFFER_HEADER_T *header)
{
	int rc = __sync_sub_and_fetch(&header->priv->refcount, 1);
	MMAL_BUFFER_HEADER_T *ref;

	if (rc < 0)
	{
		merror("buffer header released more often than acquired");
		return;
	}
	if (rc)
		return;

	/* as mmal_buffer_header_reset() + the pool's release: back in the pool with refcount 1 */
	ref = header->priv->reference;
	header->priv->reference = NULL;
	if (ref)
		mmal_buffer_header_release(ref);

	header->length = 0;
	header->offset = 0;
	header->flags = 0;
	header->pts = INT64_MIN;
	header->dts = INT64_MIN;
	header->priv->refcount = 1;
	mmal_queue_put(header->priv->pool->queue, header);
}

MMAL_STATUS_T mmal_buffer_header_replicate(MMAL_BUFFER_HEADER_T *dest, MMAL_BUFFER_HEADER_T *src)
{
	if (dest->priv->reference)
		return MMAL_EINVAL;
	mmal_buffer_header_acquire(src);
	dest->priv->reference = src;
	dest->cmd = src->cmd;
	dest->alloc_size = src->alloc_size;
	dest->data = src->data;
	dest->offset = src->offset;
	dest->length = src->length;
	dest->flags = src->flags;
	dest->pts = src->pts;
	dest->dts = src->dts;
	*dest->type = *src->type;
	return MMAL_SUCCESS;
}

/* ---------------------------------------------------------------- ports */

enum port_kind
{
	RAWCAM_OUT,
	ISP_IN,
	ISP_OUT,
	RENDER_IN,
	CONTROL
};

struct MMAL_PORT_PRIVATE_T
{
	enum port_kind kind;
	MMAL_PORT_BH_CB_T cb;
	MMAL_QUEUE_T *held;
	pthread_t thread;
	int thread_running;
	volatile int quit;
	MMAL_ES_FORMAT_T format;
	MMAL_ES_SPECIFIC_FORMAT_T es;
	char name[32];
};

struct MMAL_COMPONENT_PRIVATE_T
{
	char name[64];
	MMAL_PORT_T *in_arr[1], *out_arr[1];
};

static MMAL_PORT_T *isp_out_port; /* the isp input thread produces into this */

static MMAL_PORT_T *new_port(MMAL_COMPONENT_T *comp, enum port_kind kind, const char *name, MMAL_PORT_TYPE_T type)
{
	MMAL_PORT_T *port = calloc(1, sizeof(*port));
	port->priv = calloc(1, sizeof(*port->priv));
	port->priv->kind = kind;
	port->priv->held = mmal_queue_create();
	snprintf(port->priv->name, sizeof(port->priv->name), "%s", name);
	port->name = port->priv->name;
	port->type = type;
	port->format = &port->priv->format;
	port->format->es = &port->priv->es;
	port->component = comp;
	port->buffer_num_min = 1;
	port->buffer_num_recommended = 1;
	port->buffer_num = 1;
	return port;
}

uint32_t mmal_encoding_width_to_stride(uint32_t encoding, uint32_t width)
{
	switch (encoding)
	{
	case MMAL_ENCODING_BAYER_SBGGR10P:
	case MMAL_ENCODING_BAYER_SGBRG10P:
	case MMAL_ENCODING_BAYER_SGRBG10P:
	case MMAL_ENCODING_BAYER_SRGGB10P:
		return VCOS_ALIGN_UP(width * 5 / 4, 32);
	case MMAL_ENCODING_BAYER_SBGGR12P:
	case MMAL_ENCODING_BAYER_SGBRG12P:
	case MMAL_ENCODING_BAYER_SGRBG12P:
	case MMAL_ENCODING_BAYER_SRGGB12P:
		return VCOS_ALIGN_UP(width * 3 / 2, 32);
	case MMAL_ENCODING_BAYER_SBGGR16:
	case MMAL_ENCODING_BAYER_SGBRG16:
	case MMAL_ENCODING_BAYER_SGRBG16:
	case MMAL_ENCODING_BAYER_SRGGB16:
	case MMAL_ENCODING_UYVY:
	case MMAL_ENCODING_YUYV:
		return VCOS_ALIGN_UP(width * 2, 32);
	default:
		return VCOS_ALIGN_UP(width, 32);
	}
}

static uint32_t frame_bytes(const MMAL_ES_FORMAT_T *fmt)
{
	uint32_t stride = mmal_encoding_width_to_stride(fmt->encoding, fmt->es->video.width);
	if (fmt->encoding == MMAL_ENCODING_I420)
		return stride * fmt->es->video.height * 3 / 2;
	return stride * fmt->es->video.height;
}

MMAL_STATUS_T mmal_format_full_copy(MMAL_ES_FORMAT_T *dst, MMAL_ES_FORMAT_T *src)
{
	MMAL_ES_SPECIFIC_FORMAT_T *es = dst->es;
	*dst = *src;
	dst->es = es;
	*dst->es = *src->es;
	return MMAL_SUCCESS;
}

MMAL_STATUS_T mmal_port_format_commit(MMAL_PORT_T *port)
{
	const MMAL_VIDEO_FORMAT_T *v = &port->format->es->video;
	if (should_fail("commit", port->name))
		return MMAL_EINVAL;
	mlog("COMMIT %s enc=%4.4s %ux%u crop=%d,%d,%dx%d", port->name, (char *)&port->format->encoding, v->width,
	     v->height, v->crop.x, v->crop.y, v->crop.width, v->crop.height);
	port->buffer_size_min = port->buffer_size_recommended = port->buffer_size = frame_bytes(port->format);
	if (port->priv->kind == RAWCAM_OUT)
		port->buffer_num_recommended = 8;
	return MMAL_SUCCESS;
}

static void log_param(MMAL_PORT_T *port, const char *op, const MMAL_PARAMETER_HEADER_T *param)
{
	char hex[1024];
	unsigned int i, n = 0, len = param->size - sizeof(*param);
	const uint8_t *p = (const uint8_t *)(param + 1);
	for (i = 0; i < len && n < sizeof(hex) - 3; i++)
		n += sprintf(hex + n, "%02x", p[i]);
	hex[n] = 0;
	mlog("%s %s id=%08x size=%u %s", op, port->name, param->id, param->size, hex);
}

MMAL_STATUS_T mmal_port_parameter_set(MMAL_PORT_T *port, const MMAL_PARAMETER_HEADER_T *param)
{
	if (param->id == MMAL_PARAMETER_CUSTOM_AWB_GAINS)
	{
		/* Timing dependent, so kept out of the deterministic log. */
		const MMAL_PARAMETER_AWB_GAINS_T *g = (const MMAL_PARAMETER_AWB_GAINS_T *)param;
		static int once;
		if (!once++ || getenv("MOCK_AWB_VERBOSE"))
			fprintf(stderr, "MOCK AWB gains r=%d/%d b=%d/%d\n", g->r_gain.num, g->r_gain.den, g->b_gain.num,
				g->b_gain.den);
		return MMAL_SUCCESS;
	}
	log_param(port, "PSET", param);
	return MMAL_SUCCESS;
}

MMAL_STATUS_T mmal_port_parameter_get(MMAL_PORT_T *port, MMAL_PARAMETER_HEADER_T *param)
{
	memset(param + 1, 0, param->size - sizeof(*param));
	if (param->id == MMAL_PARAMETER_CAMERA_RX_CONFIG)
	{
		MMAL_PARAMETER_CAMERA_RX_CONFIG_T *c = (MMAL_PARAMETER_CAMERA_RX_CONFIG_T *)param;
		c->data_lanes = 2;
		c->image_id = 0x2b;
		c->embedded_data_lines = 0;
	}
	else if (param->id == MMAL_PARAMETER_CAMERA_RX_TIMING)
	{
		MMAL_PARAMETER_CAMERA_RX_TIMING_T *t = (MMAL_PARAMETER_CAMERA_RX_TIMING_T *)param;
		t->timing1 = 11;
		t->timing2 = 12;
		t->timing3 = 13;
		t->timing4 = 14;
		t->timing5 = 15;
		t->term1 = 16;
		t->term2 = 17;
	}
	mlog("PGET %s id=%08x size=%u", port->name, param->id, param->size);
	return MMAL_SUCCESS;
}

MMAL_STATUS_T mmal_port_parameter_set_boolean(MMAL_PORT_T *port, uint32_t id, MMAL_BOOL_T value)
{
	MMAL_PARAMETER_BOOLEAN_T param = { { id, sizeof(param) }, value };
	return mmal_port_parameter_set(port, &param.hdr);
}

MMAL_STATUS_T mmal_port_parameter_set_int32(MMAL_PORT_T *port, uint32_t id, int32_t value)
{
	MMAL_PARAMETER_INT32_T param = { { id, sizeof(param) }, value };
	return mmal_port_parameter_set(port, &param.hdr);
}

MMAL_STATUS_T mmal_port_parameter_set_uint32(MMAL_PORT_T *port, uint32_t id, uint32_t value)
{
	MMAL_PARAMETER_UINT32_T param = { { id, sizeof(param) }, value };
	return mmal_port_parameter_set(port, &param.hdr);
}

/* ---------------------------------------------------- the fake sensor */

static volatile int sensor_streaming;
static int i2c_fake_fd = -1;
static int i2c_cur_addr = -1;
static uint8_t regmap[65536];
static int sensor_addr = 0x36;
static int sensor_present = 1;

static void sensor_init(void)
{
	const char *s = getenv("MOCK_SENSOR");
	if (!s || !strcmp(s, "ov5647"))
	{
		sensor_addr = 0x36;
		regmap[0x300A] = 0x56;
		regmap[0x300B] = 0x47;
	}
	else if (!strcmp(s, "imx219"))
	{
		sensor_addr = 0x10;
		regmap[0x0000] = 0x02;
		regmap[0x0001] = 0x19;
	}
	else if (!strcmp(s, "imx477"))
	{
		sensor_addr = 0x1A;
		regmap[0x0016] = 0x04;
		regmap[0x0017] = 0x77;
	}
	else
		sensor_present = 0;
}

static int bus_exists(int bus)
{
	const char *list = getenv("MOCK_I2C_BUSES");
	char *end;
	if (!list)
		list = "10";
	while (*list)
	{
		long v = strtol(list, &end, 10);
		if (end == list)
			break;
		if (v == bus)
			return 1;
		list = *end ? end + 1 : end;
	}
	return 0;
}

int open(const char *path, int flags, ...)
{
	mode_t mode = 0;
	if (flags & O_CREAT)
	{
		va_list ap;
		va_start(ap, flags);
		mode = va_arg(ap, mode_t);
		va_end(ap);
	}
	if (!strncmp(path, "/dev/i2c-", 9))
	{
		int bus = atoi(path + 9);
		if (!bus_exists(bus))
		{
			mlog("I2C open %s -> ENOENT", path);
			errno = ENOENT;
			return -1;
		}
		sensor_init();
		i2c_fake_fd = syscall(SYS_openat, AT_FDCWD, "/dev/null", O_RDWR);
		mlog("I2C open %s -> ok", path);
		return i2c_fake_fd;
	}
	return syscall(SYS_openat, AT_FDCWD, path, flags, mode);
}

int ioctl(int fd, unsigned long request, ...)
{
	va_list ap;
	void *arg;
	va_start(ap, request);
	arg = va_arg(ap, void *);
	va_end(ap);

	if (fd < 0 || fd != i2c_fake_fd)
		return syscall(SYS_ioctl, fd, request, arg);

	if (request == I2C_SLAVE_FORCE || request == I2C_SLAVE)
	{
		i2c_cur_addr = (int)(uintptr_t)arg;
		mlog("I2C addr %02x", i2c_cur_addr);
		return 0;
	}
	if (request == I2C_RDWR)
	{
		struct i2c_rdwr_ioctl_data *set = arg;
		unsigned int reg, i;
		if (!sensor_present || set->nmsgs != 2 || set->msgs[0].addr != sensor_addr)
		{
			errno = EREMOTEIO;
			return -1;
		}
		reg = set->msgs[0].len == 2 ? (set->msgs[0].buf[0] << 8) | set->msgs[0].buf[1] : set->msgs[0].buf[0];
		for (i = 0; i < set->msgs[1].len; i++)
			set->msgs[1].buf[i] = regmap[(reg + i) & 0xFFFF];
		mlog("I2C read %02x reg %04x len %u", set->msgs[0].addr, reg, set->msgs[1].len);
		return 2;
	}
	errno = ENOTTY;
	return -1;
}

ssize_t write(int fd, const void *buf, size_t count)
{
	const uint8_t *b = buf;
	unsigned int reg;

	if (fd < 0 || fd != i2c_fake_fd)
		return syscall(SYS_write, fd, buf, count);

	if (!sensor_present || i2c_cur_addr != sensor_addr || count < 3)
	{
		mlog("I2C write NACK addr %02x len %zu", i2c_cur_addr, count);
		errno = EREMOTEIO;
		return -1;
	}
	reg = (b[0] << 8) | b[1];
	regmap[reg] = b[2];
	if (count == 3)
		mlog("I2C W %04x %02x", reg, b[2]);
	else
		mlog("I2C W %04x %02x%02x", reg, b[2], b[3]);
	if (reg == 0x0100)
		sensor_streaming = b[2] & 1;
	return count;
}

/* ------------------------------------------------- synthetic image data */

static int bayer_info(uint32_t enc, int *bits, const char **order)
{
	static const struct
	{
		uint32_t enc;
		int bits;
		const char *order;
	} tab[] = {
		{ MMAL_ENCODING_BAYER_SBGGR8, 8, "BGGR" },    { MMAL_ENCODING_BAYER_SGBRG8, 8, "GBRG" },
		{ MMAL_ENCODING_BAYER_SGRBG8, 8, "GRBG" },    { MMAL_ENCODING_BAYER_SRGGB8, 8, "RGGB" },
		{ MMAL_ENCODING_BAYER_SBGGR10P, 10, "BGGR" }, { MMAL_ENCODING_BAYER_SGBRG10P, 10, "GBRG" },
		{ MMAL_ENCODING_BAYER_SGRBG10P, 10, "GRBG" }, { MMAL_ENCODING_BAYER_SRGGB10P, 10, "RGGB" },
		{ MMAL_ENCODING_BAYER_SBGGR12P, 12, "BGGR" }, { MMAL_ENCODING_BAYER_SGBRG12P, 12, "GBRG" },
		{ MMAL_ENCODING_BAYER_SGRBG12P, 12, "GRBG" }, { MMAL_ENCODING_BAYER_SRGGB12P, 12, "RGGB" },
		{ MMAL_ENCODING_BAYER_SBGGR16, 16, "BGGR" },  { MMAL_ENCODING_BAYER_SGBRG16, 16, "GBRG" },
		{ MMAL_ENCODING_BAYER_SGRBG16, 16, "GRBG" },  { MMAL_ENCODING_BAYER_SRGGB16, 16, "RGGB" },
	};
	unsigned int i;
	for (i = 0; i < sizeof(tab) / sizeof(tab[0]); i++)
	{
		if (tab[i].enc == enc)
		{
			*bits = tab[i].bits;
			*order = tab[i].order;
			return 1;
		}
	}
	return 0;
}

static void put_pixel(uint8_t *row, int x, int bits, unsigned int v)
{
	if (bits == 8)
		row[x] = v;
	else if (bits == 10)
	{
		uint8_t *g = row + (x / 4) * 5;
		g[x & 3] = v >> 2;
		g[4] = (g[4] & ~(3 << (2 * (x & 3)))) | ((v & 3) << (2 * (x & 3)));
	}
	else if (bits == 12)
	{
		uint8_t *g = row + (x / 2) * 3;
		g[x & 1] = v >> 4;
		g[2] = (g[2] & ~(0xF << (4 * (x & 1)))) | ((v & 0xF) << (4 * (x & 1)));
	}
	else
	{
		row[2 * x] = v & 0xFF;
		row[2 * x + 1] = v >> 8;
	}
}

/* R = 30%, G = 55%, B = 20% of full scale, times a horizontal ramp from 0.5 to 1,
 * plus a little noise in the LSBs. Grey-world gains: R 1.83, B 2.75. */
static uint8_t *render_base_frame(const MMAL_ES_FORMAT_T *fmt, uint32_t size)
{
	uint8_t *base = calloc(1, size);
	uint32_t stride = mmal_encoding_width_to_stride(fmt->encoding, fmt->es->video.width);
	int bits, x, y, w = fmt->es->video.crop.width, h = fmt->es->video.crop.height;
	const char *order;
	unsigned int seed = 12345;

	const char *real = getenv("MOCK_FRAME_FILE");
	FILE *rf = real ? fopen(real, "rb") : NULL;

	/* A frame from a real sensor: 16 bit little endian, 10 significant bits in the MSBs, no padding. */
	if (rf && bayer_info(fmt->encoding, &bits, &order))
	{
		uint16_t *line = malloc(w * 2);
		for (y = 0; y < h && fread(line, 2, w, rf) == (size_t)w; y++)
			for (x = 0; x < w; x++)
			{
				unsigned int v = line[x] >> 6; /* 10 bit */
				if (bits == 8)
					v >>= 2;
				else if (bits == 12)
					v <<= 2;
				put_pixel(base + (size_t)y * stride, x, bits, v);
			}
		free(line);
		fclose(rf);
		return base;
	}
	if (!bayer_info(fmt->encoding, &bits, &order))
	{
		uint32_t i;
		for (i = 0; i < size; i++)
			base[i] = (i * 7) & 0xFF;
		return base;
	}
	for (y = 0; y < h; y++)
	{
		for (x = 0; x < w; x++)
		{
			char c = order[(y & 1) * 2 + (x & 1)];
			double level = c == 'R' ? 0.30 : c == 'G' ? 0.55 : 0.20;
			unsigned int full = (bits == 16 ? 1 << 10 : 1 << bits) - 1; /* 16 bit holds 10 bit data */
			unsigned int v;
			seed = seed * 1103515245 + 12345;
			v = (unsigned int)(full * level * (0.5 + 0.5 * x / w)) + ((seed >> 16) & 3);
			put_pixel(base + (size_t)y * stride, x, bits, v > full ? full : v);
		}
	}
	return base;
}

/* ------------------------------------------------------- port threads */

static void deliver(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	MMAL_PORT_BH_CB_T cb = port->priv->cb;
	if (cb)
		cb(port, buffer);
	else
		merror("no callback on %s to return a buffer to", port->name);
}

static void *rawcam_thread(void *arg)
{
	MMAL_PORT_T *port = arg;
	const char *env = getenv("MOCK_FPS");
	double fps = env ? atof(env) : 30.0;
	int64_t interval_ns = (int64_t)(1e9 / fps);
	int with_metadata = getenv("MOCK_METADATA") && atoi(getenv("MOCK_METADATA"));
	uint32_t size = frame_bytes(port->format);
	uint32_t stride = mmal_encoding_width_to_stride(port->format->encoding, port->format->es->video.width);
	uint8_t *base = render_base_frame(port->format, size);
	struct timespec next;
	int64_t frame = 0;
	int was_streaming = 0;

	clock_gettime(CLOCK_MONOTONIC, &next);
	while (!port->priv->quit)
	{
		MMAL_BUFFER_HEADER_T *buffer;
		unsigned int msize = 0;
		uint8_t *mem;

		next.tv_nsec += interval_ns;
		while (next.tv_nsec >= 1000000000L)
		{
			next.tv_sec++;
			next.tv_nsec -= 1000000000L;
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
		if (port->priv->quit)
			break;
		if (!sensor_streaming)
		{
			was_streaming = 0;
			continue;
		}
		if (!was_streaming)
			was_streaming = 1;
		frame++;

		if (with_metadata && (buffer = mmal_queue_get(port->priv->held)) != NULL)
		{
			mem = vc_handle_to_mem(buffer->data, &msize);
			if (mem)
			{
				/* two lines of "register set" embedded data: reg 0x3000 = 0x11, 0x3001 = 0x22 */
				static const uint8_t line10[] = { 0x0a, 0xaa, 0x30, 0xa5, 0x55, 0x00, 0x5a, 0x11,
								  0x5a, 0x55, 0x22, 0x07, 0x07, 0x07, 0x55, 0x07 };
				memset(mem, 0x07, 2 * stride);
				memcpy(mem, line10, sizeof(line10));
				memcpy(mem + stride, line10, sizeof(line10));
			}
			buffer->length = 2 * stride;
			buffer->flags = MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO;
			buffer->pts = frame * (interval_ns / 1000);
			__sync_fetch_and_add(&stats.metadata_made, 1);
			deliver(port, buffer);
		}

		buffer = mmal_queue_get(port->priv->held);
		if (!buffer)
		{
			__sync_fetch_and_add(&stats.frames_dropped, 1);
			continue;
		}
		mem = vc_handle_to_mem(buffer->data, &msize);
		if (!mem || msize < size)
		{
			merror("rawcam buffer has no usable VCSM handle (data=%p, size %u, need %u)", buffer->data, msize,
			       size);
			buffer->length = 0;
		}
		else
		{
			uint32_t f = (uint32_t)frame;
			uint32_t band = (uint32_t)((frame * 8) % port->format->es->video.crop.height);
			memcpy(mem, base, size);
			/* moving dark band, and the frame number in the first 4 bytes */
			memset(mem + (size_t)band * stride, 0x10, stride * 4 <= size - band * stride ? stride * 4 : stride);
			memcpy(mem, &f, 4);
			buffer->length = size;
		}
		buffer->flags = MMAL_BUFFER_HEADER_FLAG_FRAME_END;
		buffer->pts = frame * (interval_ns / 1000);
		buffer->dts = INT64_MIN;
		__sync_fetch_and_add(&stats.frames_made, 1);
		deliver(port, buffer);
	}
	free(base);
	return NULL;
}

static void *isp_in_thread(void *arg)
{
	MMAL_PORT_T *port = arg;
	while (!port->priv->quit)
	{
		MMAL_BUFFER_HEADER_T *in = mmal_queue_timedwait(port->priv->held, 20), *out;
		unsigned int msize = 0;
		if (!in)
			continue;
		__sync_fetch_and_add(&stats.isp_in, 1);
		if (in->length && !vc_handle_to_mem(in->data, &msize))
			merror("isp input buffer has no usable VCSM handle (data=%p)", in->data);

		if (in->length && isp_out_port && isp_out_port->is_enabled &&
		    (out = mmal_queue_get(isp_out_port->priv->held)) != NULL)
		{
			uint8_t *mem = vc_handle_to_mem(out->data, &msize);
			uint32_t size = frame_bytes(isp_out_port->format);
			if (!mem || msize < size)
			{
				merror("isp output buffer has no usable VCSM handle (data=%p)", out->data);
				out->length = 0;
			}
			else
			{
				memset(mem, 0x80, size);
				memcpy(mem, &in->pts, 8);
				out->length = size;
			}
			out->pts = in->pts;
			out->flags = MMAL_BUFFER_HEADER_FLAG_FRAME_END;
			__sync_fetch_and_add(&stats.isp_out, 1);
			deliver(isp_out_port, out);
		}
		deliver(port, in);
	}
	return NULL;
}

static void *render_in_thread(void *arg)
{
	MMAL_PORT_T *port = arg;
	while (!port->priv->quit)
	{
		MMAL_BUFFER_HEADER_T *in = mmal_queue_timedwait(port->priv->held, 20);
		if (!in)
			continue;
		__sync_fetch_and_add(&stats.render_in, 1);
		deliver(port, in);
	}
	return NULL;
}

MMAL_STATUS_T mmal_port_enable(MMAL_PORT_T *port, MMAL_PORT_BH_CB_T cb)
{
	void *(*fn)(void *) = NULL;
	if (should_fail("enable", port->name))
		return MMAL_ENOSPC;
	if (port->is_enabled)
		return MMAL_EINVAL;
	mlog("ENABLE %s num=%u size=%u", port->name, port->buffer_num, port->buffer_size);
	port->priv->cb = cb;
	port->priv->quit = 0;
	port->is_enabled = 1;
	if (port->priv->kind == RAWCAM_OUT)
		fn = rawcam_thread;
	else if (port->priv->kind == ISP_IN)
		fn = isp_in_thread;
	else if (port->priv->kind == RENDER_IN)
		fn = render_in_thread;
	if (fn)
	{
		pthread_create(&port->priv->thread, NULL, fn, port);
		port->priv->thread_running = 1;
	}
	return MMAL_SUCCESS;
}

MMAL_STATUS_T mmal_port_disable(MMAL_PORT_T *port)
{
	MMAL_BUFFER_HEADER_T *b;
	if (!port->is_enabled)
		return MMAL_EINVAL;
	mlog("DISABLE %s", port->name);
	port->priv->quit = 1;
	if (port->priv->thread_running)
	{
		pthread_join(port->priv->thread, NULL);
		port->priv->thread_running = 0;
	}
	port->is_enabled = 0;
	/* As the real thing does: hand back, empty, every buffer the port still holds. */
	while (!getenv("MOCK_NO_FLUSH") && (b = mmal_queue_get(port->priv->held)) != NULL)
	{
		b->length = 0;
		b->flags = 0;
		__sync_fetch_and_add(&stats.flushed, 1);
		deliver(port, b);
	}
	port->priv->cb = NULL;
	return MMAL_SUCCESS;
}

MMAL_STATUS_T mmal_port_send_buffer(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	if (!port->is_enabled)
	{
		/* The caller keeps ownership; raspiraw then loses track of the header. */
		__sync_fetch_and_add(&stats.send_disabled, 1);
		return MMAL_EINVAL;
	}
	if (port->priv->kind == RAWCAM_OUT)
		__sync_fetch_and_add(&stats.rawcam_sent, 1);
	mmal_queue_put(port->priv->held, buffer);
	return MMAL_SUCCESS;
}

/* ---------------------------------------------------------------- pools */

MMAL_POOL_T *mmal_port_pool_create(MMAL_PORT_T *port, unsigned int headers, uint32_t payload_size)
{
	MMAL_POOL_T *pool;
	unsigned int i;
	if (should_fail("pool", port->name))
		return NULL;
	mlog("POOL %s headers=%u payload=%u", port->name, headers, payload_size);
	pool = calloc(1, sizeof(*pool));
	pool->queue = mmal_queue_create();
	pool->headers_num = headers;
	pool->header = calloc(headers ? headers : 1, sizeof(*pool->header));
	for (i = 0; i < headers; i++)
	{
		MMAL_BUFFER_HEADER_T *b = calloc(1, sizeof(*b));
		b->priv = calloc(1, sizeof(*b->priv));
		b->priv->pool = pool;
		b->priv->refcount = 1;
		b->type = &b->priv->type;
		pool->header[i] = b;
		mmal_queue_put(pool->queue, b);
	}
	return pool;
}

void mmal_port_pool_destroy(MMAL_PORT_T *port, MMAL_POOL_T *pool)
{
	unsigned int i;
	mlog("POOLDESTROY %s", port->name);
	for (i = 0; i < pool->headers_num; i++)
	{
		free(pool->header[i]->priv);
		free(pool->header[i]);
	}
	free(pool->header);
	free(pool->queue);
	free(pool);
}

/* ----------------------------------------------------------- components */

MMAL_STATUS_T mmal_component_create(const char *name, MMAL_COMPONENT_T **component)
{
	MMAL_COMPONENT_T *c;
	*component = NULL;
	if (should_fail("create", name))
		return MMAL_ENOENT;
	mlog("CREATE %s", name);
	c = calloc(1, sizeof(*c));
	c->priv = calloc(1, sizeof(*c->priv));
	snprintf(c->priv->name, sizeof(c->priv->name), "%s", name);
	c->name = c->priv->name;
	c->input = c->priv->in_arr;
	c->output = c->priv->out_arr;
	c->control = new_port(c, CONTROL, "ctrl", MMAL_PORT_TYPE_CONTROL);
	if (!strcmp(name, "vc.ril.rawcam"))
	{
		c->output_num = 1;
		c->output[0] = new_port(c, RAWCAM_OUT, "rawcam:out:0", MMAL_PORT_TYPE_OUTPUT);
	}
	else if (!strcmp(name, "vc.ril.isp"))
	{
		c->input_num = c->output_num = 1;
		c->input[0] = new_port(c, ISP_IN, "isp:in:0", MMAL_PORT_TYPE_INPUT);
		c->output[0] = isp_out_port = new_port(c, ISP_OUT, "isp:out:0", MMAL_PORT_TYPE_OUTPUT);
	}
	else if (!strcmp(name, "vc.ril.video_render"))
	{
		c->input_num = 1;
		c->input[0] = new_port(c, RENDER_IN, "render:in:0", MMAL_PORT_TYPE_INPUT);
	}
	else
	{
		free(c);
		return MMAL_ENOENT;
	}
	*component = c;
	return MMAL_SUCCESS;
}

MMAL_STATUS_T mmal_component_enable(MMAL_COMPONENT_T *component)
{
	if (should_fail("cenable", component->name))
		return MMAL_ENOSPC;
	mlog("CENABLE %s", component->name);
	component->is_enabled = 1;
	return MMAL_SUCCESS;
}

MMAL_STATUS_T mmal_component_disable(MMAL_COMPONENT_T *component)
{
	mlog("CDISABLE %s", component->name);
	component->is_enabled = 0;
	return MMAL_SUCCESS;
}

MMAL_STATUS_T mmal_component_destroy(MMAL_COMPONENT_T *component)
{
	mlog("DESTROY %s", component->name);
	/* Ports that are still enabled get disabled, which flushes their buffers. */
	if (component->input_num && component->input[0]->is_enabled)
		mmal_port_disable(component->input[0]);
	if (component->output_num && component->output[0]->is_enabled)
		mmal_port_disable(component->output[0]);
	if (component->output_num && component->output[0] == isp_out_port)
		isp_out_port = NULL;
	/* The structures are leaked on purpose: late users show up as MOCK ERRORs, not crashes. */
	return MMAL_SUCCESS;
}
