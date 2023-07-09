#ifndef RASPIRAW_TYPES_H
#define RASPIRAW_TYPES_H

struct raspiraw_crop
{
	int hinc;
	int vinc;
	int width;
	int height;
	int left;
	int top;
};

typedef struct pts_node
{
	int idx;
	int64_t pts;
	struct pts_node *nxt;
} * PTS_NODE_T;

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

#endif