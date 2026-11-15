/* Unit test of raspiraw's get_pixel() against an independent reference, on random data. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "interface/mmal/mmal.h"

uint32_t get_pixel(int x, int y, uint32_t encoding, int stride, uint8_t *data);

static uint32_t ref(int x, int y, int bits, int stride, const uint8_t *d)
{
	const uint8_t *row = d + (size_t)y * stride;
	if (bits == 8)
		return row[x];
	if (bits == 10)
		return (row[(x / 4) * 5 + (x & 3)] << 2) | ((row[(x / 4) * 5 + 4] >> (2 * (x & 3))) & 3);
	if (bits == 12)
		return (row[(x / 2) * 3 + (x & 1)] << 4) | ((row[(x / 2) * 3 + 2] >> (4 * (x & 1))) & 0xF);
	return row[2 * x] | (row[2 * x + 1] << 8);
}

int main(void)
{
	static const struct { uint32_t enc; int bits; const char *name; } e[] = {
		{ MMAL_ENCODING_BAYER_SBGGR8, 8, "BGGR8" },      { MMAL_ENCODING_BAYER_SGRBG8, 8, "GRBG8" },
		{ MMAL_ENCODING_BAYER_SBGGR10P, 10, "BGGR10P" }, { MMAL_ENCODING_BAYER_SGBRG10P, 10, "GBRG10P" },
		{ MMAL_ENCODING_BAYER_SGRBG10P, 10, "GRBG10P" }, { MMAL_ENCODING_BAYER_SRGGB10P, 10, "RGGB10P" },
		{ MMAL_ENCODING_BAYER_SBGGR12P, 12, "BGGR12P" }, { MMAL_ENCODING_BAYER_SGBRG12P, 12, "GBRG12P" },
		{ MMAL_ENCODING_BAYER_SGRBG12P, 12, "GRBG12P" }, { MMAL_ENCODING_BAYER_SRGGB12P, 12, "RGGB12P" },
		{ MMAL_ENCODING_BAYER_SBGGR16, 16, "BGGR16" },   { MMAL_ENCODING_BAYER_SRGGB16, 16, "RGGB16" },
	};
	enum { W = 64, H = 4, STRIDE = 160 };
	uint8_t *data = malloc(STRIDE * H); /* exact size: sanitizer catches reads past the end */
	int i, x, y, total = 0;

	srand(1);
	for (i = 0; i < STRIDE * H; i++)
		data[i] = rand();
	for (i = 0; i < (int)(sizeof(e) / sizeof(e[0])); i++)
	{
		int bad = 0, msb_bad = 0, first = -1;
		for (y = 0; y < H; y++)
			for (x = 0; x < W; x++)
			{
				uint32_t got = get_pixel(x, y, e[i].enc, STRIDE, data), want = ref(x, y, e[i].bits, STRIDE, data);
				if (got != want)
				{
					bad++;
					if (first < 0)
						first = x;
					if (e[i].bits <= 12 && (got >> (e[i].bits - 8)) != (want >> (e[i].bits - 8)))
						msb_bad++;
				}
			}
		printf("  %-8s %3d/%d pixels wrong%s", e[i].name, bad, W * H, bad ? "" : "  ok");
		if (bad)
			printf(" (first at x=%d, %d with wrong MSBs)", first, msb_bad);
		printf("\n");
		total += bad;
	}
	return total != 0;
}
