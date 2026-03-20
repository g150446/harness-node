#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/device.h>
#include <string.h>

#define I2S_NODE    DT_NODELABEL(i2s20)
#define SAMPLE_RATE 48000
#define BLOCK_SIZE  512   /* bytes = 128 stereo 16-bit samples */
#define NUM_BLOCKS  4

K_MEM_SLAB_DEFINE(i2s_slab, BLOCK_SIZE, NUM_BLOCKS, 4);

static void fill_sine(int16_t *buf, size_t samples)
{
	static uint32_t phase = 0;
	/* 440 Hz @ 48000 Hz: 440/48000 * 65536 ≈ 601 */
	const uint32_t phase_inc = 601;

	for (size_t i = 0; i < samples; i += 2) {
		int32_t s = (int32_t)(phase & 0xFFFF) - 32768;
		s = (s * 28000) >> 15;
		buf[i]   = (int16_t)s; /* L */
		buf[i+1] = (int16_t)s; /* R */
		phase += phase_inc;
	}
}

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(I2S_NODE);

	if (!device_is_ready(dev)) {
		printk("I2S device not ready\n");
		return -1;
	}

	struct i2s_config cfg = {
		.word_size      = 16,
		.channels       = 2,
		.format         = I2S_FMT_DATA_FORMAT_I2S,
		.options        = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER,
		.frame_clk_freq = SAMPLE_RATE,
		.mem_slab       = &i2s_slab,
		.block_size     = BLOCK_SIZE,
		.timeout        = 1000,
	};

	int ret = i2s_configure(dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		printk("I2S configure failed: %d\n", ret);
		return ret;
	}
	printk("I2S configured OK, starting 440 Hz tone\n");

	/* Pre-fill TX queue before trigger */
	for (int i = 0; i < NUM_BLOCKS; i++) {
		void *mem;
		ret = k_mem_slab_alloc(&i2s_slab, &mem, K_NO_WAIT);
		if (ret < 0) {
			printk("slab alloc failed: %d\n", ret);
			return ret;
		}
		fill_sine((int16_t *)mem, BLOCK_SIZE / sizeof(int16_t));
		i2s_write(dev, mem, BLOCK_SIZE);
	}

	ret = i2s_trigger(dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("I2S trigger failed: %d\n", ret);
		return ret;
	}

	while (1) {
		void *mem;
		ret = k_mem_slab_alloc(&i2s_slab, &mem, K_MSEC(200));
		if (ret < 0) {
			printk("slab alloc timeout: %d\n", ret);
			continue;
		}
		fill_sine((int16_t *)mem, BLOCK_SIZE / sizeof(int16_t));
		ret = i2s_write(dev, mem, BLOCK_SIZE);
		if (ret < 0) {
			printk("i2s_write failed: %d\n", ret);
		}
	}

	return 0;
}
