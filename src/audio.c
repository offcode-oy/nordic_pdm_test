#include <drivers/gpio.h>
#include <nrfx_clock.h>
#include <string.h>
#include <sys/ring_buffer.h>
#include <usb/class/usb_audio.h>
#include <usb/usb_device.h>
#include <zephyr.h>
#include "audio.h"
#include "audio_settings.h"

// DEBUG
#include <nrfx_pdm.h>
#include <audio/dmic.h>

#if defined(__GNUC__) && __GNUC__ >= 7
#define __fallthrough __attribute__((fallthrough))
#else
#define __fallthrough ((void)0)
#endif

#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(audio);

// This is a bit of hack, 48k 16bit stereo for 1ms
#define USB_BLOCK_SIZE 192

// Thread properties
#define AUDIO_STACK_SIZE 5000
#define AUDIO_PRIORITY   -2
k_tid_t audio_tid = NULL;
struct k_thread audio_thread_data;
K_THREAD_STACK_DEFINE(audio_stack_area, AUDIO_STACK_SIZE);


// from main.c
void set_clock_settings(uint8_t hfclk128_div, uint8_t hfclk192_div);

//==== PDM MODE ====

#define PDM_NODE    DT_NODELABEL(pdm_mic)
static const struct device *pdm_dev = DEVICE_DT_GET(PDM_NODE);

K_MEM_SLAB_DEFINE_STATIC(pdm_rx_mem_slab, BLOCK_SIZE , BLOCK_COUNT, 4);

struct pcm_stream_cfg mic_streams = {
	.pcm_rate = SAMPLE_FREQUENCY,
	.pcm_width = SAMPLE_BIT_WIDTH,
	.block_size = BLOCK_SIZE,
	.mem_slab = &pdm_rx_mem_slab,
};

struct dmic_cfg mic_cfg = {
	.io = {
		.min_pdm_clk_freq = 1000000,
		.max_pdm_clk_freq = 4000000,
		//.min_pdm_clk_freq = 3072000,
		//.max_pdm_clk_freq = 3072000,
	},
	.streams = &mic_streams,
	.channel = {
		.req_num_chan = 2,
        .req_num_streams = 1,
	},
};

static void audio_loop(void *, void *, void *);


// Used only for usb transfer
#ifdef CONFIG_USB_DEVICE_AUDIO
NET_BUF_POOL_FIXED_DEFINE(usb_audio_pool, CONFIG_USB_MAX_NUM_TRANSFERS, USB_BLOCK_SIZE, 0, NULL);  // TODO: check the minimum number of blocks
RING_BUF_ITEM_DECLARE_POW2(usb_ring_buf, 15);                                                      // 32k usb buffer
static const struct device *mic_dev;

// USB feature updates handler
static void feature_update(const struct device *dev, const struct usb_audio_fu_evt *evt)
{
    LOG_DBG("Control selector %d for channel %d updated", evt->cs, evt->channel);
    switch (evt->cs) {
        case USB_AUDIO_FU_MUTE_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_VOLUME_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_BASS_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_MID_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_TREBLE_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_AUTOMATIC_GAIN_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_DELAY_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_BASS_BOOST_CONTROL:
            __fallthrough;
        case USB_AUDIO_FU_LOUDNESS_CONTROL:
            break;
        default:
            break;
    }
}

static volatile bool usb_taken = false;

// This is the callback USB makes when it needs more data
static void data_request_cb(const struct device *dev)
{
    int ret;

    if (usb_taken || (ring_buf_size_get(&usb_ring_buf) < USB_BLOCK_SIZE)) {
        return;
    }

    // move it to temporary buffer
    struct net_buf *buf = net_buf_alloc_fixed(&usb_audio_pool, K_NO_WAIT);
    if (buf != NULL) {
        // retrieve data from circular buffer
        uint8_t tmp_buf[USB_BLOCK_SIZE];
        ring_buf_get(&usb_ring_buf, tmp_buf, USB_BLOCK_SIZE);
        net_buf_add_mem(buf, tmp_buf, USB_BLOCK_SIZE);
        ret = usb_audio_send(mic_dev, buf, USB_BLOCK_SIZE);
        if (ret < 0) {
            LOG_ERR("usb audio data_request_cb error");
            net_buf_unref(buf);
        }
    } else {
        LOG_ERR("usb net_buf alloc error");
    }
}

static const struct usb_audio_ops mic_ops = {
        .feature_update_cb = feature_update,
        .data_request_cb = data_request_cb,
};

void usb_status_cb(enum usb_dc_status_code cb_status, const uint8_t *param)
{
    switch (cb_status) {
        // These two cases trigger when the USB cable is connected/removed and reinserted
        case USB_DC_RESET:
            __fallthrough;
        case USB_DC_CONFIGURED:
            __fallthrough;
        case USB_DC_ERROR:
            __fallthrough;
        case USB_DC_CONNECTED:
            __fallthrough;
        case USB_DC_DISCONNECTED:
            __fallthrough;
        case USB_DC_SUSPEND:
            __fallthrough;
        case USB_DC_RESUME:
            __fallthrough;
        case USB_DC_INTERFACE:
            __fallthrough;
        case USB_DC_SET_HALT:
            __fallthrough;
        case USB_DC_CLEAR_HALT:
            __fallthrough;
        case USB_DC_SOF:
            __fallthrough;
        case USB_DC_UNKNOWN:
            break;
        default:
            break;
    }
}

__attribute__((unused)) static bool audio_usb_init(void)
{
    mic_dev = device_get_binding("MICROPHONE");
    if (!mic_dev) {
        LOG_ERR("Can not get USB Microphone Device");
        return false;
    }

    LOG_INF("Found USB Microphone Device");

    usb_audio_register(mic_dev, &mic_ops);

    int ret = usb_enable(usb_status_cb);
    if (ret != 0) {
        LOG_ERR("Failed to enable USB");
        return false;
    }

    LOG_INF("USB enabled");

    int usb_buffer_size = usb_audio_get_in_frame_size(mic_dev);
    if (usb_buffer_size != USB_BLOCK_SIZE) {
        LOG_ERR("USB buffer size is not %d", USB_BLOCK_SIZE);
        return false;
    }
    LOG_INF("usb mic buffer frame size: %d", usb_buffer_size);
    return true;
}
#endif  // USB_DEVICE_AUDIO



void audio_deinit()
{
    LOG_INF("stopping audio thread");

    dmic_trigger(pdm_dev, DMIC_TRIGGER_STOP);

    if (audio_tid) {
        LOG_INF("Waiting audio thread to exit...");
        k_thread_join(audio_tid, K_MSEC(200));
        LOG_INF("Waiting audio thread exit: done");

        k_thread_abort(audio_tid);
        audio_tid = NULL;
    }
    // Decrease the clock frequency to save power
    set_clock_settings(NRF_CLOCK_HFCLK_DIV_2, NRF_CLOCK_HFCLK_DIV_4);  // 64Mhz CPU, 12Mhz QSPI, TODO: call again to set to 128Mhz for audio encoding /* power:
                                                                       // 700uA @ 128Mhz/12Mhz, <100uA: 64Mhz/12Mhz */
}

static bool audio_init_pdm(void)
{

    if (!device_is_ready(pdm_dev)) {
        LOG_ERR("%s is not ready", pdm_dev->name);
        return false;
    } else {
        LOG_INF("%s ready", pdm_dev->name);
    }

    mic_cfg.channel.req_chan_map_lo = dmic_build_channel_map(1, 0, PDM_CHAN_LEFT) | dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT);

    int ret = dmic_configure(pdm_dev, &mic_cfg);
	if (ret < 0) {
		LOG_ERR("microphone configuration error");
		return false;
	} else {
        LOG_INF("microphone configuration OK");
    }

    return true;

}

void audio_init()
{
    bool res;
    LOG_INF("audio_init()");

    res = audio_init_pdm();

    if (!res) {
        return;
    }

	if (!audio_usb_init()) {
		LOG_ERR("usb init failed, no audio over USB\n");
	}

    // Increase the clock frequency to avoid underflow
    set_clock_settings(NRF_CLOCK_HFCLK_DIV_1, NRF_CLOCK_HFCLK_DIV_4);  // 64Mhz CPU, 12Mhz QSPI, TODO: call again to set to 128Mhz for audio encoding /* power:
                                                                       // 700uA @ 128Mhz/12Mhz, <100uA: 64Mhz/12Mhz */

    audio_tid = k_thread_create(&audio_thread_data, audio_stack_area, K_THREAD_STACK_SIZEOF(audio_stack_area), audio_loop, NULL, NULL, NULL, AUDIO_PRIORITY, 0,
                                K_MSEC(0));
    k_thread_name_set(&audio_thread_data, "Audio RX");
}


static void audio_loop(void *a, void *b, void *c)
{
    LOG_INF("audio loop");


    int ret = dmic_trigger(pdm_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("microphone start trigger error");
		return;
	} else {
        LOG_INF("microphone start trigger DONE");
    }

    LOG_INF("PDM TEST !");

    LOG_INF("Streams started");

    while (1) {
        int ret;

        uint8_t *mem_block = NULL;
        size_t block_size = 0;

		ret = dmic_read(pdm_dev, 0, (void *)&mem_block, &block_size, SYS_FOREVER_MS);
		if (ret < 0) {
			LOG_ERR("microphone audio read error\n");
			continue;
		}

        // push frame to usb
        ret = ring_buf_put(&usb_ring_buf, mem_block, block_size);
        if (ret < block_size) {
        	LOG_DBG("Failed to put all the data to usb buffer: %d/%d, freeing up space... audio glitch will occur.\n", ret, block_size);

        	usb_taken = true;
        	// read enough so the leftover data fits
        	if (ring_buf_get(&usb_ring_buf, NULL, block_size - ret) != block_size - ret) {
        		__ASSERT(0, "Failed to read enough data to fill usb buffer\n");
        	}

        	usb_taken = false;
        	if (ring_buf_put(&usb_ring_buf, mem_block + ret, block_size - ret) != block_size - ret) {
        		__ASSERT(0, "Failed to put all the data to usb buffer\n");
        	} else {
        		LOG_DBG("successfully freed and put the data to buffer\n");
        	}
        } else {
        	LOG_DBG("successfully put %d bytes to buffer\n", block_size);
        }

        // free the mem_block from slab

        k_mem_slab_free(&pdm_rx_mem_slab, (void *)&mem_block);
    }


    // Decrease the clock frequency to save power
    set_clock_settings(NRF_CLOCK_HFCLK_DIV_2, NRF_CLOCK_HFCLK_DIV_4);  // 64Mhz CPU, 12Mhz QSPI, TODO: call again to set to 128Mhz for audio encoding /* power:
                                                                       // 700uA @ 128Mhz/12Mhz, <100uA: 64Mhz/12Mhz */

    LOG_INF("Streams stopped");  // TODO: App should try to recover this scenario and restart the stream
}