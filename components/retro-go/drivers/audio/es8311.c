// ES8311 I2S audio driver for ESP32-P4 / Waveshare EP44B
// No external dependencies: uses rg_i2c for codec register access,
// IDF 5.x driver/i2s_std for I2S, driver/gpio for amp enable.
//
// ES8311 I2C address: 0x18 (ADDR pin pulled low on EP44B).
// MCLK = sample_rate * 256 (I2S_MCLK_MULTIPLE_256).
// GPIO assignments come from config.h:
//   RG_GPIO_SND_MCLK        GPIO13
//   RG_GPIO_SND_I2S_BCK     GPIO12
//   RG_GPIO_SND_I2S_WS      GPIO10
//   RG_GPIO_SND_I2S_DATA    GPIO9
//   RG_GPIO_SND_AMP_ENABLE  GPIO53  (active HIGH → NS4150B enable)
//
// Init sequence derived from Espressif esp_codec_dev es8311.c:
//   es8311_open() + es8311_set_bits_per_sample(16) +
//   es8311_config_fmt(I2S_NORMAL) + es8311_config_sample() + es8311_start()

#include "rg_system.h"
#include "rg_audio.h"
#include "rg_i2c.h"

#ifdef RG_AUDIO_USE_ES8311

#include <string.h>
#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define ES8311_ADDR         0x18

// Register map (subset needed for DAC-only operation)
#define R00  0x00   // reset / clock enable
#define R01  0x01   // CLK manager 1 (MCLK source / invert)
#define R02  0x02   // CLK manager 2 (pre-divider / pre-multiplier)
#define R03  0x03   // CLK manager 3 (ADC FS mode / OSR)
#define R04  0x04   // CLK manager 4 (DAC OSR)
#define R05  0x05   // CLK manager 5 (ADC div / DAC div)
#define R06  0x06   // CLK manager 6 (BCLK invert / divider)
#define R07  0x07   // CLK manager 7 (LRCK divider high)
#define R08  0x08   // CLK manager 8 (LRCK divider low)
#define R09  0x09   // DAC serial digital port (I2S format, bit width)
#define R0A  0x0A   // ADC serial digital port
#define R0B  0x0B   // system
#define R0C  0x0C   // system
#define R0D  0x0D   // system power
#define R0E  0x0E   // analog power
#define R10  0x10   // system
#define R11  0x11   // system
#define R12  0x12   // DAC enable
#define R13  0x13   // system
#define R14  0x14   // analog PGA / DMIC select
#define R15  0x15   // ADC ramp
#define R16  0x16   // ADC MIC gain
#define R17  0x17   // ADC volume
#define R1B  0x1B   // ADC HPF
#define R1C  0x1C   // ADC equalizer
#define R31  0x31   // DAC mute
#define R32  0x32   // DAC output volume (0x00 = min, 0xFF = max)
#define R37  0x37   // DAC ramp rate
#define R44  0x44   // GPIO (I2C noise immunity / DAC reference)
#define R45  0x45   // GP control

// Clock coefficient table for MCLK = sample_rate × 256.
// All entries here share pre_div=1, pre_multi=1, adc_div=1, dac_div=1,
// fs_mode=0, lrck_h=0x00, lrck_l=0xFF, bclk_div=4; only adc/dac_osr differ
// (0x20 at 8 kHz for relaxed oversampling, 0x10 at all higher rates).
static const struct {
    uint32_t rate;
    uint8_t  dac_osr;
} s_coeff[] = {
    {  8000, 0x20 },
    { 16000, 0x10 },
    { 22050, 0x10 },
    { 32000, 0x10 },
    { 44100, 0x10 },
    { 48000, 0x10 },
    { 65536, 0x10 }, // GBA
};

static i2s_chan_handle_t s_tx       = NULL;
static int               s_volume   = 75;
static bool              s_muted    = false;
static const char       *s_error    = NULL;

static inline bool es_write(uint8_t reg, uint8_t val)
{
    return rg_i2c_write_byte(ES8311_ADDR, reg, val);
}

static inline int es_read(uint8_t reg)
{
    return rg_i2c_read_byte(ES8311_ADDR, reg);
}

// Apply ES8311 clock registers for a given sample rate.
// Assumes MCLK = rate * 256 has already been set in I2S MCLK config.
static bool es_config_sample(int rate)
{
    const __typeof__(s_coeff[0]) *c = NULL;
    for (int i = 0; i < (int)RG_COUNT(s_coeff); i++) {
        if (s_coeff[i].rate == (uint32_t)rate) {
            c = &s_coeff[i];
            break;
        }
    }
    if (!c) {
        RG_LOGW("ES8311: unsupported sample rate %d Hz\n", rate);
        return false;
    }

    // All coefficients for MCLK=rate*256: pre_div=1 pre_multi=1 → REG02=0x00
    es_write(R02, 0x00);
    // ADC div=1 DAC div=1 → REG05=0x00
    es_write(R05, 0x00);
    // REG03: fs_mode=0, adc_osr=0x10
    uint8_t r = (uint8_t)(es_read(R03) & 0x80) | 0x10;
    es_write(R03, r);
    // REG04: dac_osr
    r = (uint8_t)(es_read(R04) & 0x80) | c->dac_osr;
    es_write(R04, r);
    // REG07: lrck_h=0 (keep upper 2 bits)
    r = (uint8_t)(es_read(R07) & 0xC0);
    es_write(R07, r);
    // REG08: lrck_l=0xFF
    es_write(R08, 0xFF);
    // REG06: bclk_div=4 → (bclk_div-1)=3 → lower 5 bits = 3 (keep upper 3 bits)
    r = (uint8_t)(es_read(R06) & 0xE0) | 0x03;
    es_write(R06, r);

    return true;
}

static bool es_codec_init(int rate)
{
    // Amp off during init
    gpio_config_t pa = {
        .pin_bit_mask  = 1ULL << RG_GPIO_SND_AMP_ENABLE,
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&pa);
    gpio_set_level(RG_GPIO_SND_AMP_ENABLE, 0);

    // --- es8311_open() sequence ---
    es_write(R0D, 0xFA);    // power-down analog
    es_write(R44, 0x08);    // I2C noise immunity
    es_write(R44, 0x08);    // write twice (per Espressif driver recommendation)
    es_write(R01, 0x30);
    es_write(R02, 0x00);
    es_write(R03, 0x10);
    es_write(R16, 0x24);    // ADC MIC gain
    es_write(R04, 0x10);
    es_write(R05, 0x00);
    es_write(R0B, 0x00);
    es_write(R0C, 0x00);
    es_write(R10, 0x1F);
    es_write(R11, 0x7F);
    es_write(R00, 0x80);    // release reset; bit6=0 → slave mode

    // REG01: MCLK from pin (bit7=0), no MCLK invert (bit6=0), keep lower 6 bits
    es_write(R01, 0x3F);

    // REG06: clear SCLK invert bit (bit5)
    es_write(R06, (uint8_t)(es_read(R06) & ~0x20));

    es_write(R13, 0x10);
    es_write(R1B, 0x0A);    // ADC HPF
    es_write(R1C, 0x6A);    // ADC equalizer
    es_write(R44, 0x58);    // ADCL + DACR internal reference

    // --- set_bits_per_sample(16): REG09/R0A bits[4:2]=011 → set 0x0C ---
    es_write(R09, (uint8_t)((es_read(R09) & ~0x1C) | 0x0C));
    es_write(R0A, (uint8_t)((es_read(R0A) & ~0x1C) | 0x0C));

    // --- config_fmt(I2S_NORMAL): REG09/R0A bits[1:0]=00 ---
    es_write(R09, (uint8_t)(es_read(R09) & 0xFC));
    es_write(R0A, (uint8_t)(es_read(R0A) & 0xFC));

    // --- config_sample (MCLK = rate × 256 divider chain) ---
    if (!es_config_sample(rate))
        return false;

    // --- es8311_start(): enable output ---
    es_write(R00, 0x80);    // slave mode, reset released
    es_write(R01, 0x3F);    // MCLK from pin, no invert

    // REG09/R0A: clear bit6 (not tristate)
    es_write(R09, (uint8_t)(es_read(R09) & 0xBF));
    es_write(R0A, (uint8_t)(es_read(R0A) & 0xBF));

    es_write(R17, 0xBF);    // ADC volume (we're DAC-only; set high to avoid noise feedback)
    es_write(R0E, 0x02);    // analog power up
    es_write(R12, 0x00);    // enable DAC path
    es_write(R14, 0x1A);    // analog PGA
    es_write(R14, (uint8_t)(es_read(R14) & ~0x40)); // digital mic off
    es_write(R0D, 0x01);    // power up system
    es_write(R15, 0x40);    // ADC ramp
    es_write(R37, 0x08);    // DAC ramp rate
    es_write(R45, 0x00);    // GP control

    // Unmute DAC and set initial volume
    es_write(R31, 0x00);
    es_write(R32, (uint8_t)((s_volume * 255 + 50) / 100));

    // Enable amplifier
    gpio_set_level(RG_GPIO_SND_AMP_ENABLE, 1);

    return true;
}

// ─── Dynamic rate control ────────────────────────────────────────────────────
// The display paces emulation (one frame per panel refresh, see mipi_dsi.h lcd_sync()), and the
// panel never runs at exactly the emulated rate (this EP44B: 59.63 Hz vs 60.0 for 32 kHz/533).
// Instead of letting the I2S clock pace frames (which drops/repeats frames on the panel), keep
// the DMA ring half full by resampling the submitted audio by up to ±2%: a PI controller on the
// fill level settles at the panel/audio clock ratio (~0.8%, inaudible).
#define DRC_CHUNK_FRAMES   128      // 512 bytes: multiple of the 64-byte cache line (IDF rounds)
#define DRC_KP             0.01f    // per 60 Hz frame; tuned in simulation for 59.6–60.2 Hz panels
#define DRC_KI             0.00005f
#define DRC_ALPHA          0.02f    // fill low-pass (the measurement is quantised to a chunk)
#define DRC_MAX_DEV        0.02f
// Uncomment to log the controller state (ratio, fill level, underruns) every 10 s
// #define ES8311_DRC_PROF

static struct {
    volatile uint32_t consumed;     // bytes the DMA finished sending (ISR)
    uint32_t written;               // bytes accepted by i2s_channel_write
    int cap_frames, frames_per_tick;
    float favg, integ, ratio, pos;
    rg_audio_frame_t prev;
    uint32_t underruns;
} drc;

static bool IRAM_ATTR drc_on_sent(i2s_chan_handle_t h, i2s_event_data_t *e, void *ctx)
{
    drc.consumed += e->size;
    return false;
}

static void drc_reset(int rate, int cap_frames)
{
    memset(&drc, 0, sizeof(drc));
    drc.cap_frames = cap_frames;
    drc.frames_per_tick = RG_MAX(1, rate / 60);
    drc.favg = cap_frames / 2;
    drc.ratio = 1.f;
}

static bool es_i2s_init(int rate)
{
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.auto_clear    = true;
    // Small DMA buffers so the fill level (dynamic rate control) is known to within 4 ms at
    // 32 kHz, ~64 ms in total. The total is what absorbs a slow frame.
    chan.dma_frame_num = DRC_CHUNK_FRAMES;
    chan.dma_desc_num  = RG_MAX(4, (rate * 64 / 1000 + DRC_CHUNK_FRAMES - 1) / DRC_CHUNK_FRAMES);
    drc_reset(rate, chan.dma_frame_num * chan.dma_desc_num);

    if (i2s_new_channel(&chan, &s_tx, NULL) != ESP_OK) {
        s_error = "i2s_new_channel failed";
        return false;
    }

    i2s_std_config_t std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)rate),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = RG_GPIO_SND_MCLK,
            .bclk = RG_GPIO_SND_I2S_BCK,
            .ws   = RG_GPIO_SND_I2S_WS,
            .dout = RG_GPIO_SND_I2S_DATA,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {0},
        },
    };

    if (i2s_channel_init_std_mode(s_tx, &std) != ESP_OK) {
        i2s_del_channel(s_tx);
        s_tx = NULL;
        s_error = "i2s_channel_init_std_mode failed";
        return false;
    }

    i2s_event_callbacks_t cbs = {.on_sent = drc_on_sent};
    i2s_channel_register_event_callback(s_tx, &cbs, NULL);

    if (i2s_channel_enable(s_tx) != ESP_OK) {
        i2s_del_channel(s_tx);
        s_tx = NULL;
        s_error = "i2s_channel_enable failed";
        return false;
    }

    return true;
}

static bool driver_init(int device, int sample_rate)
{
    s_error = NULL;
    if (!es_i2s_init(sample_rate))
        return false;
    if (!es_codec_init(sample_rate)) {
        i2s_channel_disable(s_tx);
        i2s_del_channel(s_tx);
        s_tx = NULL;
        s_error = "ES8311 codec init failed";
        return false;
    }
    RG_LOGI("ES8311 ready. rate=%d vol=%d\n", sample_rate, s_volume);
    return true;
}

static bool driver_deinit(void)
{
    // Amp off first to avoid pop
    gpio_set_level(RG_GPIO_SND_AMP_ENABLE, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Mute DAC and power down
    es_write(R31, 0x60);    // DAC software mute
    es_write(R0D, 0xFA);    // power down analog
    es_write(R12, 0x02);    // disable DAC

    if (s_tx) {
        i2s_channel_disable(s_tx);
        i2s_del_channel(s_tx);
        s_tx = NULL;
    }
    return true;
}

static void drc_write(const rg_audio_frame_t *frames, size_t count)
{
    size_t written = 0;
    i2s_channel_write(s_tx, frames, count * sizeof(rg_audio_frame_t), &written, pdMS_TO_TICKS(200));
    drc.written += written;
}

static bool driver_submit(const rg_audio_frame_t *frames, size_t count)
{
    if (!s_tx || !count)
        return false;

    // Linear resampler, phase-continuous across calls: pos is the read position in input frames,
    // where -1 is the last frame of the previous call (drc.prev).
    static rg_audio_frame_t out[256];
    const float step = 1.f / drc.ratio;
    const int last = (int)count - 1;
    float pos = drc.pos;
    size_t n = 0;
    while (pos < last) {
        int i = (int)(pos + 1.f) - 1; // floor for pos >= -1
        float f = pos - i;
        const rg_audio_frame_t a = i < 0 ? drc.prev : frames[i];
        const rg_audio_frame_t b = frames[i + 1];
        out[n].left  = a.left  + (int)((b.left  - a.left)  * f);
        out[n].right = a.right + (int)((b.right - a.right) * f);
        if (++n == RG_COUNT(out)) {
            drc_write(out, n);
            n = 0;
        }
        pos += step;
    }
    if (n)
        drc_write(out, n);
    drc.pos = pos - count;
    drc.prev = frames[last];

    // Fill level → resampling ratio. Gains are per 60 Hz frame's worth of input.
    int32_t fill = (int32_t)(drc.written - drc.consumed) / (int32_t)sizeof(rg_audio_frame_t);
    if (fill < 0) {
        // Underrun (paused game, menu, slow frame): the DMA played silence, resync the count
        drc.written = drc.consumed;
        fill = 0;
        drc.underruns++;
    }
    const float w = RG_MIN(4.f, (float)count / drc.frames_per_tick);
    const float target = drc.cap_frames / 2;
    drc.favg += (fill - drc.favg) * RG_MIN(1.f, DRC_ALPHA * w);
    const float err = (target - drc.favg) / target; // > 0: too empty → produce more
    drc.integ = RG_MAX(-DRC_MAX_DEV, RG_MIN(DRC_MAX_DEV, drc.integ + err * DRC_KI * w));
    drc.ratio = RG_MAX(1.f - DRC_MAX_DEV, RG_MIN(1.f + DRC_MAX_DEV, 1.f + DRC_KP * err + drc.integ));

#ifdef ES8311_DRC_PROF
    static int64_t t0 = 0;
    static uint32_t underruns0 = 0;
    int64_t now = esp_timer_get_time();
    if (now - t0 >= 10000000) {
        RG_LOGI("DRC ratio=%.4f integ=%.4f fill=%d avg=%d/%d underruns=%u\n", drc.ratio, drc.integ,
                (int)fill, (int)drc.favg, drc.cap_frames, (unsigned)(drc.underruns - underruns0));
        t0 = now;
        underruns0 = drc.underruns;
    }
#endif
    return true;
}

static bool driver_set_volume(int percent)
{
    s_volume = percent;
    uint8_t reg = (uint8_t)((percent * 255 + 50) / 100);
    return es_write(R32, reg);
}

static bool driver_set_mute(bool mute)
{
    s_muted = mute;
    es_write(R31, mute ? 0x60 : 0x00);
    gpio_set_level(RG_GPIO_SND_AMP_ENABLE, mute ? 0 : 1);
    return true;
}

static bool driver_set_sample_rate(int rate)
{
    if (!s_tx)
        return false;

    // Reconfigure ES8311 clock registers
    if (!es_config_sample(rate))
        return false;

    // Reconfigure I2S MCLK / BCLK without tearing down the channel
    i2s_channel_disable(s_tx);
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)rate);
    i2s_channel_reconfig_std_clock(s_tx, &clk);
    drc_reset(rate, drc.cap_frames);
    i2s_channel_enable(s_tx);

    return true;
}

static const char *driver_get_error(void)
{
    return s_error;
}

const rg_audio_driver_t rg_audio_driver_es8311 = {
    .name            = "es8311",
    .init            = driver_init,
    .deinit          = driver_deinit,
    .submit          = driver_submit,
    .set_volume      = driver_set_volume,
    .set_mute        = driver_set_mute,
    .set_sample_rate = driver_set_sample_rate,
    .get_error       = driver_get_error,
};

#endif // RG_AUDIO_USE_ES8311
