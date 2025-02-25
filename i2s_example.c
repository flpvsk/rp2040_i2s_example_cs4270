/* i2s_examples.c
 *
 * Author: Daniel Collins
 * Date:   2022-02-25
 *
 * Copyright (c) 2022 Daniel Collins
 *
 * This file is part of rp2040_i2s_example.
 *
 * rp2040_i2s_example is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License, version 3 as published by the
 * Free Software Foundation.
 *
 * rp2040_i2s_example is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * rp2040_i2s_example. If not, see <https://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "i2s.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"

// I2C defines
// This example uses I2C0 on GPIO4 (SDA) and GPIO5 (SCL) running at 100KHz.
// Connect the codec I2C control to this. (Codec-specific customization is
// not part of this example.)
#define I2C_PORT i2c1
#define I2C_SDA  14
#define I2C_SCL  15

#define RESET_INV_PIN 22

#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
#endif

static __attribute__((aligned(8))) pio_i2s i2s;

// const int32_t MAX_INT = ((INT32_C(1)<<30)-1)*2+1;

static uint32_t inc_wrap(uint32_t i, uint32_t max) {
  if (i == max) return 0;
  return i + 1;
}

void print_bin(uint32_t x) {
  char bin_str[] = " 0000 0000 0000 0000 0000 0000 0000 0000";

  // 5  -> 33
  // 31 ->  0
  int i = 0;
  while (i < 39) {
    int idx = 39 - ((i / 4) * 5 + i % 4);

    if (x & 1) {
      bin_str[idx] = '1';
    } else {
      bin_str[idx] = '0';
    }

    i += 1;
    x = x >> 1;
  }

  puts(bin_str);
}

uint32_t read_i = 0;
uint32_t write_i = UINT8_MAX - 1;

int32_t buffy[UINT8_MAX] = { 0 };

static void process_audio(
    const uint32_t* input,
    uint32_t* output,
    size_t num_frames
) {
    // Just copy the input to the output
    for (size_t i = 0; i < num_frames * 2; i++) {
        // zero-out 8 LSBs
        output[i] = input[i] & ~((uint32_t)0x100 - 1);

        if (read_i == 1 && i % 2 == 0) {
          print_bin(input[i]);
        }

        // input[i] = 0;

        // if (i % 2 == 0) {
        //     output[i] = 0;
        //     continue;
        // }
        // output[i] = 0.1 * INT32_MAX * (float)(((float) phase / div) - floor(phase / div));
        // phase = inc_wrap(phase);

        // buffy[write_i] = input[i];
        // output[i] = buffy[read_i];

        // read_i = inc_wrap(read_i, UINT8_MAX - 1);
        // write_i = inc_wrap(write_i, UINT8_MAX - 1);
    }

    read_i = inc_wrap(read_i, UINT16_MAX);
}

static void dma_i2s_in_handler(void) {
    bool has_buf = false;
    /* We're double buffering using chained TCBs. By
     * checking which buffer the DMA is currently reading
     * from, we can identify which buffer it has just
     * finished reading (the completion of which has
     * triggered this interrupt).
     */
    uint32_t* dma_addr = *(uint32_t**)dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr;
    if (dma_addr == i2s.input_buffer) {
        // It is inputting to the second buffer so we can
        // overwrite the first
        process_audio(
            i2s.input_buffer,
            i2s.output_buffer,
            AUDIO_BUFFER_FRAMES
        );
        has_buf = 1;
    }

    if (dma_addr == &i2s.input_buffer[STEREO_BUFFER_SIZE]) {
        // It is currently inputting the first buffer, so we
        // write to the second
        process_audio(
            &i2s.input_buffer[STEREO_BUFFER_SIZE],
            &i2s.output_buffer[STEREO_BUFFER_SIZE],
            AUDIO_BUFFER_FRAMES
        );
        has_buf = 1;
    }

    if (!has_buf) {
      puts("WARN");
    }

    dma_hw->ints0 = 1u << i2s.dma_ch_in_data;  // clear the IRQ
}

void blink_it() {
  uint8_t i = 3;
  while (i--) {
      gpio_put(LED_PIN, 1);
      sleep_ms(100);
      gpio_put(LED_PIN, 0);
      sleep_ms(100);
  }
}

void panic_on_err(int r) {
  if (r < 0) {
    blink_it();
    panic("i2c write failed");
  }

  return;
}

int main() {
    // Set a 132.000 MHz system clock to more evenly divide the audio frequencies
    set_sys_clock_khz(132000, true);
    stdio_init_all();

    printf("System Clock: %lu\n", clock_get_hz(clk_sys));

    // Init GPIO LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(RESET_INV_PIN);
    gpio_set_dir(RESET_INV_PIN, GPIO_OUT);
    gpio_put(RESET_INV_PIN, 0); // put codec in reset

    // I2C Initialisation. Using it at 100Khz.
    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_pulls(I2C_SDA, true, false);
    gpio_set_pulls(I2C_SCL, true, false);
    gpio_set_drive_strength(I2C_SDA, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2C_SCL, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(I2C_SDA, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(I2C_SCL, GPIO_SLEW_RATE_FAST);

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // sleep_ms(5000);
    // printf("Press any key\n");
    // getchar();

    // Here, do whatever you need to set up your codec for proper operation.
    // Some codecs require register configuration over I2C, for example.

    uint8_t i2c_addr = 0b1001001;
    uint8_t i2c_cmd_power_down[2] = {
      0x02,     // Power control MAP
      (
        1 |        // Power down device
        1 << 1 |   // Power down DAC
        1 << 5     // Power down ADC
      )
    };

    uint8_t i2c_cmd_set_mode[2] = {
      0x03,     // Mode control MAP
      (
        (1 << 1) // clock divider 1.5 (single speed, master mode)
      ),
    };

    // uint8_t i2c_cmd_set_mode[2] = {
    //   0x03,     // Mode control MAP
    //   (
    //     0 // clock divider 1 (single speed, master mode)
    //   ),
    // };

    // uint8_t i2c_cmd_set_mode[2] = {
    //   0x03,     // Mode control MAP
    //   (
    //     (1 << 1) |            // clock divider 1.5
    //     (1 << 4) | (1 << 5)   // slave mode
    //   ),
    // };

    uint8_t i2c_cmd_codec_config[2] = {
      0x04,     // ADC & DAC control MAP
      (
        // (1 << 5) |  // enable digital loopback
        (1 << 3) |  // dac mode i2s
        1           // adc mode i2s
      )
    };

    uint8_t i2c_cmd_mute_config[2] = {
      0x06,     // Mute control
      0         // disable mute
    };

    uint8_t i2c_cmd_vol[2] = {
      0x07,         // DAC Channel A vol
      0b00010100,   // -10dB
    };

    uint8_t i2c_cmd_transition_config[2] = {
      0x05,     // Transition control
      1         // de-emph filter
    };

    uint8_t i2c_cmd_power_up[2] = {
      0x02,     // Power control MAP
      0,        // Power up all
    };

    sleep_ms(500);
    gpio_put(RESET_INV_PIN, 1); // enable codec

    int r;
    r = i2c_write_blocking(
        I2C_PORT,
        i2c_addr,
        i2c_cmd_power_down,
        2,
        false
    );
    printf("Power down: %d\n", r);
    panic_on_err(r);

    sleep_ms(1);

    r = i2c_write_blocking(
        I2C_PORT,
        i2c_addr,
        i2c_cmd_set_mode,
        2,
        false
    );
    printf("Mode: %d\n", r);
    panic_on_err(r);

    sleep_ms(1);

    i2c_write_blocking(
        I2C_PORT,
        i2c_addr,
        i2c_cmd_mute_config,
        2,
        false
    );

    sleep_ms(1);

    r = i2c_write_blocking(
        I2C_PORT,
        i2c_addr,
        i2c_cmd_codec_config,
        2,
        false
    );
    printf("Codec: %d\n", r);
    panic_on_err(r);

    r = i2c_write_blocking(
        I2C_PORT,
        i2c_addr,
        i2c_cmd_transition_config,
        2,
        false
    );
    panic_on_err(r);

    r = i2c_write_blocking(
        I2C_PORT,
        i2c_addr,
        i2c_cmd_vol,
        2,
        false
    );
    printf("Vol: %d\n", r);
    panic_on_err(r);

    sleep_ms(1);

    // uint8_t device_id_map = 0x01;

    // r = i2c_write_blocking(
    //     I2C_PORT,
    //     i2c_addr,
    //     &i2c_cmd_power_up[0],
    //     1,
    //     false
    // );
    // uint8_t data;
    // i2c_read_blocking(
    //     I2C_PORT,
    //     i2c_addr,
    //     &data,
    //     1,
    //     false
    // );

    // printf("pwr cfg %d\n", data);

    sleep_ms(1);


    // Note: it is usually best to configure the codec here,
    // and then enable it after starting the I2S clocks,
    // below.


    // i2s_program_start_synched(
    //     pio0,
    //     &i2s_config_default,
    //     dma_i2s_in_handler,
    //     &i2s
    // );


    i2s_program_start_slaved_lb(
        pio0,
        &i2s_config_default,
        dma_i2s_in_handler,
        &i2s
    );


    puts("i2s_example started.");

    sleep_ms(10);

    r = i2c_write_blocking(
        I2C_PORT,
        i2c_addr,
        i2c_cmd_power_up,
        2,
        false
    );

    printf("Pwr up: %d\n", r);
    // blink_it();

    // Blink the LED so we know we started everything correctly.

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }

    return 0;
}
