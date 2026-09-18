// Reference LIN master for Sim B (tests the real RP2040 slave firmware).
// Every 100ms: break (~937us low, like the Uno's 0x00 @ 9600), 0x55, PID 0x3C, then waits 20ms
// for the 6-byte response. Its own echo is skipped correctly, so any failure reported here comes
// from the slave side. Connect TX/RX/CS to its own MCP2003 instance, exactly like an MCU would be.

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BIT_NS 52083ULL          // 19200 baud
#define BREAK_BITS 18
#define POLL_US 100000
#define TIMEOUT_US 20000
#define PID_REQUEST 0x3C

typedef struct {
  pin_t tx, cs;
  timer_t bit_timer, poll_timer, timeout_timer, cs_timer;
  uint8_t levels[64];
  int level_count, level_pos;
  uint8_t rx_buf[32];
  int rx_len;
  uint32_t polls, ok, timeouts, checksum_errors;
  int last_buttons, last_pot1, last_pot2;
} chip_state_t;

static void queue_byte(chip_state_t *chip, uint8_t byte) {
  chip->levels[chip->level_count++] = 0;
  for (int i = 0; i < 8; i++) chip->levels[chip->level_count++] = (byte >> i) & 1;
  chip->levels[chip->level_count++] = 1;
}

static void on_bit(void *user_data) {
  chip_state_t *chip = user_data;
  if (chip->level_pos >= chip->level_count) {
    timer_stop(chip->bit_timer);
    pin_write(chip->tx, HIGH);
    return;
  }
  pin_write(chip->tx, chip->levels[chip->level_pos++] ? HIGH : LOW);
}

static void on_poll(void *user_data) {
  chip_state_t *chip = user_data;
  chip->level_count = 0;
  chip->level_pos = 0;
  for (int i = 0; i < BREAK_BITS; i++) chip->levels[chip->level_count++] = 0;
  chip->levels[chip->level_count++] = 1;
  chip->levels[chip->level_count++] = 1;
  queue_byte(chip, 0x55);
  queue_byte(chip, PID_REQUEST);
  chip->rx_len = 0;
  chip->polls++;
  timer_start_ns(chip->bit_timer, BIT_NS, true);
  // Same budget as the Uno firmware: 20ms counted from the start of the sync byte.
  timer_start(chip->timeout_timer, (BREAK_BITS + 2) * BIT_NS / 1000 + TIMEOUT_US, false);
}

static void on_rx_byte(void *user_data, uint8_t byte) {
  chip_state_t *chip = user_data;
  if (chip->rx_len < (int)sizeof(chip->rx_buf)) chip->rx_buf[chip->rx_len++] = byte;
}

static void on_timeout(void *user_data) {
  chip_state_t *chip = user_data;
  int start = -1;
  for (int i = 0; i + 1 < chip->rx_len; i++) {
    if (chip->rx_buf[i] == 0x55 && chip->rx_buf[i + 1] == PID_REQUEST) { start = i + 2; break; }
  }
  int available = start < 0 ? 0 : chip->rx_len - start;

  if (start < 0 || available < 6) {
    chip->timeouts++;
    if (chip->timeouts <= 5) {
      printf("[LIN master] poll #%u: TIMEOUT (%d response bytes, echo %s)\n", chip->polls, available,
             start < 0 ? "missing" : "ok");
    }
  } else {
    uint8_t *r = &chip->rx_buf[start];
    uint16_t sum = 0;
    for (int i = 0; i < 5; i++) sum += r[i];
    if ((sum & 0xFF) != r[5]) {
      chip->checksum_errors++;
      if (chip->checksum_errors <= 5) {
        printf("[LIN master] poll #%u: CHECKSUM ERROR %02X %02X %02X %02X %02X %02X\n", chip->polls,
               r[0], r[1], r[2], r[3], r[4], r[5]);
      }
    } else {
      chip->ok++;
      int buttons = r[0] & 0x7F;
      int pot1 = (r[1] << 4) | (r[2] >> 4);
      int pot2 = (r[3] << 4) | (r[4] >> 4);
      if (buttons != chip->last_buttons || abs(pot1 - chip->last_pot1) > 40 || abs(pot2 - chip->last_pot2) > 40) {
        printf("[LIN master] BTN=");
        for (int b = 0; b < 7; b++) printf("%c", (buttons >> b) & 1 ? '1' + b : '-');
        printf(" POT1=%d POT2=%d\n", pot1, pot2);
        chip->last_buttons = buttons;
        chip->last_pot1 = pot1;
        chip->last_pot2 = pot2;
      }
    }
  }

  if (chip->polls % 10 == 0) {
    printf("[LIN master] after %u polls: ok=%u timeout=%u checksum=%u\n", chip->polls, chip->ok,
           chip->timeouts, chip->checksum_errors);
  }
}

static void enable_transceiver(void *user_data) {
  chip_state_t *chip = user_data;
  pin_write(chip->cs, HIGH);
  printf("[LIN master] CS high, polling every 100ms\n");
  timer_start(chip->poll_timer, POLL_US, true);
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));
  chip->last_buttons = chip->last_pot1 = chip->last_pot2 = -1000;
  pin_init("GND", INPUT);
  chip->tx = pin_init("TX", OUTPUT_HIGH);
  chip->cs = pin_init("CS", OUTPUT_LOW);

  const uart_config_t uart_config = {
    .tx = NO_PIN, .rx = pin_init("RX", INPUT), .baud_rate = 19200,
    .rx_data = on_rx_byte, .user_data = chip,
  };
  uart_init(&uart_config);

  const timer_config_t bit_cfg = { .callback = on_bit, .user_data = chip };
  chip->bit_timer = timer_init(&bit_cfg);
  const timer_config_t poll_cfg = { .callback = on_poll, .user_data = chip };
  chip->poll_timer = timer_init(&poll_cfg);
  const timer_config_t timeout_cfg = { .callback = on_timeout, .user_data = chip };
  chip->timeout_timer = timer_init(&timeout_cfg);
  const timer_config_t cs_cfg = { .callback = enable_transceiver, .user_data = chip };
  chip->cs_timer = timer_init(&cs_cfg);

  // Give the slave firmware time to boot (its setup() blinks for ~800ms).
  timer_start(chip->cs_timer, 1000000, false);
}
