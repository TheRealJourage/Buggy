// Reference LIN slave for Sim A (tests the real Uno master firmware).
// Behaves like a correct steering-wheel slave: detects the break on RX, waits for 0x55 + PID 0x3C,
// then answers with the same 6-byte frame the RP2040 firmware sends. Values come from the sliders.
// Connect TX/RX/CS to its own MCP2003 instance, exactly like an MCU would be.

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

#define BAUD 19200
#define BREAK_MIN_NS 600000ULL   // 11.5 bit times @ 19200
#define PID_REQUEST 0x3C

typedef enum { S_IDLE, S_WAIT_SYNC, S_WAIT_PID, S_RESPONDING } slave_state_t;

typedef struct {
  pin_t rx, cs;
  uart_dev_t uart;
  uint32_t buttons_attr, pot1_attr, pot2_attr;
  timer_t cs_timer, respond_timer, idle_timer;
  slave_state_t state;
  uint64_t rx_fall_ns;
  uint8_t frame[6];
  uint32_t requests;
} chip_state_t;

static void on_rx_edge(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = user_data;
  uint64_t now = get_sim_nanos();
  if (value == LOW) {
    chip->rx_fall_ns = now;
  } else if (chip->state != S_RESPONDING && now - chip->rx_fall_ns >= BREAK_MIN_NS) {
    chip->state = S_WAIT_SYNC;
  }
}

static void on_rx_byte(void *user_data, uint8_t byte) {
  chip_state_t *chip = user_data;
  switch (chip->state) {
  case S_IDLE:
    // A break also arrives as a 0x00 byte with framing error; accept it in case the edge watch missed it.
    if (byte == 0x00) chip->state = S_WAIT_SYNC;
    break;
  case S_WAIT_SYNC:
    if (byte == 0x55) chip->state = S_WAIT_PID;
    break;
  case S_WAIT_PID:
    if (byte == PID_REQUEST) {
      chip->state = S_RESPONDING;
      timer_start(chip->respond_timer, 100, false);
    } else {
      printf("[LIN slave] unexpected PID 0x%02X\n", byte);
      chip->state = S_IDLE;
    }
    break;
  default:
    break;
  }
}

static void respond(void *user_data) {
  chip_state_t *chip = user_data;
  uint32_t buttons = attr_read(chip->buttons_attr) & 0x7F;
  uint32_t pot1 = attr_read(chip->pot1_attr) & 0xFFF;
  uint32_t pot2 = attr_read(chip->pot2_attr) & 0xFFF;
  chip->frame[0] = buttons;
  chip->frame[1] = pot1 >> 4;
  chip->frame[2] = (pot1 & 0x0F) << 4;
  chip->frame[3] = pot2 >> 4;
  chip->frame[4] = (pot2 & 0x0F) << 4;
  uint16_t sum = 0;
  for (int i = 0; i < 5; i++) sum += chip->frame[i];
  chip->frame[5] = sum & 0xFF;
  uart_write(chip->uart, chip->frame, 6);

  chip->requests++;
  if (chip->requests <= 5 || chip->requests % 50 == 0) {
    printf("[LIN slave] request #%u answered: %02X %02X %02X %02X %02X %02X\n", chip->requests,
           chip->frame[0], chip->frame[1], chip->frame[2], chip->frame[3], chip->frame[4], chip->frame[5]);
  }
}

static void on_write_done(void *user_data) {
  chip_state_t *chip = user_data;
  // Let the echo of our own last byte arrive before listening again.
  timer_start(chip->idle_timer, 1000, false);
}

static void go_idle(void *user_data) {
  ((chip_state_t *)user_data)->state = S_IDLE;
}

static void enable_transceiver(void *user_data) {
  pin_write(((chip_state_t *)user_data)->cs, HIGH);
  printf("[LIN slave] CS high, listening\n");
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));
  pin_init("GND", INPUT);
  pin_t tx = pin_init("TX", OUTPUT_HIGH);
  chip->rx = pin_init("RX", INPUT);
  chip->cs = pin_init("CS", OUTPUT_LOW);
  chip->buttons_attr = attr_init("buttons", 5);
  chip->pot1_attr = attr_init("pot1", 1024);
  chip->pot2_attr = attr_init("pot2", 3000);

  const uart_config_t uart_config = {
    .tx = tx, .rx = chip->rx, .baud_rate = BAUD,
    .rx_data = on_rx_byte, .write_done = on_write_done, .user_data = chip,
  };
  chip->uart = uart_init(&uart_config);

  const pin_watch_config_t rx_watch = { .edge = BOTH, .pin_change = on_rx_edge, .user_data = chip };
  pin_watch(chip->rx, &rx_watch);

  const timer_config_t cs_cfg = { .callback = enable_transceiver, .user_data = chip };
  chip->cs_timer = timer_init(&cs_cfg);
  const timer_config_t respond_cfg = { .callback = respond, .user_data = chip };
  chip->respond_timer = timer_init(&respond_cfg);
  const timer_config_t idle_cfg = { .callback = go_idle, .user_data = chip };
  chip->idle_timer = timer_init(&idle_cfg);

  // TX idles high before CS goes high -> transceiver enters Operation mode, not Transmitter-Off.
  timer_start(chip->cs_timer, 5000, false);
}
