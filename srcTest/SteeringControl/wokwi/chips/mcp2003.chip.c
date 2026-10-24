// MCP2003 LIN transceiver model (logic level) for Wokwi.
//
// Behaviour taken from Microchip DS20002230G / DS20005463C:
//  - POR until VBB > 5.5V; drops back to POR when VBB falls below 4.0V.
//  - CS low           -> Ready mode (receiver on, transmitter off)
//  - CS rising edge   -> Operation mode if TXD is high, Transmitter-Off mode if TXD is low
//  - CS falling edge  -> Power-Down (receiver and transmitter off)
//  - Operation: LBUS driven dominant (low) while TXD is low, otherwise released with the
//    internal 30k pull-up. The pull-up is only connected in Operation mode.
//  - RXD is open-drain and mirrors LBUS whenever the receiver is on -> every node sees its own echo.
//  - RXD monitoring: RXD must read high while the bus is recessive, otherwise -> Transmitter-Off.
//  - VREN is an output, high in every mode except POR/Power-Down.
//
// Not modelled: analog levels, slew rate, TXD dominant timeout, thermal shutdown, bus wake-up.
// The supply voltage is not taken from the VBB net (Wokwi nets are digital) but from the "vbb" attribute.

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

typedef enum { M_POR, M_READY, M_OPERATION, M_TOFF, M_POWERDOWN } lin_mode_t;

static const char *mode_names[] = { "POR (VBB too low)", "READY", "OPERATION", "TRANSMITTER-OFF", "POWER-DOWN" };

typedef struct {
  pin_t rxd, cs, txd, lbus, vren;
  uint32_t vbb_attr;
  lin_mode_t mode;
  int id;
  bool fault_reported;
} chip_state_t;

static int instance_count = 0;

static void set_mode(chip_state_t *chip, lin_mode_t mode) {
  if (chip->mode == mode) return;
  chip->mode = mode;
  printf("[MCP2003 #%d] %.1fV -> %s\n", chip->id, attr_read_float(chip->vbb_attr), mode_names[mode]);
}

static void update(chip_state_t *chip) {
  bool receiver_on = chip->mode == M_READY || chip->mode == M_OPERATION || chip->mode == M_TOFF;
  bool transmitter_on = chip->mode == M_OPERATION;

  if (transmitter_on) {
    pin_mode(chip->lbus, pin_read(chip->txd) == LOW ? OUTPUT_LOW : INPUT_PULLUP);
  } else {
    pin_mode(chip->lbus, INPUT);
  }

  bool bus_high = pin_read(chip->lbus) == HIGH;

  if (receiver_on && !bus_high) {
    pin_mode(chip->rxd, OUTPUT_LOW);
  } else {
    pin_mode(chip->rxd, INPUT);
    if (receiver_on && bus_high && pin_read(chip->rxd) == LOW) {
      if (!chip->fault_reported) {
        printf("[MCP2003 #%d] FAULT: RXD low while bus recessive (missing RXD pull-up?)\n", chip->id);
        chip->fault_reported = true;
      }
      if (chip->mode == M_OPERATION) {
        set_mode(chip, M_TOFF);
        pin_mode(chip->lbus, INPUT);
      }
    }
  }

  pin_mode(chip->vren, (chip->mode == M_POR || chip->mode == M_POWERDOWN) ? INPUT : OUTPUT_HIGH);
}

static void on_cs_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = user_data;
  if (chip->mode == M_POR) return;
  if (value == HIGH) {
    set_mode(chip, pin_read(chip->txd) == HIGH ? M_OPERATION : M_TOFF);
  } else {
    set_mode(chip, chip->mode == M_READY ? M_READY : M_POWERDOWN);
  }
  update(chip);
}

static void on_txd_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = user_data;
  if (chip->mode == M_TOFF && value == HIGH && pin_read(chip->cs) == HIGH) {
    chip->fault_reported = false;
    set_mode(chip, M_OPERATION);
  }
  update(chip);
}

static void on_lbus_change(void *user_data, pin_t pin, uint32_t value) {
  update((chip_state_t *)user_data);
}

static void on_supervisor(void *user_data) {
  chip_state_t *chip = user_data;
  float vbb = attr_read_float(chip->vbb_attr);
  if (chip->mode == M_POR) {
    if (vbb > 5.5f) {
      if (pin_read(chip->cs) == HIGH) {
        set_mode(chip, pin_read(chip->txd) == HIGH ? M_OPERATION : M_TOFF);
      } else {
        set_mode(chip, M_READY);
      }
      update(chip);
    }
  } else if (vbb < 4.0f) {
    set_mode(chip, M_POR);
    update(chip);
  }
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));
  ++instance_count;
  chip->rxd = pin_init("RXD", INPUT);
  chip->cs = pin_init("CS", INPUT_PULLDOWN);
  chip->txd = pin_init("TXD", INPUT);
  chip->lbus = pin_init("LBUS", INPUT);
  chip->vren = pin_init("VREN", INPUT);
  pin_init("WAKE", INPUT_PULLUP);
  pin_init("VSS", INPUT);
  pin_init("VBB", INPUT);
  chip->vbb_attr = attr_init_float("vbb", 12.0f);
  // "node" attribute labels the log lines (1 = slave side, 2 = master side, as in the README).
  chip->id = attr_read(attr_init("node", instance_count));
  chip->mode = M_POR;

  const pin_watch_config_t cs_watch = { .edge = BOTH, .pin_change = on_cs_change, .user_data = chip };
  pin_watch(chip->cs, &cs_watch);
  const pin_watch_config_t txd_watch = { .edge = BOTH, .pin_change = on_txd_change, .user_data = chip };
  pin_watch(chip->txd, &txd_watch);
  const pin_watch_config_t lbus_watch = { .edge = BOTH, .pin_change = on_lbus_change, .user_data = chip };
  pin_watch(chip->lbus, &lbus_watch);

  const timer_config_t supervisor = { .callback = on_supervisor, .user_data = chip };
  timer_start(timer_init(&supervisor), 1000, true);

  printf("[MCP2003 #%d] power-up, VBB = %.1fV\n", chip->id, attr_read_float(chip->vbb_attr));
}
