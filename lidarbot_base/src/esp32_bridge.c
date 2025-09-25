#include "lidarbot_base/esp32_bridge.h"
#include <stdint.h>
#include <unistd.h>
#include <wiringPiI2C.h>

// --- Protocolo (de tu firmware ESP32) ---
#define CMD_SET_SPEEDS  0x01
#define CMD_STOP        0x02
#define CMD_ENABLE      0x03
#define CMD_DISABLE     0x04

static int g_fd = -1;
static int g_addr = 0x28;  // dirección I2C del ESP32 (ajusta si hace falta)

static uint8_t crc_xor(const uint8_t *b, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i) c ^= b[i];
  return c;
}

static int i2c_write_buf(const uint8_t *buf, size_t n) {
  if (g_fd < 0) return -1;
  ssize_t w = write(g_fd, buf, n);
  return (w == (ssize_t)n) ? 0 : -1;
}

int esp32_bridge_init(int i2c_addr) {
  g_addr = i2c_addr;
  g_fd = wiringPiI2CSetup(g_addr);
  return (g_fd >= 0) ? 0 : -1;
}

static int send_cmd_simple(uint8_t cmd) {
  uint8_t pkt[2];
  pkt[0] = cmd;
  pkt[1] = cmd;  // CRC = XOR(cmd)
  return i2c_write_buf(pkt, 2);
}

int esp32_bridge_enable(void)  { return send_cmd_simple(CMD_ENABLE);  }
int esp32_bridge_disable(void) { return send_cmd_simple(CMD_DISABLE); }
int esp32_bridge_stop(void)    { return send_cmd_simple(CMD_STOP);    }

int esp32_bridge_set_speeds(int16_t left, int16_t right) {
  uint8_t pkt[6];
  pkt[0] = CMD_SET_SPEEDS;
  pkt[1] = (uint8_t)((left  >> 8) & 0xFF);
  pkt[2] = (uint8_t)( left        & 0xFF);
  pkt[3] = (uint8_t)((right >> 8) & 0xFF);
  pkt[4] = (uint8_t)( right       & 0xFF);
  pkt[5] = crc_xor(pkt, 5);
  return i2c_write_buf(pkt, 6);
}
