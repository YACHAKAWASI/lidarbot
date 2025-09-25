// Driver de motores usando ESP32 por I2C (reemplaza PCA9685+TB6612)

#include "lidarbot_base/MotorDriver.h"     // define UBYTE, UWORD, DIR, MOTORA/MOTORB
#include "lidarbot_base/esp32_bridge.h"
#include <stdio.h>
#include <stdint.h>

#define ESP32_I2C_ADDR  0x28     // misma dirección que en el firmware
#define INPUT_ABS_MAX   1000     // escala que espera el ESP32

static int16_t g_last_left  = 0;
static int16_t g_last_right = 0;

static int16_t pct_to_cmd(UWORD speed_pct, DIR dir) {
  if (speed_pct > 100) speed_pct = 100;
  // 0..100% -> 0..INPUT_ABS_MAX
  int16_t val = (int16_t)((speed_pct * INPUT_ABS_MAX) / 100);
  if (dir == BACKWARD) val = -val;
  return val;
}

void Motor_Init(void)
{
  if (esp32_bridge_init(ESP32_I2C_ADDR) != 0) {
    fprintf(stderr, "[MotorDriver] ERROR: esp32_bridge_init() falló\n");
  }
  esp32_bridge_enable();
  g_last_left = 0;
  g_last_right = 0;
  esp32_bridge_stop();
}

/**
 * Ejecuta un solo motor con % de velocidad.
 * - motor: MOTORA (izq) o MOTORB (der)
 * - dir:   FORWARD / BACKWARD
 * - speed: 0..100 (%)
 */
void Motor_Run(UBYTE motor, DIR dir, UWORD speed)
{
  int16_t cmd = pct_to_cmd(speed, dir);

  if (motor == MOTORA) {
    g_last_left = cmd;
  } else {  // MOTORB
    g_last_right = cmd;
  }

  esp32_bridge_set_speeds(g_last_left, g_last_right);
}

/** Detiene el motor indicado (si quieres parar ambos, llama 2 veces o usa esp32_bridge_stop()) */
void Motor_Stop(UBYTE motor)
{
  if (motor == MOTORA) {
    g_last_left = 0;
  } else { // MOTORB
    g_last_right = 0;
  }
  esp32_bridge_set_speeds(g_last_left, g_last_right);
}
