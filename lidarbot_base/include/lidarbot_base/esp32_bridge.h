#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Inicializa el enlace I2C con el ESP32 (devuelve 0 si OK)
int esp32_bridge_init(int i2c_addr);

// Enviar comando ENABLE / DISABLE / STOP (0 si OK)
int esp32_bridge_enable(void);
int esp32_bridge_disable(void);
int esp32_bridge_stop(void);

// Enviar velocidades: rango [-1000..+1000] por rueda (0 si OK)
int esp32_bridge_set_speeds(int16_t left, int16_t right);

#ifdef __cplusplus
}
#endif
