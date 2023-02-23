#include "soc/rtc_periph.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include <Arduino.h>

#define LED_PIN   2
#define RTC_PIN   12
#define LED_LEVEL 0 // 1

//#define BCM

static esp_err_t pwm_init() {
#ifdef BCM
  enum { LOOP, OFF, CONT1, DELAY, CONT2 };
#else
  enum { LOOP, ON, WAIT1, WAIT2 };
#endif

  constexpr uint16_t DATA_OFFSET = 0;
  constexpr uint16_t CODE_OFFSET = 1;

  const ulp_insn_t program[] = {
#ifdef BCM
    I_MOVI(R2, DATA_OFFSET),
    I_MOVI(R3, 1),

    M_LABEL(LOOP),
    I_LD(R0, R2, 0), // R0 = pwm
    I_ANDR(R0, R0, R3),
    M_BXZ(OFF),
#if LED_LEVEL == 1
    I_WR_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + RTC_PIN, RTC_GPIO_OUT_DATA_W1TS_S + RTC_PIN, 1),
#else
    I_WR_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + RTC_PIN, RTC_GPIO_OUT_DATA_W1TC_S + RTC_PIN, 1),
#endif
    M_BX(CONT1),

    M_LABEL(OFF),
#if LED_LEVEL == 1
    I_WR_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + RTC_PIN, RTC_GPIO_OUT_DATA_W1TC_S + RTC_PIN, 1),
#else
    I_WR_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + RTC_PIN, RTC_GPIO_OUT_DATA_W1TS_S + RTC_PIN, 1),
#endif

    M_LABEL(CONT1),
    I_MOVR(R0, R3),

    M_LABEL(DELAY),
    I_SUBI(R0, R0, 1),
    M_BXZ(CONT2),
    M_BX(DELAY),

    M_LABEL(CONT2),
    I_LSHI(R3, R3, 1),
    I_MOVR(R0, R3),
    M_BL(LOOP, 256),
    I_MOVI(R3, 1),
    M_BX(LOOP)
#else
    I_MOVI(R2, DATA_OFFSET),

    M_LABEL(LOOP),
    I_LD(R0, R2, 0),
    I_MOVI(R1, 255),
    I_SUBR(R1, R1, R0),
    M_BXZ(ON),
#if LED_LEVEL == 1
    I_WR_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + RTC_PIN, RTC_GPIO_OUT_DATA_W1TC_S + RTC_PIN, 1),
#else
    I_WR_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + RTC_PIN, RTC_GPIO_OUT_DATA_W1TS_S + RTC_PIN, 1),
#endif

    M_LABEL(WAIT1),
    I_SUBI(R1, R1, 1),
    M_BXZ(ON),
    M_BX(WAIT1),

    M_LABEL(ON),
    I_ANDR(R0, R0, R0),
    M_BXZ(LOOP),
#if LED_LEVEL == 1
    I_WR_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + RTC_PIN, RTC_GPIO_OUT_DATA_W1TS_S + RTC_PIN, 1),
#else
    I_WR_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + RTC_PIN, RTC_GPIO_OUT_DATA_W1TC_S + RTC_PIN, 1),
#endif

    M_LABEL(WAIT2),
    I_SUBI(R0, R0, 1),
    M_BXZ(LOOP),
    M_BX(WAIT2)
#endif
  };

  size_t size = sizeof(program) / sizeof(ulp_insn_t);
  esp_err_t err;

  RTC_SLOW_MEM[0] = 0;
  err = ulp_process_macros_and_load(CODE_OFFSET, program, &size);
  if (err == ESP_OK) {
    rtc_gpio_init((gpio_num_t)LED_PIN);
    rtc_gpio_set_direction((gpio_num_t)LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pullup_dis((gpio_num_t)LED_PIN);
    rtc_gpio_pulldown_dis((gpio_num_t)LED_PIN);
    rtc_gpio_hold_dis((gpio_num_t)LED_PIN);
    err = ulp_run(CODE_OFFSET);
  }
  return err;
}

static inline void pwm_update(uint8_t pwm) {
  RTC_SLOW_MEM[0] = pwm;
}

void setup() {
  esp_err_t err;

  Serial.begin(115200);
  err = pwm_init();
  if (err != ESP_OK) {
    Serial.printf("ULP init error 0x%X!\n", err);
    Serial.flush();
    esp_deep_sleep_start();
  }
}

void loop() {
  static const uint8_t BRIGHT[] = { 0, 1, 2, 4, 8, 16, 32, 64, 128, 192, 224, 240, 248, 252, 254, 255 };

  static int8_t pos = 0;
  static int8_t delta = 1;

  Serial.printf("\rPWM: %-3u", BRIGHT[pos]);
  pwm_update(BRIGHT[pos]);
  delay(pos ? 100 : 500);
  pos += delta;
  if ((pos < 0) || (pos >= sizeof(BRIGHT) / sizeof(BRIGHT[0]))) {
    delta = -delta;
    if (pos < 0)
      pos = 1;
    else
      pos = sizeof(BRIGHT) / sizeof(BRIGHT[0]) - 2;
  }
}
