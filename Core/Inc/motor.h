//
// Created by CLC on 2025/2/7.
//

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#define ESC_BIT_0 15
#define ESC_BIT_1 30
#define ESC_CMD_BUF_LEN 16


class DuctedMotor {
   public:
    DuctedMotor(TIM_HandleTypeDef* htim, uint32_t channel);
    void init();
    uint16_t prepareDshotPacket(uint16_t value, bool requestTelemetry);
    void pwmWriteDigital(uint16_t* esc_cmd, uint16_t value);
    void handle();
    void stop();

    TIM_HandleTypeDef* htim_;
    uint32_t channel_;
    uint8_t stop_flag_;
};

extern DuctedMotor ducted_motor;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_H
