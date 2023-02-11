/**
 * @file tmc_control.cpp
 * @author SJFOM (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-02-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "../include/tmc_control.h"

TMC2300TypeDef* tmc2300;
ConfigurationTypeDef* tmc2300_config;
uint8_t channel = 0;

TMCControl::TMCControl() {
    ;
}
TMCControl::~TMCControl() {
    ;
}

bool TMCControl::init() {
    tmc2300_init(tmc2300, 1, tmc2300_config, tmc2300_defaultRegisterResetState);
    // tmc2300_test();
    return false;
}

void TMCControl::processJob()
{
    ;
    // tmc2300_periodicJob(tmc2300, channel);
}