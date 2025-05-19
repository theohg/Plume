/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the drv8214_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#include "drv8214.h"

// Initialize the motor driver with default settings
uint8_t DRV8214::init(const DRV8214_Config& cfg) {

    // Store the configuration settings
    config = cfg;

    disableHbridge(); // Disable H-bridge to be able to configure the driver
    setControlMode(config.control_mode, config.I2CControlled); // Default to PWM control with I2C enabled
    setRegulationMode(config.regulation_mode); // Default to SPEED regulation
    setVoltageRange(config.voltage_range);  // Default to 0 V - 3.92 V range
    setOvervoltageProtection(config.ovp_enabled); // Default to overvoltage protection enabled
    setCurrentRegMode(config.current_reg_mode); // Default to no current regulation
    setStallDetection(config.stall_enabled); // Default to stall detection enabled
    setStallBehavior(config.stall_behavior); // Default to outputs disabled on stall
    enableStallInterrupt(); // Default to enable stall interrupt
    enableCountThresholdInterrupt(); // Default to enable count threshold interrupt
    setBridgeBehaviorThresholdReached(config.bridge_behavior_thr_reached); // Default to H-bridge stays enabled when RC_CNT exceeds threshold
    setInternalVoltageReference(0); // Default to internal voltage reference of 500mV
    setSoftStartStop(config.soft_start_stop_enabled); // Default to soft start/stop disbaled
    setInrushDuration(config.inrush_duration); // Default to 500 ms
    setResistanceRelatedParameters(); // configure the INV_R and INV_R_SCALE values
    enableRippleCount(); // Default to enable ripple counting
    resetRippleCounter(); // Default to reset ripple counter
    setKMC(config.kmc); // Default to KMC = 30
    setKMCScale(config.kmc_scale); // Default to KMC scale factor = 24 x 2^13
    brakeMotor(true); // Default to brake motor
    enableErrorCorrection(false); // Default to disable error correction
    if (config.verbose) {printMotorConfig(true);}

    return DRV8214_OK; // Return success code
}

// --- Helper Functions ---

uint8_t DRV8214::getDriverAdress() {
    return address;
}

uint8_t DRV8214::getDriverID() {
    return driver_ID;
}

uint8_t DRV8214::getSenseResistor() {
    return Ripropri;
}

uint8_t DRV8214::getRipplesPerRevolution() {
    return ripples_per_revolution;
}

uint8_t DRV8214::getFaultStatus() {
    return drv8214_i2c_read_register(address, DRV8214_FAULT);
}

uint32_t DRV8214::getMotorSpeedRPM() {
    return ((drv8214_i2c_read_register(address, DRV8214_RC_STATUS1) * config.w_scale * 60) / (2 * M_PI * ripples_per_revolution));
}

uint16_t DRV8214::getMotorSpeedRAD() {
    return ((drv8214_i2c_read_register(address, DRV8214_RC_STATUS1) * config.w_scale) / ripples_per_revolution);
}

uint16_t DRV8214::getMotorSpeedShaftRPM() {
    return (getMotorSpeedRPM() / motor_reduction_ratio);
}

uint16_t DRV8214::getMotorSpeedShaftRAD() {
    return (getMotorSpeedRAD() / motor_reduction_ratio);
}

uint8_t DRV8214::getMotorSpeedRegister() {
    return drv8214_i2c_read_register(address, DRV8214_RC_STATUS1);
}

uint16_t DRV8214::getRippleCount() {
    return (drv8214_i2c_read_register(address, DRV8214_RC_STATUS3) << 8) | drv8214_i2c_read_register(address, DRV8214_RC_STATUS2);
}

float DRV8214::getMotorVoltage() {
    if (config.voltage_range) {
        float voltage = (drv8214_i2c_read_register(address, DRV8214_REG_STATUS1) / 255.0f) * 3.92f;
        return voltage;
    } else {
        if (config.ovp_enabled) {
            // If OVP is enabled, the maximum voltage is 11 V
            if (drv8214_i2c_read_register(address, DRV8214_REG_STATUS1) > 0xB0) {
                return 11.0f;
            } else {     // 00h corresponds to 0 V and B0h corresponds to 11 V.
                float voltage = (drv8214_i2c_read_register(address, DRV8214_REG_STATUS1) / 176.0f) * 11.0f;
                return voltage;
            }
        } else {
            float voltage = (drv8214_i2c_read_register(address, DRV8214_REG_STATUS1) / 255.0f) * 15.7f;
            return voltage;
        }
    }
}

uint8_t DRV8214::getMotorVoltageRegister() {
    return drv8214_i2c_read_register(address, DRV8214_REG_STATUS1);
}

float DRV8214::getMotorCurrent() {
    // 00h corresponds to 0 A and C0h corresponds to the maximum value set by the CS_GAIN_SEL bit
    float current = (drv8214_i2c_read_register(address, DRV8214_REG_STATUS2) / 192.0f) * config.MaxCurrent;
    return current;
}

uint8_t DRV8214::getMotorCurrentRegister() {
    return drv8214_i2c_read_register(address, DRV8214_REG_STATUS2);
}

uint8_t DRV8214::getDutyCycle() {
    uint8_t dutyCycle = drv8214_i2c_read_register(address, DRV8214_REG_STATUS3) & REG_STATUS3_IN_DUTY;
    return (dutyCycle * 100) / 63; // Convert 6-bit value to percentage
}

uint8_t DRV8214::getCONFIG0() {
    return drv8214_i2c_read_register(address, DRV8214_CONFIG0);
}

uint16_t DRV8214::getInrushDuration() {
    return (drv8214_i2c_read_register(address, DRV8214_CONFIG1) << 8) | drv8214_i2c_read_register(address, DRV8214_CONFIG2);
}

uint8_t DRV8214::getCONFIG3() {
    return drv8214_i2c_read_register(address, DRV8214_CONFIG3);
}

uint8_t DRV8214::getCONFIG4() {
    return drv8214_i2c_read_register(address, DRV8214_CONFIG4);
}

uint8_t DRV8214::getREG_CTRL0() {
    return drv8214_i2c_read_register(address, DRV8214_REG_CTRL0);
}

uint8_t DRV8214::getREG_CTRL1() {
    return drv8214_i2c_read_register(address, DRV8214_REG_CTRL1);
}

uint8_t DRV8214::getREG_CTRL2() {
    return drv8214_i2c_read_register(address, DRV8214_REG_CTRL2);
}

uint8_t DRV8214::getRC_CTRL0() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL0);
}

uint8_t DRV8214::getRC_CTRL1() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL1);
}

uint8_t DRV8214::getRC_CTRL2() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL2);
}

uint16_t DRV8214::getRippleThreshold()
{
    uint8_t ctrl2 = drv8214_i2c_read_register(address, DRV8214_RC_CTRL2);
    uint8_t ctrl1 = drv8214_i2c_read_register(address, DRV8214_RC_CTRL1);
    // top two bits are bits 1..0 in ctrl2
    uint16_t thr_high = (ctrl2 & 0x03) << 8; // shift them to bits 9..8
    uint16_t thr_low  = ctrl1;               // bits 7..0
    return (thr_high | thr_low); 
}

uint16_t DRV8214::getRippleThresholdScaled() {
    getRippleThresholdScale();
    if (config.ripple_threshold_scale  == 0) { return getRippleThreshold() * 2;  }
    if (config.ripple_threshold_scale  == 1) { return getRippleThreshold() * 8;  }
    if (config.ripple_threshold_scale  == 2) { return getRippleThreshold() * 16; }
    if (config.ripple_threshold_scale  == 3) { return getRippleThreshold() * 64; }
    return getRippleThreshold() * config.ripple_threshold_scale;
}

uint16_t DRV8214::getRippleThresholdScale() {
    config.ripple_threshold_scale = (drv8214_i2c_read_register(address, DRV8214_RC_CTRL2) & RC_CTRL2_RC_THR_SCALE) >> 2;
    return config.ripple_threshold_scale;
}

uint8_t DRV8214::getKMC() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL4);
}

uint8_t DRV8214::getKMCScale() {
    return (drv8214_i2c_read_register(address, DRV8214_RC_CTRL2) >> 4) & 0x03;
}

uint8_t DRV8214::getFilterDamping() {
    return (drv8214_i2c_read_register(address, DRV8214_RC_CTRL5) >> 4) & 0x0F;
}

uint8_t DRV8214::getRC_CTRL6() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL6);
}

uint8_t DRV8214::getRC_CTRL7() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL7);
}

uint8_t DRV8214::getRC_CTRL8() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL8);
}

// --- Control Functions ---
void DRV8214::enableHbridge() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_EN_OUT, true);
}

void DRV8214::disableHbridge() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_EN_OUT, false);
}

void DRV8214::setStallDetection(bool stall_en) {
    config.stall_enabled = stall_en;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG3, CONFIG0_EN_STALL, stall_en);
}

void DRV8214::setVoltageRange(bool range) {
    config.voltage_range = range;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_VM_GAIN_SEL, range);
}

void DRV8214::setOvervoltageProtection(bool OVP) {
    config.ovp_enabled = OVP;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_EN_OVP, true);
}

void DRV8214::resetRippleCounter() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_CLR_CNT, true);
}

void DRV8214::resetFaultFlags() {
    disableHbridge();
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_CLR_FLT, true);
    enableHbridge();
}

void DRV8214::enableDutyCycleControl() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_DUTY_CTRL, true);
}

void DRV8214::disableDutyCycleControl() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_DUTY_CTRL, false);
}

void DRV8214::setInrushDuration(uint16_t threshold) {
    drv8214_i2c_write_register(address, DRV8214_CONFIG1, (threshold >> 8) & 0xFF);
    drv8214_i2c_write_register(address, DRV8214_CONFIG2, threshold & 0xFF);
}

void DRV8214::setCurrentRegMode(uint8_t mode) {

    if (mode > 3) { mode = 3; } // Cap mode to 3
    switch (mode){
    case 0: // 0b00
        mode = 0x00; // No current regulation at anytime
        config.current_reg_mode = 0;
        break;
    case 1: // 0b01
        mode = 0x40; // Current regulation at all time if stall detection is desabled
        config.current_reg_mode = 1; // Current regulation during tinrush only if stall detection is enabled
        break;       
    case 2: // 0b10
        mode = 0x80; // Current regulation at all time
        config.current_reg_mode = 2;
        break;
    case 3: // 0b11
        mode = 0xC0; // Current regulation at all time
        config.current_reg_mode = 3;
        break;
    default:
        break;
    }
    drv8214_i2c_modify_register_bits(address, DRV8214_CONFIG3, CONFIG3_IMODE, mode);
}

void DRV8214::setStallBehavior(bool behavior) {
    // The SMODE bit programs the device's response to a stall condition. 
    // When SMODE = 0b, the STALL bit becomes 1b, the outputs are disabled
    // When SMODE = 1b, the STALL bit becomes 1b, but the outputs continue to drive current into the motor
    config.stall_behavior = behavior;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG3, CONFIG3_SMODE, behavior);
}

void DRV8214::setInternalVoltageReference(float reference_voltage) {
    // VVREF must be lower than VVM by at least 1.25 V. The maximum recommended value of VVREF is 3.3 V. 
    // If INT_VREF bit is set to 1b, VVREF is internally selected with a fixed value of 500 mV.
    if (reference_voltage == 0) { 
        config.Vref = 0.5f; // Default
        drv8214_i2c_modify_register(address, DRV8214_CONFIG3, CONFIG3_INT_VREF, true);
    } else { 
        config.Vref = reference_voltage;
        drv8214_i2c_modify_register(address, DRV8214_CONFIG3, CONFIG3_INT_VREF, false);
    }
}

void DRV8214::configureConfig3(uint8_t config3) {
    drv8214_i2c_write_register(address, DRV8214_CONFIG3, config3);
}

void DRV8214::setI2CControl(bool I2CControl) {
    config.I2CControlled = I2CControl;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_BC, I2CControl);
}

void DRV8214::enablePWMControl() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_PMODE, true);
}

void DRV8214::enablePHENControl() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_PMODE, false);
}

void DRV8214::enableStallInterrupt() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_STALL_REP, true);
}

void DRV8214::disableStallInterrupt() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_STALL_REP, false);
}

void DRV8214::enableCountThresholdInterrupt() {
    drv8214_i2c_modify_register_bits(address, DRV8214_CONFIG4, CONFIG4_RC_REP, 0b10000000);
}

void DRV8214::disableCountThresholdInterrupt() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_RC_REP, false);
}

void DRV8214::setBridgeBehaviorThresholdReached(bool stops) {
    // stops = 0b: H-bridge stays enabled when RC_CNT exceeds threshold
    // stops = 1b: H-bridge is disabled (High-Z) when RC_CNT exceeds threshold
    config.bridge_behavior_thr_reached = stops; 
    drv8214_i2c_modify_register(address, DRV8214_RC_CTRL0, RC_CTRL0_RC_HIZ, stops);
}

void DRV8214::setSoftStartStop(bool enable) {
    drv8214_i2c_modify_register(address, DRV8214_REG_CTRL0, REG_CTRL0_EN_SS, enable);
}

void DRV8214::configureControl0(uint8_t control0) {
    drv8214_i2c_write_register(address, DRV8214_REG_CTRL0, control0);
}

void DRV8214::setRegulationAndStallCurrent(float requested_current) {
    // According to Table 8-7 "CS_GAIN_SEL Settings":
    //   000b =>  225 μA/A, max current 4 A
    //   001b =>  225 μA/A, max current 2 A
    //   010b => 1125 μA/A, max current 1 A
    //   011b => 1125 μA/A, max current 0.5 A
    //   1X0b => 5560 μA/A, max current 0.25 A
    //   1X1b => 5560 μA/A, max current 0.125 A

    uint8_t cs_gain_sel;

    // Clamp very low currents (<0.125 A) to the lowest recommended setting:
    if (requested_current < 0.125f) {
        cs_gain_sel = 0b111; // 5560 μA/A, max current 0.125 A
        config.Aipropri = 5560e-6; // Convert μA/A to A/A
        config.MaxCurrent = 0.125f;
    }
    else if (requested_current < 0.25f) {
        cs_gain_sel = 0b110; // 5560 μA/A, max current 0.25 A
        config.Aipropri = 5560e-6;
        config.MaxCurrent = 0.25f;
    }
    else if (requested_current < 0.5f) {
        cs_gain_sel = 0b011; // 1125 μA/A, max current 0.5 A
        config.Aipropri = 1125e-6;
        config.MaxCurrent = 0.5f;
    }
    else if (requested_current < 1.0f) {
        cs_gain_sel = 0b010; // 1125 μA/A, max current 1 A
        config.Aipropri = 1125e-6;
        config.MaxCurrent = 1.0f;
    }
    else if (requested_current < 2.0f) {
        cs_gain_sel = 0b001; // 225 μA/A, max current 2 A
        config.Aipropri = 225e-6;
        config.MaxCurrent = 2.0f;
    }
    else {
        // For >= 2.0 A, recommended setting is 000b (max current 4 A).
        // Also clamp above 4 A to the same setting (since 4 A is the top of the recommended range).
        cs_gain_sel = 0b000; // 225 μA/A, max current 4 A
        config.Aipropri = 225e-6;
        config.MaxCurrent = 4.0f;
    }

    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL0, RC_CTRL0_CS_GAIN_SEL, cs_gain_sel);

    // Update Itrip calculation with the new scale
    config.Itrip = config.Vref / (Ripropri * config.Aipropri);

    if (config.verbose) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "Requested Itrip = %f A => Chosen CS_GAIN_SEL: 0b%d => Aipropri = %f uA/A => Actual Itrip = %f A\n", requested_current, cs_gain_sel, config.Aipropri, config.Itrip);
        drvPrint(buffer);
    }
}

void DRV8214::setRippleSpeed(uint16_t speed) {
    if (speed > motor_max_rpm) { speed = motor_max_rpm; } // Cap speed to the maximum RPM of the motor

    // Find the corresponding ripples frequency (Hz) value
    uint32_t ripple_speed = (speed * motor_reduction_ratio * ripples_per_revolution * 2 * M_PI) / 60;

    // Define max feasible ripple speed based on 8-bit WSET_VSET and max scaling factor (128)
    const uint16_t MAX_SPEED = 32640; // 255 * 128 = 32640 rad/s
    
    // Cap threshold to the maximum feasible value
    if (ripple_speed > MAX_SPEED) { ripple_speed = MAX_SPEED; }

    // Define scaling factors and corresponding bit values
    struct ScaleOption {
        uint16_t scale;
        uint8_t bits;
    };

    ScaleOption scaleOptions[] = {
        {16, 0b00},
        {32, 0b01},
        {64, 0b10},
        {128, 0b11}
    };

    // Find the optimal scaling factor and 10-bit value
    uint16_t WSET_VSET = ripple_speed;
    uint8_t W_SCALE = 0b00;

    for (const auto &option : scaleOptions) {
        if (ripple_speed >= option.scale) {
            WSET_VSET = ripple_speed / option.scale;
            if (WSET_VSET <= 255) { // Ensure WSET_VSET fits within 8 bits
                W_SCALE = option.bits;
                break;
            }
        }
    }
    config.w_scale = scaleOptions[W_SCALE].scale;

    WSET_VSET = WSET_VSET & 0xFF; // Ensure WSET_VSET fits within 8 bits

    if (config.verbose) {
        char buffer[256];  // Adjust the buffer size as needed
        snprintf(buffer, sizeof(buffer), "WSET_VSET: %d | W_SCALE: %d or 0b%d | Effective Target Speed: %d rad/s\n", WSET_VSET, config.w_scale, W_SCALE, WSET_VSET * config.w_scale);
        drvPrint(buffer);
    }
    drv8214_i2c_write_register(address, DRV8214_REG_CTRL1, WSET_VSET);
    drv8214_i2c_modify_register_bits(address, DRV8214_REG_CTRL0, REG_CTRL0_W_SCALE, W_SCALE);
}

void DRV8214::setVoltageSpeed(float voltage) {
    if (voltage < 0.0f) { voltage = 0.0f; } // Ensure voltage is non-negative

    // Depending on the VM_GAIN_SEL bit (voltage_range), clamp and scale accordingly
    if (config.voltage_range) {
        // VM_GAIN_SEL = 1 → Range: 0 to 3.92 V
        if (voltage > 3.92f) { voltage = 3.92f; }
        // Apply formula from table 8-23: WSET_VSET = voltage * (255 / 3.92)
        float scaled = voltage * (255.0f / 3.92f);
        uint8_t regVal = static_cast<uint8_t>(scaled + 0.5f); // Round to nearest integer
        drv8214_i2c_write_register(address, DRV8214_REG_CTRL1, regVal);
    } else {
        // VM_GAIN_SEL = 0 → Range: 0 to 15.7 V
        if (voltage > 15.7f) { voltage = 11.0f; } // Cap voltage to 11 V because of Overvoltage Protection
        // Apply formula from table 8-23: WSET_VSET = voltage * (255 / 15.7)
        float scaled = voltage * (255.0f / 15.7f);
        uint8_t regVal = static_cast<uint8_t>(scaled + 0.5f); // Round to nearest integer
        drv8214_i2c_write_register(address, DRV8214_REG_CTRL1, regVal);
    }
}

void DRV8214::configureControl2(uint8_t control2) {
    drv8214_i2c_write_register(address, DRV8214_REG_CTRL2, control2);
}

void DRV8214::enableRippleCount(bool enable) {
    drv8214_i2c_modify_register(address, DRV8214_RC_CTRL0, RC_CTRL0_EN_RC, enable);
}

void DRV8214::enableErrorCorrection(bool enable) {
    drv8214_i2c_modify_register(address, DRV8214_RC_CTRL0, RC_CTRL0_DIS_EC, !enable);
}

void DRV8214::configureRippleCount0(uint8_t ripple0) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL0, ripple0);
}

void DRV8214::setRippleCountThreshold(uint16_t threshold) {
    // Define max feasible threshold based on 10-bit RC_THR and max scaling factor (64)
    const uint16_t MAX_THRESHOLD = 65535; // 1024 * 64 = 65536
    
    // Cap threshold to the maximum feasible value
    if (threshold > MAX_THRESHOLD) {
        threshold = MAX_THRESHOLD;
    }

    // Define scaling factors and corresponding bit values
    struct ScaleOption {
        uint16_t scale;
        uint8_t bits;
    };

    ScaleOption scaleOptions[] = {
        {2, 0b00},
        {8, 0b01},
        {16, 0b10},
        {64, 0b11}
    };
    
    // Find the optimal scaling factor and 10-bit value
    uint16_t rc_thr = threshold;
    uint8_t rc_thr_scale_bits = 0b00;

    for (const auto &option : scaleOptions) {
        if (threshold >= option.scale) {
            rc_thr = threshold / option.scale;
            if (rc_thr < 1024) { // Ensure it fits in 10 bits
                rc_thr_scale_bits = option.bits;
                break;
            }
        }
    }
    if (config.verbose) {
        char buffer[256];  // Adjust the buffer size as needed
        snprintf(buffer, sizeof(buffer), "RC_THR: %d | RC_THR_SCALE: %d ", rc_thr, rc_thr_scale_bits);
        drvPrint(buffer);
    }
    config.ripple_threshold = rc_thr;
    config.ripple_threshold_scale = rc_thr_scale_bits;
    // Ensure rc_thr fits within 10 bits
    rc_thr = rc_thr & 0x3FF;
    
    // Split into lower 8 bits and upper 2 bits
    uint8_t rc_thr_low  = rc_thr & 0xFF;         // bits 7..0
    uint8_t rc_thr_high = (rc_thr >> 8) & 0x03;  // bits 9..8
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL1, rc_thr_low);
    setRippleThresholdScale(rc_thr_scale_bits);
    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL2, RC_CTRL2_RC_THR_HIGH, rc_thr_high);
}

void DRV8214::setRippleThresholdScale(uint8_t scale) {
    scale = scale & 0x03;
    scale = scale << 2; //make sure the 2 bits of scale are placed on bit 2 and 3
    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL2, RC_CTRL2_RC_THR_SCALE, scale);
}

void DRV8214::setKMCScale(uint8_t scale) {
    scale = scale << 4; //make sure the 2 bits of scale are placed on bit 4 and 5
    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL2, RC_CTRL2_KMC_SCALE, scale);
}

void DRV8214::setMotorInverseResistance(uint8_t resistance) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL3, resistance);
}

void DRV8214::setMotorInverseResistanceScale(uint8_t scale) {
    scale = scale << 6; //make sure the 2 bits of scale are placed on bit 6 and 7
    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL2, RC_CTRL2_INV_R_SCALE, scale);
}

void DRV8214::setResistanceRelatedParameters() {
    // Possible values of INV_R_SCALE and corresponding register bit settings
    const uint16_t scaleValues[4] = {2, 64, 1024, 8192};
    const uint8_t scaleBits[4] = {0b00, 0b01, 0b10, 0b11};

    // Default values (minimum valid values)
    uint8_t bestScaleBits = 0b00;  // Default to scale 2
    uint8_t bestInvR = 1;          // Minimum valid INV_R

    // Iterate from largest scale to smallest for best resolution
    for (int i = 3; i >= 0; --i)
    {
        float candidate = scaleValues[i] / motor_internal_resistance;
        float rounded = roundf(candidate);

        // Ensure the value is at least 1
        if (rounded < 1.0f) {
            rounded = 1.0f;
        }

        // If within valid range, select this scale and break
        if (rounded <= 255.0f)
        {
            bestScaleBits = scaleBits[i];
            bestInvR = static_cast<uint8_t>(rounded);
            break;
        }
    }
    config.inv_r = bestInvR;
    config.inv_r_scale = scaleValues[bestScaleBits];

    // Set the selected INV_R and INV_R_SCALE
    setMotorInverseResistanceScale(bestScaleBits);
    setMotorInverseResistance(bestInvR);
}

void DRV8214::setKMC(uint8_t factor) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL4, factor);
}

void DRV8214::setFilterDamping(uint8_t damping) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL5, damping);
}

void DRV8214::configureRippleCount6(uint8_t ripple6) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL6, ripple6);
}

void DRV8214::configureRippleCount7(uint8_t ripple7) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL7, ripple7);
}

void DRV8214::configureRippleCount8(uint8_t ripple8) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL8, ripple8);
}

// --- Motor Control Functions ---
void DRV8214::setControlMode(ControlMode mode, bool I2CControl) {
    config.control_mode = mode;
    setI2CControl(I2CControl);
    switch (mode) {
        case PWM:
            enablePWMControl();
            break;
        case PH_EN:
            enablePHENControl();
            break;
    }
}

void DRV8214::setRegulationMode(RegulationMode regulation) {
    uint8_t reg_ctrl = 0;  // Default value
    switch (regulation) {
        case CURRENT_FIXED:
            reg_ctrl = (0b00 << 3);  // Fixed Off-Time Current Regulation
            break;
        case CURRENT_CYCLES:
            reg_ctrl = (0b01 << 3);  // Cycle-By-Cycle Current Regulation
            break;
        case SPEED:
            reg_ctrl = (0b10 << 3);  // Speed Regulation
            enableRippleCount();
            break;
        case VOLTAGE:
            reg_ctrl = (0b11 << 3);  // Voltage Regulation
            break;
    }
    config.regulation_mode = regulation;
    drv8214_i2c_modify_register_bits(address, DRV8214_REG_CTRL0, REG_CTRL0_REG_CTRL, reg_ctrl);
}

void DRV8214::turnForward(uint16_t speed, float voltage, float requested_current) {
    disableHbridge();
    switch (config.regulation_mode) {
        case CURRENT_FIXED: // No speed control if using I2C (will applied full tension to motor)
            setRegulationAndStallCurrent(requested_current);
            break;
        case CURRENT_CYCLES: // No speed control if using I2C (will applied full tension to motor)
            setRegulationAndStallCurrent(requested_current);
            break;
        case SPEED: 
            setRippleSpeed(speed);
            break;
        case VOLTAGE:
            setVoltageSpeed(voltage);
            break;
    }
    
    if (config.control_mode == PWM) {
        // Table 8-5 => Forward => Input1=1, Input2=0
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, true);  // Input1=1
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, false); // Input2=0
    } 
    else { // PH/EN mode
        // Table 8-4 => Forward => EN=1, PH=1
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, true); // EN=1
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, true); // PH=1
    }
    enableHbridge();
    if (config.verbose) { drvPrint("Turning Forward\n"); }
}

void DRV8214::turnReverse(uint16_t speed, float voltage, float requested_current) {
    enableHbridge();
    switch (config.regulation_mode) {
        case CURRENT_FIXED: // No speed control if using I2C (will applied full tension to motor)
            setRegulationAndStallCurrent(requested_current);
            break;
        case CURRENT_CYCLES: // No speed control if using I2C (will applied full tension to motor)
            setRegulationAndStallCurrent(requested_current);
            break;
        case SPEED: 
            setRippleSpeed(speed);
            break;
        case VOLTAGE:
            setVoltageSpeed(voltage);
            break;
    }
    if (config.control_mode == PWM) {
        // Table 8-5 => Reverse => Input1=0, Input2=1
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, false);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, true);
    } 
    else { // PH/EN mode
        // Table 8-4 => Reverse => EN=1, PH=0
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, true);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, false);
    }
    if (config.verbose) { drvPrint("Turning Reverse\n"); }
}

void DRV8214::brakeMotor(bool initial_config) {
    enableHbridge();
    if (config.control_mode == PWM) {
        // Table 8-5 => Brake => Input1=1, Input2=1 => both outputs low
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, true);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, true);
    }
    else { // PH/EN mode
        // Table 8-4 => Brake => EN=0 => outputs go low
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, false);
        // PH can be 0 or 1, the datasheet shows "X" => still brake with EN=0
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, false);
    }
    if (config.verbose & !initial_config) { drvPrint("Braking Motor\n"); }
}

void DRV8214::coastMotor() {
    enableHbridge();
    if (config.control_mode == PWM) {
        // Table 8-5 => Coast => Input1=0, Input2=0 => High-Z while awake
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, false);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, false);
    }
    else {
        // PH/EN mode has no "coast" state in the datasheet table. There's no official high-Z while awake.
        // We could do "sleep" or "brake," or just do nothing here;
        drvPrint("PH/EN mode does not support coast (High-Z) while awake.");
    }
    if (config.verbose) { drvPrint("Coasting Motor\n"); }
}

void DRV8214::turnXRipples(uint16_t ripples_target, bool stops, bool direction, uint16_t speed, float voltage, float requested_current) {
    setRippleCountThreshold(ripples_target);
    resetRippleCounter();
    if (stops != config.bridge_behavior_thr_reached) { setBridgeBehaviorThresholdReached(stops); } // Set bridge behavior if different
    if (direction) { turnForward(speed, voltage, requested_current); } else { turnReverse(speed, voltage, requested_current); }
}

void DRV8214::turnXRevolutions(uint16_t revolutions_target, bool stops, bool direction, uint16_t speed, float voltage, float requested_current) {

    uint8_t ripples_target = revolutions_target * ripples_per_revolution * motor_reduction_ratio;
    turnXRipples(ripples_target, stops, direction, speed, voltage, requested_current);
}

void DRV8214::printMotorConfig(bool initial_config) {
    char buffer[256];  // Adjust the buffer size as needed
    
    if (initial_config) {
        // Using snprintf to safely format the string
        snprintf(buffer, sizeof(buffer), "----- Finished initialized driver %d -----\n", driver_ID);
        drvPrint(buffer);
    } else {
        snprintf(buffer, sizeof(buffer), "DRV8214 Driver %d", driver_ID);
        drvPrint(buffer);
    }
    snprintf(buffer, sizeof(buffer),
        "Address: 0x%02X | Sense Resistor: %d Ohms | Ripples per Rotor Revolution: %d | Ripples per Shaft Revolution: %d\n",
        address, Ripropri, ripples_per_revolution, ripples_per_revolution * motor_reduction_ratio);
    drvPrint(buffer);
    
    snprintf(buffer, sizeof(buffer), "Configuration: OVP: %s | STALL detect: %s | I2C controlled: %s | Mode: %s",
        config.voltage_range ? "Enabled" : "Disabled",
        config.stall_enabled ? "Enabled" : "Disabled",
        config.I2CControlled ? "Yes" : "No",
        (config.control_mode == PWM) ? "PWM" : "PH_EN");
    drvPrint(buffer);
    
    // Regulation mode details
    drvPrint(" | Regulation: ");
    switch (config.regulation_mode) {
        case CURRENT_FIXED:   drvPrint("CURRENT_FIXED\n"); break;
        case CURRENT_CYCLES:  drvPrint("CURRENT_CYCLES\n"); break;
        case SPEED:           drvPrint("SPEED\n"); break;
        case VOLTAGE:         drvPrint("VOLTAGE\n"); break;
    }
    
    snprintf(buffer, sizeof(buffer),
        "Vref: %.3f | Current Reg. Mode: %d | VRange: %s \n",
            config.Vref, config.current_reg_mode,
            config.voltage_range ? "0V-3.92V" : "0V-15.7V");
    drvPrint(buffer);

    snprintf(buffer, sizeof(buffer),
        "Stall Behavior: %s | Bridge Behavior Thr. reached: %s\n",
        config.stall_behavior ? "Drive current" : "Disable outputs",
        config.bridge_behavior_thr_reached ? "H-bridge disabled" : "H-bridge stays enabled");
    drvPrint(buffer);

    snprintf(buffer, sizeof(buffer),
        "Inrush Duration: %d ms | INV_R: %d | INV_R_SCALE: %d\n",
        config.inrush_duration, config.inv_r, config.inv_r_scale);
    drvPrint(buffer);

    snprintf(buffer, sizeof(buffer),
        "KMC: %d | KMCScale: %d\n",
        config.kmc, config.kmc_scale);
    drvPrint(buffer);
}

void DRV8214::drvPrint(const char* msg) {
    #ifdef DRV8214_PLATFORM_ARDUINO
        if (_debugPort) {
            _debugPort->print(msg);
        }
    #elif defined(DRV8214_PLATFORM_STM32)
        // Option 1: Using HAL_UART_Transmit directly
        // HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
        // Option 2: If you have retargeted printf to UART, you could simply use:
        printf("%s", msg);
    #endif
}

void DRV8214::printFaultStatus() {
    char buffer[256];  // Buffer for formatted output
    uint8_t faultReg = drv8214_i2c_read_register(address, DRV8214_FAULT);

    snprintf(buffer, sizeof(buffer), "DRV8214 Driver %d - FAULT Register Status:\n", driver_ID);
    drvPrint(buffer);

    if (faultReg & (1 << 7)) {
        drvPrint(" - FAULT: General fault detected.\n");
    } else {
        drvPrint(" - FAULT: No faults detected.\n");
    }

    if (faultReg & (1 << 5)) {
        drvPrint(" - STALL: Motor stall detected.\n");
    }

    if (faultReg & (1 << 4)) {
        drvPrint(" - OCP: Overcurrent protection (OCP) event occurred.\n");
    }

    if (faultReg & (1 << 3)) {
        drvPrint(" - OVP: Overvoltage protection (OVP) event occurred.\n");
    }

    if (faultReg & (1 << 2)) {
        drvPrint(" - TSD: Thermal shutdown (TSD) event occurred.\n");
    }

    if (faultReg & (1 << 1)) {
        drvPrint(" - NPOR: Device is in power-on reset (NPOR).\n");
    }

    if (faultReg & (1 << 0)) {
        drvPrint(" - CNT_DONE: Ripple counting threshold exceeded.\n");
    }
}

#ifdef DRV8214_PLATFORM_ARDUINO
    void DRV8214::setDebugStream(Stream* debugPort) {
        _debugPort = debugPort;
    }
#endif
