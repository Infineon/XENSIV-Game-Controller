// std includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// project c includes
#include "Logger.h"

// common to all sensors
#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// common to same generation of sensors
#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"

// sensor specific includes
#include "TLx493D_W2BW_defines.h"
#include "TLx493D_W2BW_enums.h"
#include "TLx493D_W2BW.h"


TLx493D_Register_t TLx493D_W2BW_regDef[] = {
    { /* W2BW_BX_MSBS_e, */    TLx493D_READ_MODE_e,         0x00,   0xFF,   0,  8 },
    { /* W2BW_BY_MSBS_e, */    TLx493D_READ_MODE_e,         0x01,   0xFF,   0,  8 },
    { /* W2BW_BZ_MSBS_e, */    TLx493D_READ_MODE_e,         0x02,   0xFF,   0,  8 },
    { /* W2BW_TEMP_MSBS_e, */  TLx493D_READ_MODE_e,         0x03,   0xFF,   0,  8 },
    { /* W2BW_BX_LSBS_e, */    TLx493D_READ_MODE_e,         0x04,   0xF0,   4,  4 },
    { /* W2BW_BY_LSBS_e, */    TLx493D_READ_MODE_e,         0x04,   0x0F,   0,  4 },
    { /* W2BW_TEMP_LSBS_e, */  TLx493D_READ_MODE_e,         0x05,   0xC0,   6,  2 },
    { /* W2BW_ID_e, */         TLx493D_READ_MODE_e,         0x05,   0x30,   4,  2 },
    { /* W2BW_BZ_LSBS_e, */    TLx493D_READ_MODE_e,         0x05,   0x0F,   0,  4 },
    { /* W2BW_P_e, */          TLx493D_READ_MODE_e,         0x06,   0x80,   7,  1 },
    { /* W2BW_FF_e, */         TLx493D_READ_MODE_e,         0x06,   0x40,   6,  1 },
    { /* W2BW_CF_e, */         TLx493D_READ_MODE_e,         0x06,   0x20,   5,  1 },
    { /* W2BW_T_e, */          TLx493D_READ_MODE_e,         0x06,   0x10,   4,  1 },
    { /* W2BW_PD3_e, */        TLx493D_READ_MODE_e,         0x06,   0x08,   3,  1 },
    { /* W2BW_PD0_e, */        TLx493D_READ_MODE_e,         0x06,   0x04,   2,  1 },
    { /* W2BW_FRM_e, */        TLx493D_READ_MODE_e,         0x06,   0x03,   0,  2 },
    { /* W2BW_XL_MSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x07,   0xFF,   0,  8 },
    { /* W2BW_XH_MSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x08,   0xFF,   0,  8 },
    { /* W2BW_YL_MSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x09,   0xFF,   0,  8 },
    { /* W2BW_YH_MSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0A,   0xFF,   0,  8 },
    { /* W2BW_ZL_MSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0B,   0xFF,   0,  8 },
    { /* W2BW_ZH_MSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0C,   0xFF,   0,  8 },
    { /* W2BW_WA_e, */         TLx493D_READ_MODE_e,         0x0D,   0x80,   7,  1 },
    { /* W2BW_WU_e, */         TLx493D_READ_WRITE_MODE_e,   0x0D,   0x40,   6,  1 },
    { /* W2BW_XH_LSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0D,   0x38,   3,  3 },
    { /* W2BW_XL_LSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0D,   0x07,   0,  3 },
    { /* W2BW_YH_LSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0E,   0x38,   3,  3 },
    { /* W2BW_YL_LSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0E,   0x07,   0,  3 },
    { /* W2BW_ZH_LSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0F,   0x38,   3,  3 },
    { /* W2BW_ZL_LSBS_e, */    TLx493D_READ_WRITE_MODE_e,   0x0F,   0x07,   0,  3 },
    { /* W2BW_DT_e, */         TLx493D_READ_WRITE_MODE_e,   0x10,   0x80,   7,  1 },
    { /* W2BW_AM_e, */         TLx493D_READ_WRITE_MODE_e,   0x10,   0x40,   6,  1 },
    { /* W2BW_TRIG_e, */       TLx493D_READ_WRITE_MODE_e,   0x10,   0x30,   4,  2 },
    { /* W2BW_X2_e, */         TLx493D_READ_WRITE_MODE_e,   0x10,   0x08,   3,  1 },
    { /* W2BW_TL_MAG_e, */     TLx493D_READ_WRITE_MODE_e,   0x10,   0x06,   1,  2 },
    { /* W2BW_CP_e, */         TLx493D_READ_WRITE_MODE_e,   0x10,   0x01,   0,  1 },
    { /* W2BW_FP_e, */         TLx493D_READ_WRITE_MODE_e,   0x11,   0x80,   7,  1 },
    { /* W2BW_IICADR_e, */     TLx493D_READ_WRITE_MODE_e,   0x11,   0x60,   5,  2 },
    { /* W2BW_PR_e, */         TLx493D_READ_WRITE_MODE_e,   0x11,   0x10,   4,  1 },
    { /* W2BW_CA_e, */         TLx493D_READ_WRITE_MODE_e,   0x11,   0x08,   3,  1 },
    { /* W2BW_INT_e, */        TLx493D_READ_WRITE_MODE_e,   0x11,   0x04,   2,  1 },
    { /* W2BW_MODE_e, */       TLx493D_READ_WRITE_MODE_e,   0x11,   0x03,   0,  2 },
    { /* W2BW_PRD_e, */        TLx493D_READ_WRITE_MODE_e,   0x13,   0xE0,   5,  3 },
    // { /* W2BW_X4_e, */         TLx493D_READ_WRITE_MODE_e,        0x14,   0x01,   0,  1 },
    { /* W2BW_X4_e, */         TLx493D_WRITE_MODE_e,        0x14,   0x01,   0,  1 },
    { /* W2BW_TYPE_e, */       TLx493D_READ_MODE_e,         0x16,   0x30,   4,  2 },
    { /* W2BW_HWV_e, */        TLx493D_READ_MODE_e,         0x16,   0x0F,   0,  4 },
};


TLx493D_CommonFunctions_t TLx493D_W2BW_commonFunctions = {
    .init                           = TLx493D_W2BW_init,
    .deinit                         = TLx493D_W2BW_deinit,

    .readRegisters                  = TLx493D_W2BW_readRegisters,

    .calculateRawTemperature        = TLx493D_W2BW_calculateRawTemperature,
    .getRawTemperature              = TLx493D_W2BW_getRawTemperature,

    .calculateRawMagneticField      = TLx493D_W2BW_calculateRawMagneticField,
    .getRawMagneticField            = TLx493D_W2BW_getRawMagneticField,

    .calculateRawMagneticFieldAndTemperature = TLx493D_W2BW_calculateRawMagneticFieldAndTemperature,
    .getRawMagneticFieldAndTemperature       = TLx493D_W2BW_getRawMagneticFieldAndTemperature,

    .calculateTemperature           = TLx493D_W2BW_calculateTemperature,
    .getTemperature                 = TLx493D_W2BW_getTemperature,
    
    .calculateMagneticField         = TLx493D_W2BW_calculateMagneticField,
    .getMagneticField               = TLx493D_W2BW_getMagneticField,
    
    .calculateMagneticFieldAndTemperature = TLx493D_W2BW_calculateMagneticFieldAndTemperature,
    .getMagneticFieldAndTemperature = TLx493D_W2BW_getMagneticFieldAndTemperature,

    // functions related to the "Config" register
    .setMeasurement                 = TLx493D_W2BW_setMeasurement,
    .setTrigger                     = TLx493D_W2BW_setTrigger,
    .setSensitivity                 = TLx493D_W2BW_setSensitivity,

    // functions related to the "Mod1" and "Mod2" registers
    .setDefaultConfig               = TLx493D_W2BW_setDefaultConfig,
    .setIICAddress                  = TLx493D_W2BW_setIICAddress,
    .enable1ByteReadMode            = TLx493D_W2BW_enable1ByteReadMode,

    .enableInterrupt                = TLx493D_W2BW_enableInterrupt,
    .disableInterrupt               = TLx493D_W2BW_disableInterrupt,

    .enableCollisionAvoidance       = TLx493D_W2BW_enableCollisionAvoidance,
    .disableCollisionAvoidance      = TLx493D_W2BW_disableCollisionAvoidance,

    .setPowerMode                   = TLx493D_W2BW_setPowerMode,
    .setUpdateRate                  = TLx493D_W2BW_setUpdateRate,

    // functions related to the "Diag" register
    .hasValidData                   = TLx493D_W2BW_hasValidData,
    .isFunctional                   = TLx493D_W2BW_isFunctional,

    // functions available only to a subset of sensors with wake-up functionality
    // functions related to the "WU" register
    .hasWakeUp                      = TLx493D_W2BW_hasWakeUp,
    .isWakeUpEnabled                = TLx493D_W2BW_isWakeUpEnabled,
    .enableWakeUpMode               = TLx493D_W2BW_enableWakeUpMode,
    .disableWakeUpMode              = TLx493D_W2BW_disableWakeUpMode,

    .setWakeUpThresholdsAsInteger   = TLx493D_W2BW_setWakeUpThresholdsAsInteger,
    .setWakeUpThresholds            = TLx493D_W2BW_setWakeUpThresholds,

    .softwareReset                  = TLx493D_W2BW_softwareReset,

    // functions used internally and not accessible through the common interface
    .calculateFuseParity            = TLx493D_W2BW_calculateFuseParity,
    .calculateBusParity             = TLx493D_W2BW_calculateBusParity,
    .calculateConfigurationParity   = TLx493D_W2BW_calculateConfigurationParity,
    // .calculateConfigurationParity   = TLx493D_W2BW_calculateConfigurationParityBit,

    .hasValidFuseParity             = TLx493D_W2BW_hasValidFuseParity,
    .hasValidBusParity              = TLx493D_W2BW_hasValidBusParity,
    .hasValidConfigurationParity    = TLx493D_W2BW_hasValidConfigurationParity,
 
    .hasValidWakeUpParity           = TLx493D_W2BW_hasValidWakeUpParity,
    .isInTestMode                   = TLx493D_W2BW_isInTestMode,
    
    .hasValidTBit                   = TLx493D_W2BW_hasValidTBit,
    
    .setResetValues                 = TLx493D_W2BW_setResetValues,

    .selectIICAddress               = TLx493D_W2BW_selectIICAddress,

    .calculateRawMagneticFieldAtTemperature = TLx493D_W2BW_calculateRawMagneticFieldAtTemperature,

    .getSensitivityScaleFactor      = TLx493D_W2BW_getSensitivityScaleFactor,
};

bool TLx493D_W2BW_init(TLx493D_t *sensor) {
    return tlx493d_common_init(sensor, GEN_2_REG_MAP_SIZE, TLx493D_W2BW_regDef, &TLx493D_W2BW_commonFunctions, TLx493D_W2BW_e, TLx493D_I2C_e);
}

bool TLx493D_W2BW_deinit(TLx493D_t *sensor) {
    return tlx493d_common_deinit(sensor);
}


bool TLx493D_W2BW_readRegisters(TLx493D_t *sensor) {
    return tlx493d_common_readRegisters(sensor);
}


void TLx493D_W2BW_calculateRawTemperature(TLx493D_t *sensor, int16_t *temperature) {
    tlx493d_gen_2_calculateRawTemperature(sensor, W2BW_TEMP_MSBS_e, W2BW_TEMP_LSBS_e, temperature);
}


bool TLx493D_W2BW_getRawTemperature(TLx493D_t *sensor, int16_t *temperature) {
    return tlx493d_common_getRawTemperature(sensor, temperature);
}


void TLx493D_W2BW_calculateRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z) {
    tlx493d_gen_2_calculateRawMagneticField(sensor, W2BW_BX_MSBS_e, W2BW_BX_LSBS_e, W2BW_BY_MSBS_e, W2BW_BY_LSBS_e, W2BW_BZ_MSBS_e, W2BW_BZ_LSBS_e, x, y, z);
}


bool TLx493D_W2BW_getRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z) {
    return tlx493d_common_getRawMagneticField(sensor, x, y, z);
}


void TLx493D_W2BW_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z, int16_t *temperature) {
    TLx493D_W2BW_calculateRawMagneticField(sensor, x, y, z);
    TLx493D_W2BW_calculateRawTemperature(sensor, temperature);
}


bool TLx493D_W2BW_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z, int16_t *temperature) {
    return tlx493d_common_getRawMagneticFieldAndTemperature(sensor, x, y, z, temperature);
}


void TLx493D_W2BW_calculateTemperature(TLx493D_t *sensor, double *temp) {
    tlx493d_gen_2_calculateTemperature(sensor, W2BW_TEMP_MSBS_e, W2BW_TEMP_LSBS_e, temp);
}

bool TLx493D_W2BW_getTemperature(TLx493D_t *sensor, double *temp) {
    return tlx493d_common_getTemperature(sensor, temp);
}

void TLx493D_W2BW_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    tlx493d_gen_2_calculateMagneticField(sensor, W2BW_BX_MSBS_e, W2BW_BX_LSBS_e, W2BW_BY_MSBS_e, W2BW_BY_LSBS_e, W2BW_BZ_MSBS_e, W2BW_BZ_LSBS_e, x, y, z);
}

bool TLx493D_W2BW_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z) {
    return tlx493d_common_getMagneticField(sensor, x, y, z);
}


void TLx493D_W2BW_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    TLx493D_W2BW_calculateMagneticField(sensor, x, y, z);
    TLx493D_W2BW_calculateTemperature(sensor, temp);
}


bool TLx493D_W2BW_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp) {
    return tlx493d_common_getMagneticFieldAndTemperature(sensor, x, y, z, temp);
}


bool TLx493D_W2BW_setMeasurement(TLx493D_t *sensor, TLx493D_MeasurementType_t val) {
    return tlx493d_gen_2_setMeasurement(sensor, W2BW_DT_e, W2BW_AM_e, W2BW_CP_e, val);
}


bool TLx493D_W2BW_setTrigger(TLx493D_t *sensor, TLx493D_TriggerType_t trigger) {
    return tlx493d_gen_2_setTrigger(sensor, W2BW_TRIG_e, W2BW_CP_e, trigger);
}


bool TLx493D_W2BW_setSensitivity(TLx493D_t *sensor, TLx493D_SensitivityType_t val) {
    return tlx493d_gen_2_setSensitivity(sensor, TLx493D_HAS_X4_e, W2BW_X2_e, W2BW_X4_e, W2BW_CP_e, val);
}


bool TLx493D_W2BW_setDefaultConfig(TLx493D_t *sensor) {
    return tlx493d_gen_2_setDefaultConfig(sensor, W2BW_CONFIG_REG_e, W2BW_MOD1_REG_e, W2BW_MOD2_REG_e, W2BW_CP_e, W2BW_CA_e, W2BW_INT_e);
}


bool TLx493D_W2BW_setIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t address) {
    return tlx493d_gen_2_setIICAddress(sensor, W2BW_IICADR_e, W2BW_FP_e, address);
}


bool TLx493D_W2BW_enable1ByteReadMode(TLx493D_t *sensor) {
    return tlx493d_gen_2_set1ByteReadMode(sensor, W2BW_PR_e, W2BW_FP_e, W2BW_PRD_e, 1);
}


bool TLx493D_W2BW_enableCollisionAvoidance(TLx493D_t *sensor) {
    return tlx493d_gen_2_setCollisionAvoidance(sensor, W2BW_CA_e, W2BW_FP_e, W2BW_PRD_e, 0);
}


bool TLx493D_W2BW_disableCollisionAvoidance(TLx493D_t *sensor) {
    return tlx493d_gen_2_setCollisionAvoidance(sensor, W2BW_CA_e, W2BW_FP_e, W2BW_PRD_e, 1);
}


bool TLx493D_W2BW_enableInterrupt(TLx493D_t *sensor) {
    return tlx493d_gen_2_setInterrupt(sensor, W2BW_INT_e, W2BW_FP_e, W2BW_PRD_e, 0);
}


bool TLx493D_W2BW_disableInterrupt(TLx493D_t *sensor) {
    return tlx493d_gen_2_setInterrupt(sensor, W2BW_INT_e, W2BW_FP_e, W2BW_PRD_e, 1);
}


bool TLx493D_W2BW_setPowerMode(TLx493D_t *sensor, uint8_t mode) {
    return tlx493d_gen_2_setPowerMode(sensor, W2BW_MODE_e, W2BW_FP_e, mode);
}


bool TLx493D_W2BW_setUpdateRate(TLx493D_t *sensor, TLx493D_UpdateRateType_t val) {
    return tlx493d_gen_2_setUpdateRate(sensor, W2BW_FP_e, W2BW_PRD_e, val);
}


bool TLx493D_W2BW_hasValidData(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidData(sensor);
}


bool TLx493D_W2BW_isFunctional(TLx493D_t *sensor) {
    return tlx493d_gen_2_isFunctional(sensor);
}


bool TLx493D_W2BW_hasWakeUp(TLx493D_t *sensor) {
    return true;
}


bool TLx493D_W2BW_isWakeUpEnabled(TLx493D_t *sensor) {
    return tlx493d_gen_2_isWakeUpEnabled(sensor, W2BW_WA_e);
}


bool TLx493D_W2BW_enableWakeUpMode(TLx493D_t *sensor) {
    tlx493d_common_setBitfield(sensor, W2BW_WU_e, 1);
    tlx493d_common_setBitfield(sensor, W2BW_CP_e, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_gen_2_writeConfigurationRegisters(sensor) ? sensor->functions->readRegisters(sensor) : false;
}


bool TLx493D_W2BW_disableWakeUpMode(TLx493D_t *sensor) {
    return tlx493d_gen_2_disableWakeUpMode(sensor, W2BW_WU_e, W2BW_CP_e);
}


bool TLx493D_W2BW_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xlTh, int16_t xhTh, int16_t ylTh, int16_t yhTh, int16_t zlTh, int16_t zhTh) {
    return tlx493d_gen_2_setWakeUpThresholdsAsInteger(sensor,
                                                      W2BW_XL_MSBS_e, W2BW_XL_LSBS_e, W2BW_XH_MSBS_e, W2BW_XH_LSBS_e,
                                                      W2BW_YL_MSBS_e, W2BW_YL_LSBS_e, W2BW_YH_MSBS_e, W2BW_YH_LSBS_e,
                                                      W2BW_ZL_MSBS_e, W2BW_ZL_LSBS_e, W2BW_ZH_MSBS_e, W2BW_ZH_LSBS_e,
                                                      xlTh, xhTh, ylTh, yhTh, zlTh, zhTh);
}


// thesholds im mT, to be converted to proper format
bool TLx493D_W2BW_setWakeUpThresholds(TLx493D_t *sensor,
                                      double temperature, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
    return tlx493d_gen_2_setWakeUpThresholds(sensor,
                                             W2BW_XL_MSBS_e, W2BW_XL_LSBS_e, W2BW_XH_MSBS_e, W2BW_XH_LSBS_e,
                                             W2BW_YL_MSBS_e, W2BW_YL_LSBS_e, W2BW_YH_MSBS_e, W2BW_YH_LSBS_e,
                                             W2BW_ZL_MSBS_e, W2BW_ZL_LSBS_e, W2BW_ZH_MSBS_e, W2BW_ZH_LSBS_e,
                                             TLx493D_HAS_X4_e, W2BW_X2_e, W2BW_X4_e,
                                             temperature, xLow, xHigh, yLow, yHigh, zLow, zHigh);
}


bool TLx493D_W2BW_softwareReset(TLx493D_t *sensor) {
    tlx493d_warnFeatureNotAvailableForSensorType(sensor, "softwareReset");
    return false;
}


uint8_t TLx493D_W2BW_calculateFuseParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_calculateFuseParity(sensor, W2BW_FP_e, W2BW_PRD_e);
}


uint8_t TLx493D_W2BW_calculateBusParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_calculateBusParity(sensor, 5);
}


uint8_t TLx493D_W2BW_calculateConfigurationParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_calculateConfigurationParityWakeUp(sensor, W2BW_CP_e);
}


bool TLx493D_W2BW_hasValidFuseParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidFuseParity(sensor, W2BW_FF_e);
}


bool TLx493D_W2BW_hasValidBusParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidBusParity(sensor, W2BW_P_e);
}


bool TLx493D_W2BW_hasValidConfigurationParity(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidConfigurationParity(sensor, W2BW_CF_e);
}


bool TLx493D_W2BW_hasValidWakeUpParity(TLx493D_t *sensor) {
    tlx493d_warnFeatureNotAvailableForSensorType(sensor, "hasValidWakeUpParity");
    return false;
}


bool TLx493D_W2BW_isInTestMode(TLx493D_t *sensor) {
    tlx493d_warnFeatureNotAvailableForSensorType(sensor, "isInTestMode");
    return false;
}


bool TLx493D_W2BW_hasValidIICadr(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidIICadr(sensor, W2BW_ID_e, W2BW_IICADR_e);
}


bool TLx493D_W2BW_hasValidTBit(TLx493D_t *sensor) {
    return tlx493d_gen_2_hasValidTBit(sensor, W2BW_T_e);
}


void TLx493D_W2BW_setResetValues(TLx493D_t *sensor) {
    sensor->regMap[0x07] = 0x80;
    sensor->regMap[0x08] = 0x7F;
    sensor->regMap[0x09] = 0x80;
    sensor->regMap[0x0A] = 0x7F;
    sensor->regMap[0x0B] = 0x80;
    sensor->regMap[0x0C] = 0x7F;
    sensor->regMap[0x0D] = 0x38;
    sensor->regMap[0x0E] = 0x38;
    sensor->regMap[0x0F] = 0x38;  
    sensor->regMap[0x10] = 0x01; // CONFIG
    sensor->regMap[0x11] = 0x80; // MOD1 : A0 : 0x80, A1 : 0x20, A2 : 0x40, A3 : 0xE0
    sensor->regMap[0x13] = 0x00; // MOD2
    sensor->regMap[0x14] = 0x00; // CONFIG2
}


uint8_t TLx493D_W2BW_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr) {
    return tlx493d_gen_2_selectIICAddress(sensor, addr);
}


void TLx493D_W2BW_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens,
                                                         double xInmT, double yInmT, double zInmT,
                                                         int16_t *x, int16_t *y, int16_t *z) {
    tlx493d_gen_2_calculateRawMagneticFieldAtTemperature(sensor, rawTemp, sens, xInmT, yInmT, zInmT, x, y, z);
}


double TLx493D_W2BW_getSensitivityScaleFactor(TLx493D_t *sensor) {
    return tlx493d_gen_2_getSensitivityScaleFactor(sensor, TLx493D_HAS_X4_e, W2BW_X2_e, W2BW_X4_e);
}
