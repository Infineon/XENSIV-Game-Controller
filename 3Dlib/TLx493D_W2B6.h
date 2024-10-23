#ifndef TLX493D_W2B6_H
#define TLX493D_W2B6_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// common to same generation of sensors

// sensor specific includes


#ifdef __cplusplus

extern "C" {

#endif


bool TLx493D_W2B6_init(TLx493D_t *sensor);
bool TLx493D_W2B6_deinit(TLx493D_t *sensor);

bool TLx493D_W2B6_readRegisters(TLx493D_t *sensor);

void TLx493D_W2B6_calculateRawTemperature(TLx493D_t *sensor, int16_t *temperature);
bool TLx493D_W2B6_getRawTemperature(TLx493D_t *sensor, int16_t *temperature);

void TLx493D_W2B6_calculateRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z);
bool TLx493D_W2B6_getRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z);

void TLx493D_W2B6_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z, int16_t *temperature);
bool TLx493D_W2B6_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z, int16_t *temperature);

void TLx493D_W2B6_calculateTemperature(TLx493D_t *sensor, double *temp);
bool TLx493D_W2B6_getTemperature(TLx493D_t *sensor, double *temp);

void TLx493D_W2B6_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);
bool TLx493D_W2B6_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);

void TLx493D_W2B6_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_W2B6_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);

bool TLx493D_W2B6_setMeasurement(TLx493D_t *sensor, TLx493D_MeasurementType_t mVals);
bool TLx493D_W2B6_setTrigger(TLx493D_t *sensor, TLx493D_TriggerType_t trig);
bool TLx493D_W2B6_setSensitivity(TLx493D_t *sensor, TLx493D_SensitivityType_t sens);

bool TLx493D_W2B6_setDefaultConfig(TLx493D_t *sensor);
bool TLx493D_W2B6_setIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t address);
bool TLx493D_W2B6_enable1ByteReadMode(TLx493D_t *sensor);

bool TLx493D_W2B6_enableCollisionAvoidance(TLx493D_t *sensor);
bool TLx493D_W2B6_disableCollisionAvoidance(TLx493D_t *sensor);

bool TLx493D_W2B6_enableInterrupt(TLx493D_t *sensor);
bool TLx493D_W2B6_disableInterrupt(TLx493D_t *sensor);

bool TLx493D_W2B6_setPowerMode(TLx493D_t *sensor, TLx493D_PowerModeType_t mode);
bool TLx493D_W2B6_setUpdateRate(TLx493D_t *sensor, TLx493D_UpdateRateType_t rate);

bool TLx493D_W2B6_hasValidData(TLx493D_t *sensor);
bool TLx493D_W2B6_isFunctional(TLx493D_t *sensor);

bool TLx493D_W2B6_hasWakeUp(TLx493D_t *sensor);
bool TLx493D_W2B6_isWakeUpEnabled(TLx493D_t *sensor);
bool TLx493D_W2B6_enableWakeUpMode(TLx493D_t *sensor);
bool TLx493D_W2B6_disableWakeUpMode(TLx493D_t *sensor);
bool TLx493D_W2B6_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xlTh, int16_t xhTh, int16_t ylTh, int16_t yhTh, int16_t zlTh, int16_t zhTh);
bool TLx493D_W2B6_setWakeUpThresholds(TLx493D_t *sensor, double temperature, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

bool TLx493D_W2B6_softwareReset(TLx493D_t *sensor);


// utilities
uint8_t TLx493D_W2B6_calculateFuseParity(TLx493D_t *sensor);
uint8_t TLx493D_W2B6_calculateBusParity(TLx493D_t *sensor);
uint8_t TLx493D_W2B6_calculateConfigurationParity(TLx493D_t *sensor);

bool TLx493D_W2B6_hasValidFuseParity(TLx493D_t *sensor);
bool TLx493D_W2B6_hasValidBusParity(TLx493D_t *sensor);
bool TLx493D_W2B6_hasValidConfigurationParity(TLx493D_t *sensor);

bool TLx493D_W2B6_hasValidWakeUpParity(TLx493D_t *sensor);
bool TLx493D_W2B6_isInTestMode(TLx493D_t *sensor);

bool TLx493D_W2B6_hasValidTBit(TLx493D_t *sensor);
bool TLx493D_W2B6_hasValidIICadr(TLx493D_t *sensor);

void TLx493D_W2B6_setResetValues(TLx493D_t *sensor);

uint8_t TLx493D_W2B6_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr);

void TLx493D_W2B6_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens,
                                                         double xInmT, double yInmT, double zInmT,
                                                         int16_t *x, int16_t *y, int16_t *z);

double TLx493D_W2B6_getSensitivityScaleFactor(TLx493D_t *sensor);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_W2B6_H
