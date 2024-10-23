/**
 * @file TLx493D_W2BW.h
 * @author Infineon Technologies AG
 * @brief Definiton of the complete sensor functionality
 * @copyright Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TLX493D_W2BW_H_
#define TLX493D_W2BW_H_


/** std includes*/
#include <stdbool.h>
#include <stdint.h>

/** Common to all sensors */
#include "tlx493d_types.h"

/** Common to the same generation of senors */

/** Sensor specific includes */


#ifdef __cplusplus

extern "C" {

#endif


/**
 * @brief Initializes the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It initializes the sensor structure and sets the I2C communication protocol
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_init(TLx493D_t *sensor);

/**
 * @brief De-Initializes the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
 *
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_deinit(TLx493D_t *sensor);

bool TLx493D_W2BW_readRegisters(TLx493D_t *sensor);

void TLx493D_W2BW_calculateRawTemperature(TLx493D_t *sensor, int16_t *temperature);
bool TLx493D_W2BW_getRawTemperature(TLx493D_t *sensor, int16_t *temperature);

void TLx493D_W2BW_calculateRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z);
bool TLx493D_W2BW_getRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z);

void TLx493D_W2BW_calculateRawMagneticFieldAndTemperature(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z, int16_t *temperature);
bool TLx493D_W2BW_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z, int16_t *temperature);

/**
 * @brief Updates the required registers to read the temperature value of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It updates the required registers and then calls the getTemperature() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLx493D_W2BW_calculateTemperature(TLx493D_t *sensor, double *temp);

/**
 * @brief Retrieves the temperature value of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It reads out the required registers to calculate the temperature value.
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] temp Retrieved temperature value of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_getTemperature(TLx493D_t *sensor, double *temp);

/**
 * @brief Updates the required registers to read the magnetic field values of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It updates the required registers and then calls the calculateFieldValues() function of the sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
void TLx493D_W2BW_calculateMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);

/**
 * @brief Retrieves the magnetic field values of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It reads out the required registers to calculate the magnetic field value for the X, Y, Z-Axis
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in, out] x Retrieved magnetic value of the X-Axis
 * @param[in, out] y Retrieved magnetic value of the Y-Axis
 * @param[in, out] z Retrieved magnetic value of the Z-Axis
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);


void TLx493D_W2BW_calculateMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);
bool TLx493D_W2BW_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temp);

bool TLx493D_W2BW_setMeasurement(TLx493D_t *sensor, TLx493D_MeasurementType_t mVals);
bool TLx493D_W2BW_setTrigger(TLx493D_t *sensor, TLx493D_TriggerType_t trig);
bool TLx493D_W2BW_setSensitivity(TLx493D_t *sensor, TLx493D_SensitivityType_t sens);


/**
 * @brief Sets the default configuration for the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * It sets the sensor into 1-Byte-Mode and also enables the temperature measurement
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful 
 */
bool TLx493D_W2BW_setDefaultConfig(TLx493D_t *sensor);
bool TLx493D_W2BW_setIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t address);

bool TLx493D_W2BW_enable1ByteReadMode(TLx493D_t *sensor);

bool TLx493D_W2BW_enableCollisionAvoidance(TLx493D_t *sensor);
bool TLx493D_W2BW_disableCollisionAvoidance(TLx493D_t *sensor);

/**
 * @brief Enables the interrupt after measurement completion of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_enableInterrupt(TLx493D_t *sensor);

/**
 * @brief Disables the interrupt after measurement completion of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLx493D_W2BW_disableInterrupt(TLx493D_t *sensor);


bool TLx493D_W2BW_setPowerMode(TLx493D_t *sensor, uint8_t mode);


/**
 * @brief Sets the update rate for the measurements of the XENSIV™ TLx493D-W2BW magnetic 3D sensor
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @param[in] bit Value which is set to the PRD-Bitfield
 * @return true - If successful
 * @return false - If unsuccessful 
 */
bool TLx493D_W2BW_setUpdateRate(TLx493D_t *sensor, TLx493D_UpdateRateType_t ur);


bool TLx493D_W2BW_hasValidData(TLx493D_t *sensor);
bool TLx493D_W2BW_isFunctional(TLx493D_t *sensor);

bool TLx493D_W2BW_hasWakeUp(TLx493D_t *sensor);

/**
 * @brief Checks if the wake up functionality is enabled
 * 
 * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2BW magnetic 3D sensor structure
 * @return true - If wake up is enabled
 * @return false - If wake up is disabled
 */
bool TLx493D_W2BW_isWakeUpEnabled(TLx493D_t *sensor);

bool TLx493D_W2BW_enableWakeUpMode(TLx493D_t *sensor);
bool TLx493D_W2BW_disableWakeUpMode(TLx493D_t *sensor);

bool TLx493D_W2BW_setWakeUpThresholdsAsInteger(TLx493D_t *sensor, int16_t xhTh, int16_t xlTh, int16_t yhTh, int16_t ylTh, int16_t zhTh, int16_t zlTh);
bool TLx493D_W2BW_setWakeUpThresholds(TLx493D_t *sensor, double temperature, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh);

bool TLx493D_W2BW_softwareReset(TLx493D_t *sensor);

uint8_t TLx493D_W2BW_calculateFuseParity(TLx493D_t *sensor);
uint8_t TLx493D_W2BW_calculateBusParity(TLx493D_t *sensor);
uint8_t TLx493D_W2BW_calculateConfigurationParity(TLx493D_t *sensor);

bool TLx493D_W2BW_hasValidFuseParity(TLx493D_t *sensor);
bool TLx493D_W2BW_hasValidBusParity(TLx493D_t *sensor);
bool TLx493D_W2BW_hasValidConfigurationParity(TLx493D_t *sensor);

bool TLx493D_W2BW_hasValidWakeUpParity(TLx493D_t *sensor);
bool TLx493D_W2BW_isInTestMode(TLx493D_t *sensor);

bool TLx493D_W2BW_hasValidTBit(TLx493D_t *sensor);
bool TLx493D_W2BW_hasValidIICadr(TLx493D_t *sensor);

void TLx493D_W2BW_setResetValues(TLx493D_t *sensor);

uint8_t TLx493D_W2BW_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr);

void TLx493D_W2BW_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens,
                                                         double xInmT, double yInmT, double zInmT,
                                                         int16_t *x, int16_t *y, int16_t *z);

double TLx493D_W2BW_getSensitivityScaleFactor(TLx493D_t *sensor);


#ifdef __cplusplus

}


#endif


#endif /** TLX493D_W2BW_H */