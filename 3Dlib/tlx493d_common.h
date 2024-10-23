#ifndef TLX493D_COMMON_H
#define TLX493D_COMMON_H

// project c includes
// common to all sensors
#include "tlx493d_types.h"


#ifdef __cplusplus

extern "C" {

#endif


// /**
//  * @brief Initializes the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It initializes the sensor structure and sets the I2C communication protocol
//  *
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
bool tlx493d_common_init(TLx493D_t *sensor, uint8_t regMapSize, TLx493D_Register_t *regDef, TLx493D_CommonFunctions_t *commonFuncs, TLx493D_SupportedSensorType_t sensorType, TLx493D_SupportedComLibraryInterfaceType_t comIFType);


// /**
//  * @brief De-Initializes the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It frees the corresponding pointers of the sensor structure and sets the pointers to NULL
//  *
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
bool tlx493d_common_deinit(TLx493D_t *sensor);


void    tlx493d_common_getBitfield(TLx493D_t *sensor, uint8_t bitField, uint8_t *bitFieldValue);
uint8_t tlx493d_common_returnBitfield(TLx493D_t *sensor, uint8_t bitField);
void    tlx493d_common_setBitfield(TLx493D_t *sensor, uint8_t bitField, uint8_t newBitFieldValue);

bool tlx493d_common_writeRegister(TLx493D_t* sensor, uint8_t bitField);
bool tlx493d_common_readRegisters(TLx493D_t *sensor);


void tlx493d_common_calculateRawTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, int16_t *temperature);
bool tlx493d_common_getRawTemperature(TLx493D_t *sensor, int16_t *temperature);

void tlx493d_common_calculateRawMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                              uint8_t bzMSBBF, uint8_t bzLSBBF, int16_t *x, int16_t *y, int16_t *z);
bool tlx493d_common_getRawMagneticField(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z);

bool tlx493d_common_getRawMagneticFieldAndTemperature(TLx493D_t *sensor, int16_t *x, int16_t *y, int16_t *z, int16_t *temperature);


// /**
//  * @brief Retrieves the temperature value of the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It reads out the required registers to calculate the temperature value.
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @param[in, out] temp Retrieved temperature value of the sensor
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
bool tlx493d_common_getTemperature(TLx493D_t *sensor, double *temperature);


// /**
//  * @brief Updates the required registers to read the magnetic field values of the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It updates the required registers and then calls the calculateFieldValues() function of the sensor
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @param[in, out] x Retrieved magnetic value of the X-Axis
//  * @param[in, out] y Retrieved magnetic value of the Y-Axis
//  * @param[in, out] z Retrieved magnetic value of the Z-Axis
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
bool tlx493d_common_getMagneticField(TLx493D_t *sensor, double *x, double *y, double *z);


// /**
//  * @brief Updates the required registers to read the magnetic field values of the XENSIV™ TLx493D-W2B6 magnetic 3D sensor
//  * It updates the required registers and then calls the calculateFieldValues() function of the sensor
//  * 
//  * @param[in] sensor Pointer to the XENSIV™ TLx493D-W2B6 magnetic 3D sensor structure
//  * @param[in, out] x Retrieved magnetic value of the X-Axis
//  * @param[in, out] y Retrieved magnetic value of the Y-Axis
//  * @param[in, out] z Retrieved magnetic value of the Z-Axis
//  * @return true - If successful
//  * @return false - If unsuccessful
//  */
bool tlx493d_common_getMagneticFieldAndTemperature(TLx493D_t *sensor, double *x, double *y, double *z, double *temperature);


// utilities
uint8_t tlx493d_common_calculateParity(uint8_t data);
uint8_t tlx493d_common_getOddParity(uint8_t parity);
uint8_t tlx493d_common_getEvenParity(uint8_t parity);

void tlx493d_common_concatBytes(TLx493D_t *sensor, uint8_t msbBitfield, uint8_t lsbBitfield, int16_t *result);

const char *tlx493d_common_getTypeAsString(TLx493D_t *sensor);

void tlx493d_common_setIICAddress(TLx493D_t *sensor, uint8_t addr);

// void tlx493d_common_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF);


void tlx493d_warnFeatureNotAvailableForSensorType(TLx493D_t *sensor, const char *featureName);
void tlx493d_errorBitfieldNotReadableForSensorType(TLx493D_t *sensor, uint8_t field);
void tlx493d_errorBitfieldNotWritableForSensorType(TLx493D_t *sensor, uint8_t field);
void tlx493d_errorFunctionNotSupportedForSensorType(TLx493D_t *sensor, const char *func);
void tlx493d_errorSelectionNotSupportedForSensorType(TLx493D_t *sensor, uint8_t sel, const char *selType);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_COMMON_H
