#ifndef TLX493D_TYPES_H
#define TLX493D_TYPES_H


// std includes
#include <stdbool.h>
#include <stdint.h>

// project c includes
#include "pal.h"


#ifdef __cplusplus

extern "C" {

#endif


// typedef struct TLx493D_t  TLx493D_t;


/*
  List of all supported sensor devices.
*/
typedef enum {
               TLx493D_A1B6_e = 0,
               TLx493D_A2B6_e,
               TLx493D_P2B6_e,
               TLx493D_W2B6_e,
               TLx493D_W2BW_e,
               TLx493D_P3B6_e,
               TLx493D_P3I8_e,
} TLx493D_SupportedSensorType_t;


/*
  List of all supported communication interfaces.
*/
typedef enum {
               TLx493D_I2C_e = 0,
               TLx493D_SPI_e,
              //  TLx493D_I2C_OR_SPI_e,
} TLx493D_SupportedComLibraryInterfaceType_t;


/*
  List of supported register access modes.
*/
typedef enum { TLx493D_READ_MODE_e = 0, 
               TLx493D_WRITE_MODE_e,
               TLx493D_READ_WRITE_MODE_e,
} TLx493D_RegisterAccessModeType_t;


typedef enum {
    TLx493D_IIC_ADDR_A0_e = 0,
    TLx493D_IIC_ADDR_A1_e,
    TLx493D_IIC_ADDR_A2_e,
    TLx493D_IIC_ADDR_A3_e,
    TLx493D_IIC_ADDR_A4_e,
    TLx493D_IIC_ADDR_A5_e,
    TLx493D_IIC_ADDR_A6_e,
    TLx493D_IIC_ADDR_A7_e,
} TLx493D_IICAddressType_t;


typedef enum {
    // 1st and 2nd generation
    TLx493D_BxByBz_e = 0,
    
    //  1st and 3rd generation
    TLx493D_BxByBzTemp_e,

    // 3rd generation
    TLx493D_VHall_Bias_e,
    TLx493D_Spintest_e,
    TLx493D_SAT_test_e,
    TLx493D_BxTemp_e,
    TLx493D_BxBy_e,
    TLx493D_BzTemp_e,
} TLx493D_MeasurementType_t;


typedef enum {
               TLx493D_NO_ADC_ON_READ_e = 0,
               TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e,
               TLx493D_ADC_ON_READ_AFTER_REG_05_e,
} TLx493D_TriggerType_t;


/*
  List of supported ranges.
*/
typedef enum {
               TLx493D_FULL_RANGE_e = 0, 
               TLx493D_SHORT_RANGE_e,
               TLx493D_EXTRA_SHORT_RANGE_e,
} TLx493D_SensitivityType_t;


/*
  List of supported ranges.
*/
typedef enum {
               TLx493D_HAS_X1_e = 0, 
               TLx493D_HAS_X2_e, 
               TLx493D_HAS_X4_e,
} TLx493D_AvailableSensitivityType_t;


typedef enum {
               TLx493D_LOW_POWER_MODE_e = 0,
               TLx493D_MASTER_CONTROLLED_MODE_e,
               TLx493D_FAST_MODE_e,
} TLx493D_PowerModeType_t;


typedef enum {
               // Generation 2
               TLx493D_UPDATE_RATE_FAST_e = 0,
               TLx493D_UPDATE_RATE_SLOW_e,
               TLx493D_UPDATE_RATE_770_HZ_e,
               TLx493D_UPDATE_RATE_97_HZ_e,
               TLx493D_UPDATE_RATE_24_HZ_e,
               TLx493D_UPDATE_RATE_12_HZ_e,
               TLx493D_UPDATE_RATE_6_HZ_e,
               TLx493D_UPDATE_RATE_3_HZ_e,
               TLx493D_UPDATE_RATE_0_4_HZ_e,
               TLx493D_UPDATE_RATE_0_05_HZ_e,

               // Generation 3
               TLx493D_UPDATE_RATE_1000_HZ_e,
               TLx493D_UPDATE_RATE_125_HZ_e,
               TLx493D_UPDATE_RATE_31_HZ_e,
               TLx493D_UPDATE_RATE_16_HZ_e,
} TLx493D_UpdateRateType_t;


/*
  Structure to store name, access mode, address, mask and offset of registers.
*/
typedef struct TLx493D_Register_t {
    // uint8_t                           name;
    TLx493D_RegisterAccessModeType_t  accessMode;
    uint8_t                           address;
    uint8_t                           mask;
    uint8_t                           offset;
    uint8_t                           numBits;
} TLx493D_Register_t;


typedef bool (*TLx493D_OneParamsFuncPtr)(TLx493D_t *sensor);
typedef bool (*TLx493D_TransferFuncPtr)(TLx493D_t *sensor, uint8_t *tx_buffer, uint8_t tx_len, uint8_t *rx_buffer, uint8_t rx_len);


typedef struct TLx493D_SPIParameters_t {
    uint8_t dummy;
} TLx493D_SPIParameters_t;


typedef struct TLx493D_I2CParameters_t {
    uint8_t address;
} TLx493D_I2CParameters_t;


typedef struct TLx493D_ComLibraryFunctions_t {
    union {
        TLx493D_OneParamsFuncPtr  spi_init;
        TLx493D_OneParamsFuncPtr  iic_init;
    } init;

    union {
        TLx493D_OneParamsFuncPtr  spi_deinit;
        TLx493D_OneParamsFuncPtr  iic_deinit;
    } deinit;

    union {
        TLx493D_TransferFuncPtr  spi_transfer;
        TLx493D_TransferFuncPtr  iic_transfer;
    } transfer;
} TLx493D_ComLibraryFunctions_t;


typedef union TLx493D_ComLibraryParameters_t {
    TLx493D_SPIParameters_t  spi_params;
    TLx493D_I2CParameters_t  iic_params;
} TLx493D_ComLibraryParameters_t;


typedef union TLx493D_ComLibraryObject_t {
    TLx493D_SPIObject_t  *spi_obj;
    TLx493D_I2CObject_t  *iic_obj;
} TLx493D_ComLibraryObject_t;


typedef struct TLx493D_CommunicationInterface_t {
    TLx493D_ComLibraryFunctions_t   *comLibFuncs;
    TLx493D_ComLibraryParameters_t   comLibParams;
    TLx493D_ComLibraryObject_t       comLibObj;
    // bool                             isToBeDeleted;
} TLx493D_CommunicationInterface_t;


typedef union TLx493D_BoardSupportObject_t {
    TLx493D_Kit2GoBoardSupportObject_t  *k2go_obj;
} TLx493D_BoardSupportObject_t;


typedef struct TLx493D_BoardSupportInterface_t {
    // TLx493D_BoardSupportFunctions_t   *boardSupportFuncs;
    // TLx493D_BoardSupportParameters_t   boardSupportParams;
    TLx493D_BoardSupportObject_t       boardSupportObj;
    // bool                               isToBeDeleted;
} TLx493D_BoardSupportInterface_t;


// Functions common to all sensors
typedef bool (*TLx493D_InitFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_DeinitFuncPtr)(TLx493D_t *);

typedef bool (*TLx493D_ReadRegistersFuncPtr)(TLx493D_t *);

typedef void (*TLx493D_CalculateRawTemperatureFuncPtr)(TLx493D_t *, int16_t *);
typedef bool (*TLx493D_GetRawTemperatureFuncPtr)(TLx493D_t *, int16_t *);

typedef void (*TLx493D_CalculateRawMagneticFieldFuncPtr)(TLx493D_t *, int16_t *, int16_t *, int16_t *);
typedef bool (*TLx493D_GetRawMagneticFieldFuncPtr)(TLx493D_t *, int16_t *, int16_t *, int16_t *);

typedef void (*TLx493D_CalculateRawMagneticFieldAndTemperatureFuncPtr)(TLx493D_t *, int16_t *, int16_t *, int16_t *, int16_t *);
typedef bool (*TLx493D_GetRawMagneticFieldAndTemperatureFuncPtr)(TLx493D_t *, int16_t *, int16_t *, int16_t *, int16_t *);

typedef void (*TLx493D_CalculateTemperatureFuncPtr)(TLx493D_t *, double *);
typedef bool (*TLx493D_GetTemperatureFuncPtr)(TLx493D_t *, double *);

typedef void (*TLx493D_CalculateMagneticFieldFuncPtr)(TLx493D_t *, double *, double *, double *);
typedef bool (*TLx493D_GetMagneticFieldFuncPtr)(TLx493D_t *, double *, double *, double *);

typedef void (*TLx493D_CalculateMagneticFieldAndTemperatureFuncPtr)(TLx493D_t *, double *, double *, double *, double *);
typedef bool (*TLx493D_GetMagneticFieldAndTemperatureFuncPtr)(TLx493D_t *, double *, double *, double *, double *);


// functions related to the "Config" register
typedef bool (*TLx493D_SetMeasurementFuncPtr)(TLx493D_t *, TLx493D_MeasurementType_t);
typedef bool (*TLx493D_SetTriggerFuncPtr)(TLx493D_t *, uint8_t);
typedef bool (*TLx493D_SetSetSensitivityFuncPtr)(TLx493D_t *, TLx493D_SensitivityType_t);


// functions related to the "Mod1" and "Mod2" registers
typedef bool (*TLx493D_SetDefaultConfigFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_SetIICAddressFuncPtr)(TLx493D_t *, TLx493D_IICAddressType_t);
typedef bool (*TLx493D_Enable1ByteReadModeFuncPtr)(TLx493D_t *);

typedef bool (*TLx493D_EnableInterruptFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_DisableInterruptFuncPtr)(TLx493D_t *);

typedef bool (*TLx493D_EnableCollisionAvoidanceFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_DisableCollisionAvoidanceFuncPtr)(TLx493D_t *);

typedef bool (*TLx493D_SetPowerModeFuncPtr)(TLx493D_t *, TLx493D_PowerModeType_t);
typedef bool (*TLx493D_SetUpdateRateFuncPtr)(TLx493D_t *, TLx493D_UpdateRateType_t);

// functions related to the "Diag" register
typedef bool (*TLx493D_HasValidDataFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_IsFunctionalFuncPtr)(TLx493D_t *);


// functions available only to a subset of sensors with wake-up functionality
// functions related to the "WU" register
typedef bool (*TLx493D_HasWakeUpFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_IsWakeUpEnabledFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_EnableWakeUpModeFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_DisableWakeUpModeFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_SetWakeUpThresholdsAsIntegerFuncPtr)(TLx493D_t *, int16_t, int16_t, int16_t, int16_t, int16_t, int16_t);
typedef bool (*TLx493D_SetWakeUpThresholdsFuncPtr)(TLx493D_t *, double, double, double, double, double, double, double);

typedef bool (*TLx493D_SoftwareResetFuncPtr)(TLx493D_t *);


// functions used internally and not accessible through the common interface
typedef uint8_t (*TLx493D_CalculateFuseParityFuncPtr)(TLx493D_t *);
typedef uint8_t (*TLx493D_CalculateBusParityFuncPtr)(TLx493D_t *);
typedef uint8_t (*TLx493D_CalculateConfigParityFuncPtr)(TLx493D_t *);

typedef bool (*TLx493D_HasValidFuseParityFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_HasValidBusParityFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_HasValidConfigParityFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_HasValidWakeUpParityFuncPtr)(TLx493D_t *);

typedef bool (*TLx493D_HasValidTBitFuncPtr)(TLx493D_t *);
typedef bool (*TLx493D_isInTestModeFuncPtr)(TLx493D_t *);

typedef void (*TLx493D_SetResetValuesFuncPtr)(TLx493D_t *);

typedef void (*TLx493D_CalculateRawMagneticFieldAtTemperatureFuncPtr)(TLx493D_t *, int16_t, TLx493D_SensitivityType_t,
                                                                      double, double, double,
                                                                      int16_t *, int16_t *, int16_t *);

typedef double (*TLx493D_GetSensitivityScaleFactorFuncPtr)(TLx493D_t *);

typedef uint8_t (*TLx493D_SelectIICAddressFuncPtr)(TLx493D_t *, TLx493D_IICAddressType_t);


// Functions used to refer to sensor specific functions by a common name. These functions are not part of the common user C/C++ interface.
typedef struct TLx493D_CommonFunctions_t {
    TLx493D_InitFuncPtr                         init;
    TLx493D_DeinitFuncPtr                       deinit;

    TLx493D_ReadRegistersFuncPtr                readRegisters;

    TLx493D_CalculateRawTemperatureFuncPtr      calculateRawTemperature;
    TLx493D_GetRawTemperatureFuncPtr            getRawTemperature;

    TLx493D_CalculateRawMagneticFieldFuncPtr    calculateRawMagneticField;
    TLx493D_GetRawMagneticFieldFuncPtr          getRawMagneticField;

    TLx493D_CalculateRawMagneticFieldAndTemperatureFuncPtr  calculateRawMagneticFieldAndTemperature;
    TLx493D_GetRawMagneticFieldAndTemperatureFuncPtr        getRawMagneticFieldAndTemperature;

    TLx493D_CalculateTemperatureFuncPtr         calculateTemperature;
    TLx493D_GetTemperatureFuncPtr               getTemperature;

    TLx493D_CalculateMagneticFieldFuncPtr       calculateMagneticField;
    TLx493D_GetMagneticFieldFuncPtr             getMagneticField;
    
    TLx493D_CalculateMagneticFieldAndTemperatureFuncPtr  calculateMagneticFieldAndTemperature;
    TLx493D_GetMagneticFieldAndTemperatureFuncPtr        getMagneticFieldAndTemperature;

    // functions related to the "Config" register
    TLx493D_SetMeasurementFuncPtr             setMeasurement;
    TLx493D_SetTriggerFuncPtr                   setTrigger;
    TLx493D_SetSetSensitivityFuncPtr            setSensitivity;

    
    // functions related to the "Mod1" and "Mod2" registers
    TLx493D_SetDefaultConfigFuncPtr             setDefaultConfig;
    TLx493D_SetIICAddressFuncPtr                setIICAddress;
    TLx493D_Enable1ByteReadModeFuncPtr          enable1ByteReadMode;

    TLx493D_EnableInterruptFuncPtr              enableInterrupt;
    TLx493D_DisableInterruptFuncPtr             disableInterrupt;

    TLx493D_EnableCollisionAvoidanceFuncPtr     enableCollisionAvoidance;
    TLx493D_DisableCollisionAvoidanceFuncPtr    disableCollisionAvoidance;

    TLx493D_SetPowerModeFuncPtr                 setPowerMode;
    TLx493D_SetUpdateRateFuncPtr                setUpdateRate;

    // functions related to the "Diag" register
    TLx493D_HasValidDataFuncPtr                 hasValidData;
    TLx493D_IsFunctionalFuncPtr                 isFunctional;


    // functions available only to a subset of sensors with wake-up functionality
    // functions related to the "WU" register
    TLx493D_HasWakeUpFuncPtr                    hasWakeUp;
    TLx493D_IsWakeUpEnabledFuncPtr              isWakeUpEnabled;
    TLx493D_EnableWakeUpModeFuncPtr             enableWakeUpMode;
    TLx493D_DisableWakeUpModeFuncPtr            disableWakeUpMode;

    TLx493D_SetWakeUpThresholdsAsIntegerFuncPtr setWakeUpThresholdsAsInteger;                
    TLx493D_SetWakeUpThresholdsFuncPtr          setWakeUpThresholds;                

    TLx493D_SoftwareResetFuncPtr                softwareReset;
    

    // functions used internally and not accessible through the common interface
    TLx493D_CalculateFuseParityFuncPtr          calculateFuseParity;
    TLx493D_CalculateBusParityFuncPtr           calculateBusParity;
    TLx493D_CalculateConfigParityFuncPtr        calculateConfigurationParity;
    
    TLx493D_HasValidFuseParityFuncPtr           hasValidFuseParity;
    TLx493D_HasValidBusParityFuncPtr            hasValidBusParity;
    TLx493D_HasValidConfigParityFuncPtr         hasValidConfigurationParity;
   
    TLx493D_HasValidWakeUpParityFuncPtr         hasValidWakeUpParity;
    TLx493D_isInTestModeFuncPtr                 isInTestMode;

    TLx493D_HasValidTBitFuncPtr                 hasValidTBit;
 
    TLx493D_SetResetValuesFuncPtr               setResetValues;

    TLx493D_SelectIICAddressFuncPtr             selectIICAddress;

    TLx493D_CalculateRawMagneticFieldAtTemperatureFuncPtr  calculateRawMagneticFieldAtTemperature;

    TLx493D_GetSensitivityScaleFactorFuncPtr    getSensitivityScaleFactor;
} TLx493D_CommonFunctions_t;


/*
  Structure to store all relevant infos for a particular sensor.
*/
typedef struct TLx493D_t {
    uint8_t                         *regMap;
    TLx493D_Register_t              *regDef;
    TLx493D_CommonFunctions_t       *functions;

    TLx493D_CommunicationInterface_t  comInterface;

    TLx493D_BoardSupportInterface_t   boardSupportInterface;

    uint8_t                                     regMapSize;
    TLx493D_SupportedSensorType_t               sensorType;
    TLx493D_SupportedComLibraryInterfaceType_t  comIFType;
} TLx493D_t;


#ifdef __cplusplus

}

#endif


#endif // TLX493D_TYPES_H