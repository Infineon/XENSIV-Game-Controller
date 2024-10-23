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

#include "CommunicationInterface.h"


void tlx493d_gen_2_calculateRawTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, int16_t *temperature) {
    tlx493d_common_calculateRawTemperature(sensor, tempMSBBF, tempLSBBF, temperature);
}


void tlx493d_gen_2_calculateRawMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                          uint8_t bzMSBBF, uint8_t bzLSBBF, int16_t *x, int16_t *y, int16_t *z) {
    tlx493d_common_calculateRawMagneticField(sensor, bxMSBBF, bxLSBBF, byMSBBF, byLSBBF, bzMSBBF, bzLSBBF, x, y, z);
}


void tlx493d_gen_2_calculateTemperature(TLx493D_t *sensor, uint8_t tempMSBBF, uint8_t tempLSBBF, double *temperature) {
    int16_t value = 0;

    tlx493d_common_concatBytes(sensor, tempMSBBF, tempLSBBF, &value);

    value <<= 2; // least significant 2 bits are implicit, therefore shift by 2 !
    *temperature = (((double) value - GEN_2_TEMP_OFFSET) * GEN_2_TEMP_RESOLUTION) + GEN_2_TEMP_REF;
}


void tlx493d_gen_2_calculateMagneticField(TLx493D_t *sensor, uint8_t bxMSBBF, uint8_t bxLSBBF, uint8_t byMSBBF, uint8_t byLSBBF,
                                          uint8_t bzMSBBF, uint8_t bzLSBBF, double *x, double *y, double *z) {
    int16_t valueX = 0, valueY = 0, valueZ = 0;

    tlx493d_common_concatBytes(sensor, bxMSBBF, bxLSBBF, &valueX);
    tlx493d_common_concatBytes(sensor, byMSBBF, byLSBBF, &valueY);
    tlx493d_common_concatBytes(sensor, bzMSBBF, bzLSBBF, &valueZ);

    double scaledSensitivity = GEN_2_FULL_RANGE_FIELD_SENSITIVITY * sensor->functions->getSensitivityScaleFactor(sensor);

    *x = ((double) valueX) / scaledSensitivity;
    *y = ((double) valueY) / scaledSensitivity;
    *z = ((double) valueZ) / scaledSensitivity;                           
    // *x = ((double) valueX) * GEN_2_MAG_FIELD_MULT;
    // *y = ((double) valueY) * GEN_2_MAG_FIELD_MULT;
    // *z = ((double) valueZ) * GEN_2_MAG_FIELD_MULT;
}


bool tlx493d_gen_2_setOneConfigBitfield(TLx493D_t *sensor, uint8_t bfBF, uint8_t cpBF, uint8_t val) {
    tlx493d_common_setBitfield(sensor, bfBF, val);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, cpBF);
}


bool tlx493d_gen_2_setTwoConfigBitfields(TLx493D_t *sensor, uint8_t firstBF, uint8_t secondBF, uint8_t cpBF, uint8_t first, uint8_t second) {
    tlx493d_common_setBitfield(sensor, firstBF, first);
    tlx493d_common_setBitfield(sensor, secondBF, second);
    tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_common_writeRegister(sensor, cpBF);
}


bool tlx493d_gen_2_setMeasurement(TLx493D_t *sensor, uint8_t dtBF, uint8_t amBF, uint8_t cpBF, TLx493D_MeasurementType_t val) {
    uint8_t dt = 0;
    uint8_t am = 0;

    switch(val) {
        case TLx493D_BxByBzTemp_e : dt = 0;
                                    am = 0;
                                    break;

        case TLx493D_BxByBz_e : dt = 1;
                                am = 0;
                                break;
        
        case TLx493D_BxBy_e : dt = 1;
                              am = 1;
                              break;
        
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_MeasurementType_t");
                  return false;
    }

    return tlx493d_gen_2_setTwoConfigBitfields(sensor, dtBF, amBF, cpBF, dt, am);
}


//  // This option depends on PR and MODE.
bool tlx493d_gen_2_setTrigger(TLx493D_t *sensor, uint8_t trigBF, uint8_t cpBF, TLx493D_TriggerType_t val) {
    uint8_t trig = 0;

    switch(val) {
        case TLx493D_NO_ADC_ON_READ_e : trig = 0;
                                        break;

        case TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e : trig = 1;
                                                      break;
        
        case TLx493D_ADC_ON_READ_AFTER_REG_05_e : trig = 2;
                                                  break;
        
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_TriggerType_t");
                  return false;
    }

    return tlx493d_gen_2_setOneConfigBitfield(sensor, trigBF, cpBF, trig);
}


bool tlx493d_gen_2_setSensitivity(TLx493D_t *sensor, TLx493D_AvailableSensitivityType_t availSens, uint8_t x2BF, uint8_t x4BF, uint8_t cpBF,
                                  TLx493D_SensitivityType_t val) {
    uint8_t shortVal      = 0;
    uint8_t extraShortVal = 0;

    switch(availSens) {
        case TLx493D_HAS_X1_e : return val == TLx493D_FULL_RANGE_e;
    
        case TLx493D_HAS_X2_e : return tlx493d_gen_2_setOneConfigBitfield(sensor, x2BF, cpBF, val == TLx493D_FULL_RANGE_e ? 0 : 1);
    
        case TLx493D_HAS_X4_e : {  
                                    switch(val) {
                                        case TLx493D_FULL_RANGE_e  : shortVal      = 0;
                                                                     extraShortVal = 0;
                                                                     break;

                                        case TLx493D_SHORT_RANGE_e : shortVal      = 1;
                                                                     extraShortVal = 0;
                                                                     break;
                                     
                                        case TLx493D_EXTRA_SHORT_RANGE_e : shortVal      = 1;
                                                                           extraShortVal = 1;
                                                                           break;

                                        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_SensitivityType_t");
                                                  return false;
                                    }

// print("\nsetSensitivity    short : %d    xtra_short : %d\n", shortVal, extraShortVal);
                                    // return (
                                    tlx493d_gen_2_setOneConfigBitfield(sensor, x4BF, cpBF, extraShortVal);
                                    return tlx493d_gen_2_setOneConfigBitfield(sensor, x2BF, cpBF, shortVal);
                                                //: false);
        }

        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, availSens, "TLx493D_AvailableSensitivityType_t");
                  return false;
    }
    
    return false;
}


// bool tlx493d_gen_2_setMagneticTemperatureCompensation(TLx493D_t *sensor, uint8_t tl_magBF, uint8_t cpBF, uint8_t mtc) {
// return tlx493d_gen_2_setOneConfigBitfield(sensor, tl_magBF, cpBF, mtc);
// }


bool tlx493d_gen_2_setDefaultConfig(TLx493D_t *sensor, uint8_t configREG, uint8_t mod1REG, uint8_t mod2REG, uint8_t cpBF, uint8_t caBF, uint8_t intBF) {
    tlx493d_common_setBitfield(sensor, caBF, 0);
    tlx493d_common_setBitfield(sensor, intBF, 1);

    if( sensor->functions->enable1ByteReadMode(sensor) ) {
        sensor->functions->readRegisters(sensor);

        if( tlx493d_common_returnBitfield(sensor, cpBF) == 0x01 && !sensor->functions->hasValidConfigurationParity(sensor) ) {
            tlx493d_common_setBitfield(sensor, cpBF, 0x00);
        }
        else {
            tlx493d_common_setBitfield(sensor, cpBF, sensor->functions->calculateConfigurationParity(sensor));
        }

        tlx493d_common_writeRegister(sensor, cpBF);
    }

    return false;
}


bool tlx493d_gen_2_setIICAddress(TLx493D_t *sensor, uint8_t iicadrBF, uint8_t fpBF, TLx493D_IICAddressType_t addr) {
    uint8_t bitfieldValue = 0;
    uint8_t deviceAddress = 0;

    switch (addr) {
        case TLx493D_IIC_ADDR_A0_e:
            bitfieldValue = 0b00;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A0;
            break;

        case TLx493D_IIC_ADDR_A1_e:
            bitfieldValue = 0b01;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A1;
            break;

        case TLx493D_IIC_ADDR_A2_e:
            bitfieldValue = 0b10;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A2;
            break;

        case TLx493D_IIC_ADDR_A3_e:
            bitfieldValue = 0b11;
            deviceAddress = GEN_2_STD_IIC_ADDR_WRITE_A3;
            break;
        
        default:
            return false;
    }

    tlx493d_common_setBitfield(sensor, iicadrBF, bitfieldValue);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    bool b = tlx493d_common_writeRegister(sensor, fpBF);
    tlx493d_common_setIICAddress(sensor, deviceAddress);

    return b;
}


bool tlx493d_gen_2_set1ByteReadMode(TLx493D_t *sensor, uint8_t prBF, uint8_t fpBF, uint8_t prdBF, uint8_t pr) {
    tlx493d_common_setBitfield(sensor, prBF, pr);
    tlx493d_common_setBitfield(sensor, fpBF, tlx493d_gen_2_calculateFuseParity(sensor, fpBF, prdBF));

    return tlx493d_common_writeRegister(sensor, prBF);
}


// CA only in Low Power and MCM mode, not in Fast Mode !
// MODE depends on PR and TRIG !
bool tlx493d_gen_2_setCollisionAvoidance(TLx493D_t *sensor, uint8_t caBF, uint8_t fpBF, uint8_t prdBF, uint8_t ca) {
    tlx493d_common_setBitfield(sensor, caBF, ca);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    return tlx493d_common_writeRegister(sensor, caBF);
}


bool tlx493d_gen_2_setInterrupt(TLx493D_t *sensor, uint8_t intBF, uint8_t fpBF, uint8_t prdBF, uint8_t irq) {
    tlx493d_common_setBitfield(sensor, intBF, irq);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    return tlx493d_common_writeRegister(sensor, intBF);
}


bool tlx493d_gen_2_setPowerMode(TLx493D_t *sensor, uint8_t modeBF, uint8_t fpBF, TLx493D_PowerModeType_t val) {
    uint8_t mod1 = sensor->regDef[fpBF].address;
    uint8_t mode = 0;

    switch(val) {
        case TLx493D_LOW_POWER_MODE_e : mode = 0b00;
                                break;

        case TLx493D_MASTER_CONTROLLED_MODE_e : mode = 0b01;
                                        break;

        case TLx493D_FAST_MODE_e : mode = 0b11;
                                  break;

        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_PowerModeType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, modeBF, mode);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    uint8_t buf[4] = { mod1,
                       sensor->regMap[mod1],
                       sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
                       sensor->regMap[mod1 + 2]
                     };

    return tlx493d_transfer(sensor, buf, sizeof(buf), NULL, 0);
}


bool tlx493d_gen_2_setUpdateRate(TLx493D_t *sensor, uint8_t fpBF, uint8_t prdBF, TLx493D_UpdateRateType_t val) {
    uint8_t mod1 = sensor->regDef[fpBF].address;
    uint8_t rate = 0;

    switch(val) {
        case TLx493D_UPDATE_RATE_770_HZ_e : rate = 0b000;
                                    break;

        case TLx493D_UPDATE_RATE_97_HZ_e : rate = 0b001;
                                   break;

        case TLx493D_UPDATE_RATE_24_HZ_e : rate = 0b010;
                                   break;

        case TLx493D_UPDATE_RATE_12_HZ_e : rate = 0b011;
                                   break;

        case TLx493D_UPDATE_RATE_6_HZ_e : rate = 0b100;
                                  break;

        case TLx493D_UPDATE_RATE_3_HZ_e : rate = 0b101;
                                  break;

        case TLx493D_UPDATE_RATE_0_4_HZ_e : rate = 0b110;
                                    break;

        case TLx493D_UPDATE_RATE_0_05_HZ_e : rate = 0b111;
                                     break;

        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, val, "TLx493D_UpdateRateType_t");
                  return false;
    }

    tlx493d_common_setBitfield(sensor, prdBF, rate);
    tlx493d_common_setBitfield(sensor, fpBF, sensor->functions->calculateFuseParity(sensor));

    uint8_t buf[4] = { mod1,
                       sensor->regMap[mod1],
                       sensor->regMap[mod1 + 1], // reserved register must have been read once in setDefaultConfig to get factory settings !
                       sensor->regMap[mod1 + 2]
                     };

    return tlx493d_transfer(sensor, buf, sizeof(buf), NULL, 0);
}


bool tlx493d_gen_2_hasValidData(TLx493D_t *sensor) {
    return sensor->functions->hasValidBusParity(sensor) && sensor->functions->hasValidTBit(sensor);
}


bool tlx493d_gen_2_isFunctional(TLx493D_t *sensor) {
    return sensor->functions->hasValidFuseParity(sensor) && sensor->functions->hasValidConfigurationParity(sensor);
}


// bool tlx493d_gen_2_hasWakeUp(TLx493D_t *sensor, uint8_t typeBF) {
//     TLx493D_Register_t *type = &sensor->regDef[typeBF];
//     return ((sensor->regMap[type->address] & type->mask) >> type->offset) != 0b11;
// }


bool tlx493d_gen_2_isWakeUpEnabled(TLx493D_t *sensor, uint8_t waBF) {
    return tlx493d_common_returnBitfield(sensor, waBF) != 0;
}


bool tlx493d_gen_2_writeConfigurationRegisters(TLx493D_t *sensor) {
    uint8_t txBuffer[11] = {
                               0x07,
                               sensor->regMap[0x07],
                               sensor->regMap[0x08],
                               sensor->regMap[0x09],
                               sensor->regMap[0x0A],
                               sensor->regMap[0x0B],
                               sensor->regMap[0x0C],
                               sensor->regMap[0x0D],
                               sensor->regMap[0x0E],
                               sensor->regMap[0x0F],
                               sensor->regMap[0x10]
                           };

    return tlx493d_transfer(sensor, txBuffer, sizeof(txBuffer), NULL, 0);
}

bool tlx493d_gen_2_enableWakeUpMode(TLx493D_t *sensor, uint8_t tstBF, uint8_t wuBF, uint8_t cpbBF) {
    bool isTestMode = tlx493d_common_returnBitfield(sensor, tstBF) != 0;

    if( isTestMode ) {
        tlx493d_common_setBitfield(sensor, tstBF, 0);
        tlx493d_common_setBitfield(sensor, cpbBF, sensor->functions->calculateConfigurationParity(sensor));
        
        if( !tlx493d_gen_2_writeConfigurationRegisters(sensor) ) {
            return false;
        }
    }

    tlx493d_common_setBitfield(sensor, wuBF, 1);
    tlx493d_common_setBitfield(sensor, cpbBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_gen_2_writeConfigurationRegisters(sensor) ? sensor->functions->readRegisters(sensor) : false;
}


bool tlx493d_gen_2_disableWakeUpMode(TLx493D_t *sensor, uint8_t wuBF, uint8_t cpbBF) {
    tlx493d_common_setBitfield(sensor, wuBF, 0);
    tlx493d_common_setBitfield(sensor, cpbBF, sensor->functions->calculateConfigurationParity(sensor));

    return tlx493d_gen_2_writeConfigurationRegisters(sensor) ? sensor->functions->readRegisters(sensor) : false;
}


bool tlx493d_gen_2_setThreshold(TLx493D_t *sensor, uint8_t msbsBF, uint8_t lsbsBF, int16_t threshold12Bits) {
    TLx493D_Register_t *msbs = &sensor->regDef[msbsBF];
    TLx493D_Register_t *lsbs = &sensor->regDef[lsbsBF];

    int16_t thresh11Bits = threshold12Bits >> 1;

    uint8_t lower = thresh11Bits & (lsbs->mask >> lsbs->offset);
    uint8_t upper = (thresh11Bits >> lsbs->numBits) & (msbs->mask >> msbs->offset);

    // print("threshold12Bits = %#X / %d   thresh11Bits = %#X / %d  upper = %#X / %d  lower = %#X / %d\n",
    //       threshold12Bits, threshold12Bits, thresh11Bits, thresh11Bits, upper, upper, lower, lower);

    tlx493d_common_setBitfield(sensor, msbsBF, upper);
    tlx493d_common_setBitfield(sensor, lsbsBF, lower);

    return tlx493d_gen_2_writeConfigurationRegisters(sensor);
}


bool tlx493d_gen_2_setWakeUpThresholdsAsInteger(TLx493D_t *sensor,
                                                uint8_t xlMSBBF, uint8_t xlLSBBF, uint8_t xhMSBBF, uint8_t xhLSBBF,
                                                uint8_t ylMSBBF, uint8_t ylLSBBF, uint8_t yhMSBBF, uint8_t yhLSBBF,
                                                uint8_t zlMSBBF, uint8_t zlLSBBF, uint8_t zhMSBBF, uint8_t zhLSBBF,
                                                int16_t xlTh, int16_t xhTh, int16_t ylTh, int16_t yhTh, int16_t zlTh, int16_t zhTh) {
    bool retVal = tlx493d_gen_2_setThreshold(sensor, xlMSBBF, xlLSBBF, xlTh);
    retVal &= tlx493d_gen_2_setThreshold(sensor, xhMSBBF, xhLSBBF, xhTh);

    retVal &= tlx493d_gen_2_setThreshold(sensor, ylMSBBF, ylLSBBF, ylTh);
    retVal &= tlx493d_gen_2_setThreshold(sensor, yhMSBBF, yhLSBBF, yhTh);

    retVal &= tlx493d_gen_2_setThreshold(sensor, zlMSBBF, zlLSBBF, zlTh);
    retVal &= tlx493d_gen_2_setThreshold(sensor, zhMSBBF, zhLSBBF, zhTh);

    sensor->functions->readRegisters(sensor);

    return retVal;
}


bool tlx493d_gen_2_setWakeUpThresholds(TLx493D_t *sensor,
                                       uint8_t xlMSBBF, uint8_t xlLSBBF, uint8_t xhMSBBF, uint8_t xhLSBBF,
                                       uint8_t ylMSBBF, uint8_t ylLSBBF, uint8_t yhMSBBF, uint8_t yhLSBBF,
                                       uint8_t zlMSBBF, uint8_t zlLSBBF, uint8_t zhMSBBF, uint8_t zhLSBBF,
                                       TLx493D_AvailableSensitivityType_t availSens, uint8_t x2BF, uint8_t x4BF,
                                       double temperature, double xLow, double xHigh, double yLow, double yHigh, double zLow, double zHigh) {
    int16_t xlTh, ylTh, zlTh, xhTh, yhTh, zhTh, tr;

    tlx493d_gen_2_convertTemperatureToRaw(sensor, temperature, &tr);

    TLx493D_SensitivityType_t sens = tlx493d_gen_2_getSensitivityType(sensor, availSens, x2BF, x4BF);

    sensor->functions->calculateRawMagneticFieldAtTemperature(sensor, tr, sens, xLow,  yLow,  zLow,  &xlTh, &ylTh, &zlTh);
    sensor->functions->calculateRawMagneticFieldAtTemperature(sensor, tr, sens, xHigh, yHigh, zHigh, &xhTh, &yhTh, &zhTh);
// print("sens = %d\n", sens);
// printDouble(temperature);
// print("\nt = %d\n", tr);
// print("xl = %d   xh = %d\n", xlTh, xhTh);
// print("yl = %d   yh = %d\n", ylTh, yhTh);
// print("zl = %d   zh = %d\n", zlTh, zhTh);

    return tlx493d_gen_2_setWakeUpThresholdsAsInteger(sensor,
                                                      xlMSBBF, xlLSBBF, xhMSBBF, xhLSBBF,
                                                      ylMSBBF, ylLSBBF, yhMSBBF, yhLSBBF,
                                                      zlMSBBF, zlLSBBF, zhMSBBF, zhLSBBF,
                                                      xlTh, xhTh, ylTh, yhTh, zlTh, zhTh);
}


// Fuse/mode parity bit FP
uint8_t tlx493d_gen_2_calculateFuseParity(TLx493D_t *sensor, uint8_t fpBF, uint8_t prdBF) {
    TLx493D_Register_t *fp  = &sensor->regDef[fpBF];
    TLx493D_Register_t *prd = &sensor->regDef[prdBF];

	uint8_t parity = sensor->regMap[fp->address] & ~fp->mask;
	parity ^= sensor->regMap[prd->address] & prd->mask;

	return tlx493d_common_getOddParity(tlx493d_common_calculateParity(parity));
}


// Calculate bus (data) parity bit P
uint8_t tlx493d_gen_2_calculateBusParity(TLx493D_t *sensor, uint8_t to) {
	uint8_t parity = sensor->regMap[0];

	for (uint8_t i = 1; i < to; ++i) {
		parity ^= sensor->regMap[i];
	}

	return tlx493d_common_getOddParity(tlx493d_common_calculateParity(parity));
}


uint8_t tlx493d_gen_2_calculateConfigurationParity(TLx493D_t *sensor, uint8_t cpBF) {
    TLx493D_Register_t *cp     = &sensor->regDef[cpBF];
	uint8_t             parity = tlx493d_common_calculateParity(sensor->regMap[cp->address] & ~cp->mask);
	return tlx493d_common_getEvenParity(parity);
}


uint8_t tlx493d_gen_2_calculateConfigurationParityWakeUp(TLx493D_t *sensor, uint8_t cpBF) {
// uint8_t tlx493d_gen_2_calculateConfigurationParityWakeUp(TLx493D_t *sensor, uint8_t waBF, uint8_t tstBF, uint8_t phBF, uint8_t cpBF) {
// uint8_t tlx493d_gen_2_calculateConfigurationParityWakeup(TLx493D_t *sensor, uint8_t cpBF, uint8_t from, uint8_t to) {
    TLx493D_Register_t *cp     = &sensor->regDef[cpBF];
	uint8_t              parity = sensor->regMap[0x07]; //from];

    for(uint8_t i = 0x08; i <= 0x0C; ++i) {
    // for(uint8_t i = from + 1; i <= 0x0C; ++i) {
        parity ^= sensor->regMap[i];
    }

    parity ^= (sensor->regMap[0x0D] & 0x7F); // WA
    parity ^= (sensor->regMap[0x0E] & 0x3F); // TST
    parity ^= (sensor->regMap[0x0F] & 0x3F); // PH
//     parity ^= (sensor->regMap[0x0D] & ~sensor->regDef[waBF].mask);
//     parity ^= (sensor->regMap[0x0E] & ~sensor->regDef[tstBF].mask);
//     parity ^= (sensor->regMap[0x0F] & ~sensor->regDef[phBF].mask);

    parity ^= sensor->regMap[0x10] & ~cp->mask;

	return tlx493d_common_getOddParity(tlx493d_common_calculateParity(parity));
}


bool tlx493d_gen_2_hasValidFuseParity(TLx493D_t *sensor, uint8_t ffBF) {
    return tlx493d_common_returnBitfield(sensor, ffBF) != 0;
}


bool tlx493d_gen_2_hasValidBusParity(TLx493D_t *sensor, uint8_t pBF) {
    return sensor->functions->calculateBusParity(sensor) == tlx493d_common_returnBitfield(sensor, pBF);
}


bool tlx493d_gen_2_hasValidConfigurationParity(TLx493D_t *sensor, uint8_t cfBF) {
    return tlx493d_common_returnBitfield(sensor, cfBF) != 0;
}


bool tlx493d_gen_2_hasValidIICadr(TLx493D_t *sensor, uint8_t idBF, uint8_t iicAdrBF) {
    return tlx493d_common_returnBitfield(sensor, idBF) == tlx493d_common_returnBitfield(sensor, iicAdrBF);
}


bool tlx493d_gen_2_hasValidTBit(TLx493D_t *sensor, uint8_t tBF) {
    return tlx493d_common_returnBitfield(sensor, tBF) == 0;
}


// bool tlx493d_gen_2_hasValidTemperatureData(TLx493D_t *sensor) {
//     return sensor->functions->hasValidData(sensor) && sensor->functions->hasValidPD3Bit(sensor);
// }


// bool tlx493d_gen_2_hasValidMagneticFieldData(TLx493D_t *sensor) {
//     return sensor->functions->hasValidData(sensor) && sensor->functions->hasValidPD0Bit(sensor);
// }


// bool tlx493d_gen_2_hasValidPD3Bit(TLx493D_t *sensor, uint8_t pd3BF) {
//     TLx493D_Register_t *pd3 = &sensor->regDef[pd3BF];
//     return (sensor->regMap[pd3->address] & pd3->mask) != 0;
// }


// bool tlx493d_gen_2_hasValidPD0Bit(TLx493D_t *sensor, uint8_t pd0BF) {
//     TLx493D_Register_t *pd0 = &sensor->regDef[pd0BF];
//     return (sensor->regMap[pd0->address] & pd0->mask) != 0;
// }


// uint8_t tlx493d_gen_2_getID(TLx493D_t *sensor, uint8_t idBF) {
//     TLx493D_Register_t *bf = &sensor->regDef[idBF];
//     return (sensor->regMap[bf->address] && bf->mask) >> bf->offset;
// }


// uint8_t tlx493d_gen_2_getFrameCounter(TLx493D_t *sensor, uint8_t frmBF) {
//     TLx493D_Register_t *frm = &sensor->regDef[frmBF];
//     return (sensor->regMap[frm->address] && frm->mask) >> frm->offset;
// }


// uint8_t tlx493d_gen_2_getType(TLx493D_t *sensor, uint8_t typeBF) {
//     TLx493D_Register_t *type = &sensor->regDef[typeBF];
//     return (sensor->regMap[type->address] && type->mask) >> type->offset;
// }


// uint8_t tlx493d_gen_2_getHWV(TLx493D_t *sensor, uint8_t hwvBF) {
//     TLx493D_Register_t *hwv = &sensor->regDef[hwvBF];
//     return (sensor->regMap[hwv->address] && hwv->mask) >> hwv->offset;
// }


uint8_t tlx493d_gen_2_selectIICAddress(TLx493D_t *sensor, TLx493D_IICAddressType_t addr) {
    switch(addr) {
        case TLx493D_IIC_ADDR_A0_e : return GEN_2_STD_IIC_ADDR_WRITE_A0;

        case TLx493D_IIC_ADDR_A1_e : return GEN_2_STD_IIC_ADDR_WRITE_A1;

        case TLx493D_IIC_ADDR_A2_e : return GEN_2_STD_IIC_ADDR_WRITE_A2;

        case TLx493D_IIC_ADDR_A3_e : return GEN_2_STD_IIC_ADDR_WRITE_A3;

        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, addr, "TLx493D_IICAddressType_t");
                  return 0;
    }
}


double tlx493d_gen_2_getSensitivityScaleFactor(TLx493D_t *sensor, TLx493D_AvailableSensitivityType_t availSens, uint8_t x2BF, uint8_t x4BF) {
    switch(availSens) {
        case TLx493D_HAS_X1_e : return 1.0;
    
        case TLx493D_HAS_X2_e : return tlx493d_common_returnBitfield(sensor, x2BF) == 0 ? 1.0 : 2.0;
    
        case TLx493D_HAS_X4_e : {
                                    bool x2IsNotSet = tlx493d_common_returnBitfield(sensor, x2BF) == 0;
                                    bool x4IsNotSet = tlx493d_common_returnBitfield(sensor, x4BF) == 0;
                                    // print("x4 : %d\n", x4IsNotSet ? 0 : 1);
                                    return (x2IsNotSet ? 1.0 : (x4IsNotSet ? 2.0 : 4.0));
        }
    
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, availSens, "TLx493D_AvailableSensitivityType_t");
                  return 0.0;
    }
}


void tlx493d_gen_2_calculateRawMagneticFieldAtTemperature(TLx493D_t *sensor, int16_t rawTemp, TLx493D_SensitivityType_t sens,
                                                          double xInmT, double yInmT, double zInmT,
                                                          int16_t *x, int16_t *y, int16_t *z) {
    double scaledSensitivity = GEN_2_FULL_RANGE_FIELD_SENSITIVITY * sensor->functions->getSensitivityScaleFactor(sensor);
    
    *x = xInmT * scaledSensitivity;
    *y = yInmT * scaledSensitivity;
    *z = zInmT * scaledSensitivity;
}


void tlx493d_gen_2_convertTemperatureToRaw(TLx493D_t *sensor, double temperature, int16_t *rawTemperature) {
    *rawTemperature = (int16_t) (((temperature - GEN_2_TEMP_REF) / GEN_2_TEMP_RESOLUTION + GEN_2_TEMP_OFFSET) / 4.0);
}


TLx493D_SensitivityType_t tlx493d_gen_2_getSensitivityType(TLx493D_t *sensor, TLx493D_AvailableSensitivityType_t availSens, uint8_t x2BF, uint8_t x4BF) {
    switch(availSens) {
        case TLx493D_HAS_X1_e : return TLx493D_FULL_RANGE_e;
    
        case TLx493D_HAS_X2_e : return tlx493d_common_returnBitfield(sensor, x2BF) == 0 ? TLx493D_FULL_RANGE_e : TLx493D_SHORT_RANGE_e;
    
        case TLx493D_HAS_X4_e : {
                                    bool x2IsNotSet = tlx493d_common_returnBitfield(sensor, x2BF) == 0;
                                    bool x4IsNotSet = tlx493d_common_returnBitfield(sensor, x4BF) == 0;
                                    return x2IsNotSet ? TLx493D_FULL_RANGE_e : (x4IsNotSet ? TLx493D_SHORT_RANGE_e : TLx493D_EXTRA_SHORT_RANGE_e);
        }
    
        default : tlx493d_errorSelectionNotSupportedForSensorType(sensor, availSens, "TLx493D_AvailableSensitivityType_t");
                  return TLx493D_FULL_RANGE_e;
    }
}