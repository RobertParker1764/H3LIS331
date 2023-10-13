//==============================================================================
// File: H3LIS331.cpp
// Robert Parker
// 9/27/2023
// Version: 1.0
//
// This file along with H3LIS331.h defines a class used for interfacing with
// the H3LIS331 high-G, low power, 3-axis MEMS accelerometer sensor chip from
// STMicroelectronics.
// This class wraps the STMicroelectronis provided H3LIS331dl C API functions as
// contained in file h3lis331dl_reg.h.
// This implelmenetation supports only a subset of H3LIS331 functionality.
// Excluded functionality includes:
//  1. I2C serial interface is not supported. Only SPI is supported.
//==============================================================================
//==============================================================================

#include "Arduino.h"
#include "H3LIS331.hpp"
#include "h3lis331dl-pid/h3lis331dl_reg.c"

// The chip select pin used for SPI communication with the sensor
// It needs to be a global variable because the static functions used by the
// bmp3 API do not have access to the private class data member _cs
uint8_t H3LIS331_ChipSelect = 0;

// Hardware interface functions used by the LSM6DSO32 API for SPI digital interface
static int32_t spi_read(void *spiPtr, uint8_t regAddr, uint8_t *regData, uint16_t len);
static int32_t spi_write(void *spiPtr, uint8_t regAddr, const uint8_t *regData, uint16_t len);

//=========================================================================
// Constructor
//=========================================================================
// Currently a do-nothing constructor.
//
// Parameters: None
// Return:  A new LSM6DSO32 instance object
//==========================================================================
H3LIS331::H3LIS331(void) {}

//=========================================================================
// begin
//=========================================================================
// Initializes the class object and sensor before first use. Following the
// call to begin() the sensor will be configured as follows:
//  Accelerometer full scale range: +/-100 g
//  Power-down mode (no output)
//  High pass filter bypassed
//  No interrupts enabled
//  Block data update is enabled
//
// Parameters:
//   cs - The processor pin used to select the sensor durint SPI transactions
//   spiPtr - A pointer to the SPI bus controller object.
//   spiFrequency - The frequency in Hz that the SPI bus clock will be set to
// Return:  0
//==========================================================================
bool H3LIS331::begin(uint8_t cs, SPIClass *spiPtr, uint32_t spiFrequency)
{
    H3LIS331_ChipSelect = cs;    // Initialize the global chipSelect variable
    
    // Set the chipSelect pin HIGH
    pinMode(H3LIS331_ChipSelect, OUTPUT);
    digitalWrite(H3LIS331_ChipSelect, HIGH);
    
    //Initialize the interface to the LSM6DSO32 API
    sensorAPI_intf.write_reg = &spi_write;
    sensorAPI_intf.read_reg = &spi_read;
    sensorAPI_intf.handle = spiPtr;
    
    accelRange = H3LIS331DL_100g;
    
    return _init();
}

//=========================================================================
// _init (Private method)
//=========================================================================
// This is a helper method for initialization of the sensor
//
// Parameters: None
// Return:  False if an error occurs during initialization. True othewise.
//==========================================================================
bool H3LIS331::_init(void) {
    uint8_t chipID;
    h3lis331dl_device_id_get(&sensorAPI_intf, &chipID);  // H3LIS331 API call
    
    // make sure we're talking to the right chip
    if (chipID != H3LIS331DL_ID) {
#ifdef H3LIS331_DEBUG
        Serial.println("Bad chip ID");
        Serial.println(chipID, HEX);
#endif
        return false;
    }
    
    // Configure sensor for Block Data Update (BDU)
    h3lis331dl_block_data_update_set(&sensorAPI_intf, 1);    // H3LIS331 API call

    return true;
}

//=========================================================================
// setRange
//=========================================================================
// Set the full scale output range of the accelerometer
//
// Parameters:
//  range - Desired full scale range. May be +/-100, +/-200, or +/-400 Gs
// Return:  None
//==========================================================================
void H3LIS331::setRange(h3lis331dl_fs_t range) {
    h3lis331dl_full_scale_set(&sensorAPI_intf, range);
    accelRange = range;
}

//=========================================================================
// getRange
//=========================================================================
// Read the current full scale output range setting of the accelerometer
//
// Parameters:
//  none
// Return:  The current output range setting
//==========================================================================
h3lis331dl_fs_t H3LIS331::getRange(void) {
    h3lis331dl_fs_t value;
    h3lis331dl_full_scale_get(&sensorAPI_intf, &value);
    
    return value;
}

//=========================================================================
// setOutputDataRate
//=========================================================================
// Set the full scale output range of the accelerometer
//
// Parameters:
//  ODR - Desired data rate. Valid vales are off, 0.5, 1, 2, 5, 10, 50,
//        100, 400, and 1k Hz.
// Return:  None
//==========================================================================
void H3LIS331::setOutputDataRate(h3lis331dl_dr_t ODR) {
    h3lis331dl_data_rate_set(&sensorAPI_intf, ODR);
}

//=========================================================================
// getOutputDataRate
//=========================================================================
// Read the current ODR setting of the accelerometer
//
// Parameters:
//  none
// Return:  The current ODR setting
//==========================================================================
h3lis331dl_dr_t H3LIS331::getOutputDataRate(void) {
    h3lis331dl_dr_t value;
    h3lis331dl_data_rate_get(&sensorAPI_intf, &value);
    return value;
}

//=========================================================================
// enableAxis
//=========================================================================
// Enable measurement in the specified accelerometer axis
//
// Parameters:
//  axis - The desired axis to enable
// Return:  None
//==========================================================================
void H3LIS331::enableAxis(h3lis331_axis_t axis) {
    if (axis != H3LIS331_XYZ) {
        // Start by disabling all the axis
        h3lis331dl_axis_x_data_set(&sensorAPI_intf, 0);
        h3lis331dl_axis_y_data_set(&sensorAPI_intf, 0);
        h3lis331dl_axis_z_data_set(&sensorAPI_intf, 0);
    }
    switch (axis) {
        case H3LIS331_NONE:
            return;
            break;
        case H3LIS331_X:
            h3lis331dl_axis_x_data_set(&sensorAPI_intf, 1);
            return;
            break;
        case H3LIS331_Y:
            h3lis331dl_axis_y_data_set(&sensorAPI_intf, 1);
            return;
            break;
        case H3LIS331_Z:
            h3lis331dl_axis_z_data_set(&sensorAPI_intf, 1);
            return;
            break;
        case H3LIS331_XY:
            h3lis331dl_axis_x_data_set(&sensorAPI_intf, 1);
            h3lis331dl_axis_y_data_set(&sensorAPI_intf, 1);
            return;
            break;
        case H3LIS331_XZ:
            h3lis331dl_axis_x_data_set(&sensorAPI_intf, 1);
            h3lis331dl_axis_z_data_set(&sensorAPI_intf, 1);
            return;
            break;
        case H3LIS331_YZ:
            h3lis331dl_axis_y_data_set(&sensorAPI_intf, 1);
            h3lis331dl_axis_z_data_set(&sensorAPI_intf, 1);
            return;
            break;
        case H3LIS331_XYZ:
            h3lis331dl_axis_x_data_set(&sensorAPI_intf, 1);
            h3lis331dl_axis_y_data_set(&sensorAPI_intf, 1);
            h3lis331dl_axis_z_data_set(&sensorAPI_intf, 1);
            return;
    }
}

//=========================================================================
// getEnabledAxis
//=========================================================================
// Read the current enabled measurement axis
//
// Parameters:
//  None
// Return:  The currently enabled measurement axis
//==========================================================================
h3lis331_axis_t H3LIS331::getEnabledAxis(void) {
    h3lis331_axis_t axis = H3LIS331_NONE;
    uint8_t value;
    
    h3lis331dl_axis_x_data_get(&sensorAPI_intf, &value);
    if (value == 1) {
        axis = H3LIS331_X;
    }
    
    h3lis331dl_axis_y_data_get(&sensorAPI_intf, &value);
    if (value == 1) {
        if (axis == H3LIS331_NONE) {
            axis = H3LIS331_Y;
        } else {
            axis = H3LIS331_XY;
        }
    }
    
    h3lis331dl_axis_z_data_get(&sensorAPI_intf, &value);
    if (value == 1) {
        if (axis == H3LIS331_NONE) {
            axis = H3LIS331_Z;
        } else if (axis == H3LIS331_X) {
            axis = H3LIS331_XZ;
        } else if (axis == H3LIS331_Y) {
            axis = H3LIS331_YZ;
        } else {
            axis = H3LIS331_XYZ;
        }
    }
    
    return axis;
}

//=========================================================================
// setBlockDataUpdate
//=========================================================================
// Enables sensor block data update
//
// Parameters:
//  None
// Return:  None
//==========================================================================
void H3LIS331::setBlockDataUpdate(void) {
    h3lis331dl_block_data_update_set(&sensorAPI_intf, 1);
    return;
}

//=========================================================================
// clearBlockDataUpdate
//=========================================================================
// Disables sensor block data update
//
// Parameters:
//  None
// Return:  None
//==========================================================================
void H3LIS331::clearBlockDataUpdate(void) {
    h3lis331dl_block_data_update_set(&sensorAPI_intf, 0);
    return;
}

//=========================================================================
// blockDataUpdateEnabled
//=========================================================================
// Returns the state of the BDU bit in control register 4.
//
// Parameters:
//  None
// Return:  Returns true of the BDU bit is set. False otherwise.
//==========================================================================
bool H3LIS331::blockDataUpdateEnabled(void) {
    uint8_t bdu;
    h3lis331dl_block_data_update_get(&sensorAPI_intf, &bdu);
    if (bdu == 1) {
        return true;
    } else {
        return false;
    }
}

//=========================================================================
// setInterruptPinsConfig
//=========================================================================
// Configures the interrupt pins
//
// Parameters:
//  pin1Source - Valid values are Boot Running, Data Ready, Interrupt 1,
//               Interrupt 1 OR Interrupt 2
//  pin2Source - Valid values are Boot Running, Data Ready, Interrupt 2,
//               Interrupt 1 OR Interrupt 2
//  latched - Interrupt pins latch an interrupt signal (default is false)
//  activeHigh - Interrupt pin outputs are active high (default is true)
//  pushPull - Interrupt pin outputs are push-pull style. If true the output
//             pins are push-pull. If false the pins outputs are open drain.
//             The default is push-pull (true)
// Return:  None
//==========================================================================
void H3LIS331::setInterruptPinsConfig(h3lis331dl_i1_cfg_t pin1Source,
                                      h3lis331dl_i2_cfg_t pin2Source,
                                      bool latched, bool activeHigh,
                                      bool pushPull) {
    h3lis331dl_pin_int1_route_set(&sensorAPI_intf, pin1Source);
    h3lis331dl_pin_int2_route_set(&sensorAPI_intf, pin2Source);
    if (latched) {
        h3lis331dl_int1_notification_set(&sensorAPI_intf, H3LIS331DL_INT1_LATCHED);
    } else {
        h3lis331dl_int1_notification_set(&sensorAPI_intf, H3LIS331DL_INT1_PULSED);
    }
    
    if (activeHigh) {
        h3lis331dl_pin_polarity_set(&sensorAPI_intf, H3LIS331DL_ACTIVE_HIGH);
    } else {
        h3lis331dl_pin_polarity_set(&sensorAPI_intf, H3LIS331DL_ACTIVE_LOW);
    }
    
    if (pushPull) {
        h3lis331dl_pin_mode_set(&sensorAPI_intf, H3LIS331DL_PUSH_PULL);
    } else {
        h3lis331dl_pin_mode_set(&sensorAPI_intf, H3LIS331DL_OPEN_DRAIN);
    }
    return;
}

//=========================================================================
// interrupt1Config
//=========================================================================
// Configures the interrupt #1 settings
//
// Parameters:
//  eventsEnabled - The interrupt event(s) that are to be enabled. This
//                  parameter is formed by ORing the desired values of type
//                  h3lis331_interrupt_events_t (X_LOW, X_HIGH, etc.)
//  orEvents - If true then interrupt events are or-ed to generate an
//             interrupt. If false then interrupt events are and-ed to
//             generate an interrupt.
//  threshold - Interrupt threshold value. The scale of this value is not
//              defined in the data sheet. Assume that the value is scaled
//              between the upper and lower acceleration limits set.
//  duration - Interrupt duration value. No information is available in the
//             data sheet regarding the scaling of this value.
// Return:  None
//==========================================================================
void H3LIS331::interrupt1Config(uint8_t eventsEnabled, bool orEvents,
                                uint8_t threshold, uint8_t duration) {
    h3lis331dl_int1_on_th_conf_t interruptEvents;
    // Enable the desired interrupt events
    if (eventsEnabled && H3LIS331_X_LOW) {
        interruptEvents.int1_xlie = 1;
    } else {
        interruptEvents.int1_xlie = 0;
    }
    
    if (eventsEnabled && H3LIS331_X_HIGH) {
        interruptEvents.int1_xhie = 1;
    } else {
        interruptEvents.int1_xhie = 0;
    }
    
    if (eventsEnabled && H3LIS331_Y_LOW) {
        interruptEvents.int1_ylie = 1;
    } else {
        interruptEvents.int1_ylie = 0;
    }
    
    if (eventsEnabled && H3LIS331_Y_HIGH) {
        interruptEvents.int1_yhie = 1;
    } else {
        interruptEvents.int1_yhie = 0;
    }
    
    if (eventsEnabled && H3LIS331_Z_LOW) {
        interruptEvents.int1_zlie = 1;
    } else {
        interruptEvents.int1_zlie = 0;
    }
    
    if (eventsEnabled && H3LIS331_Z_HIGH) {
        interruptEvents.int1_zhie = 1;
    } else {
        interruptEvents.int1_zhie = 0;
    }
    
    h3lis331dl_int1_on_threshold_conf_set(&sensorAPI_intf, interruptEvents);
    
    // Set the interrupt threshold and duration
    h3lis331dl_int1_treshold_set(&sensorAPI_intf, threshold);
    h3lis331dl_int1_dur_set(&sensorAPI_intf, duration);
}

//=========================================================================
// interrupt2Config
//=========================================================================
// Configures the interrupt #2 settings
//
// Parameters:
//  eventsEnabled - The interrupt event(s) that are to be enabled. This
//                  parameter is formed by ORing the desired values of type
//                  h3lis331_interrupt_events_t (X_LOW, X_HIGH, etc.)
//  orEvents - If true then interrupt events are or-ed to generate an
//             interrupt. If false then interrupt events are and-ed to
//             generate an interrupt.
//  threshold - Interrupt threshold value. The scale of this value is not
//              defined in the data sheet. Assume that the value is scaled
//              between the upper and lower acceleration limits set.
//  duration - Interrupt duration value. No information is available in the
//             data sheet regarding the scaling of this value.
// Return:  None
//==========================================================================
void H3LIS331::interrupt2Config(uint8_t eventsEnabled, bool orEvents,
                                uint8_t threshold, uint8_t duration) {
    
    h3lis331dl_int2_on_th_conf_t interruptEvents;
    // Enable the desired interrupt events
    if (eventsEnabled && H3LIS331_X_LOW) {
        interruptEvents.int2_xlie = 1;
    } else {
        interruptEvents.int2_xlie = 0;
    }
    
    if (eventsEnabled && H3LIS331_X_HIGH) {
        interruptEvents.int2_xhie = 1;
    } else {
        interruptEvents.int2_xhie = 0;
    }
    
    if (eventsEnabled && H3LIS331_Y_LOW) {
        interruptEvents.int2_ylie = 1;
    } else {
        interruptEvents.int2_ylie = 0;
    }
    
    if (eventsEnabled && H3LIS331_Y_HIGH) {
        interruptEvents.int2_yhie = 1;
    } else {
        interruptEvents.int2_yhie = 0;
    }
    
    if (eventsEnabled && H3LIS331_Z_LOW) {
        interruptEvents.int2_zlie = 1;
    } else {
        interruptEvents.int2_zlie = 0;
    }
    
    if (eventsEnabled && H3LIS331_Z_HIGH) {
        interruptEvents.int2_zhie = 1;
    } else {
        interruptEvents.int2_zhie = 0;
    }
    
    h3lis331dl_int2_on_threshold_conf_set(&sensorAPI_intf, interruptEvents);
    
    // Set the interrupt threshold and duration
    h3lis331dl_int2_treshold_set(&sensorAPI_intf, threshold);
    h3lis331dl_int2_dur_set(&sensorAPI_intf, duration);
}

//=========================================================================
// getStatus
//=========================================================================
// Returns the value of the status register
//
// Parameters: None
// Return:  The status register value
//==========================================================================
h3lis331dl_status_reg_t H3LIS331::getStatus(void) {
    h3lis331dl_status_reg_t value;
    
    h3lis331dl_status_reg_get(&sensorAPI_intf, &value);
    return value;
}

//=========================================================================
// isDataReady
//=========================================================================
// Returns the state of the ZYXDA bit in the status register
//
// Parameters: None
// Return:  True if the ZYXDA bit is set. False otherwise
//==========================================================================
bool H3LIS331::isDataReady(void) {
    h3lis331dl_status_reg_t value;
    h3lis331dl_status_reg_get(&sensorAPI_intf, &value);
    if (value.zyxda) {
        return true;
    } else {
        return false;
    }
}


//=========================================================================
// getX_Accel
//=========================================================================
// Returns the X axis acceleration value
//
// Parameters: None
// Return:  The acceleration value in G's
//==========================================================================
float H3LIS331::getX_Accel(void) {
    return accelerationReading[0];
}

//=========================================================================
// getY_Accel
//=========================================================================
// Returns the Y axis acceleration value
//
// Parameters: None
// Return:  The acceleration value in G's
//==========================================================================
float H3LIS331::getY_Accel(void) {
    return accelerationReading[1];
}

//=========================================================================
// getZ_Accel
//=========================================================================
// Returns the Z axis acceleration value
//
// Parameters: None
// Return:  The acceleration value in G's
//==========================================================================
float H3LIS331::getZ_Accel(void) {
    return accelerationReading[2];
}

//=========================================================================
// getAcceleration
//=========================================================================
// Reads the current X, Y, and Z axis acceleration
//
// Parameters: None
// Return:  None
//==========================================================================
void H3LIS331::getAcceleration(void) {
    int16_t values[3] = {0, 0, 0};
    
    h3lis331dl_acceleration_raw_get(&sensorAPI_intf, values);
    switch (accelRange) {
        case H3LIS331DL_100g:
            accelerationReading[0] = h3lis331dl_from_fs100_to_mg(values[0]) / 1000.0;
            accelerationReading[1] = h3lis331dl_from_fs100_to_mg(values[1]) / 1000.0;
            accelerationReading[2] = h3lis331dl_from_fs100_to_mg(values[2]) / 1000.0;
            break;
        case H3LIS331DL_200g:
            accelerationReading[0] = h3lis331dl_from_fs200_to_mg(values[0]) / 1000.0;
            accelerationReading[1] = h3lis331dl_from_fs200_to_mg(values[1]) / 1000.0;
            accelerationReading[2] = h3lis331dl_from_fs200_to_mg(values[2]) / 1000.0;
            break;
        case H3LIS331DL_400g:
            accelerationReading[0] = h3lis331dl_from_fs400_to_mg(values[0]) / 1000.0;
            accelerationReading[1] = h3lis331dl_from_fs400_to_mg(values[1]) / 1000.0;
            accelerationReading[2] = h3lis331dl_from_fs400_to_mg(values[2]) / 1000.0;
            break;
    }
}

//=========================================================================
// getInterrupt1Source
//=========================================================================
// Reads the INT1_SRC register
//
// Parameters: None
// Return:  The current contents of the INT1_SRC register
//==========================================================================
h3lis331dl_int1_src_t H3LIS331::getInterrupt1Source() {
    h3lis331dl_int1_src_t value;
    h3lis331dl_int1_src_get(&sensorAPI_intf, &value);   // H3LIS331 API call
    return value;
}

//=========================================================================
// getInterrupt2Source
//=========================================================================
// Reads the INT2_SRC register
//
// Parameters: None
// Return:  The current contents of the INT2_SRC register
//==========================================================================
h3lis331dl_int2_src_t H3LIS331::getInterrupt2Source() {
    h3lis331dl_int2_src_t value;
    h3lis331dl_int2_src_get(&sensorAPI_intf, &value);   // H3LIS331 API call
    return value;
}

//=========================================================================
// IsInt1Active
//=========================================================================
// Returns true if interrupt 1 is currently active/triggered
//
// Parameters: None
// Return:  True if Interrupt 1 is triggered, False otherwise
//==========================================================================
bool H3LIS331::isInt1Active() {
    h3lis331dl_int1_src_t value;
    h3lis331dl_int1_src_get(&sensorAPI_intf, &value);   // H3LIS331 API call
    if (value.ia) {
        return true;
    } else {
        return false;
    }
}

//=========================================================================
// IsInt2Active
//=========================================================================
// Returns true if interrupt 2 is currently active/triggered
//
// Parameters: None
// Return:  True if Interrupt 2 is triggered, False otherwise
//==========================================================================
bool H3LIS331::isInt2Active() {
    h3lis331dl_int2_src_t value;
    h3lis331dl_int2_src_get(&sensorAPI_intf, &value);   // H3LIS331 API call
    if (value.ia) {
        return true;
    } else {
        return false;
    }
}



//=========================================================================
// spi_read
//=========================================================================
// This function is called by the H3LIS331 API to read data from the sensor.
// First a starting register address is written to the sensor and then the
// data from one or more sensor registers is read back.
// Parameters:
//   spiPtr - A pointer to the SPI bus controller object
//   regAddr - The address of the sensor register from which reading will
//             begin.
//   regData - A pointer to a data buffer where the register data read from
//             the sensor will be stored.
//   len - The number of registers that will be read during the transaction
// Return:  0
//==========================================================================
static int32_t spi_read(void *spiPtr, uint8_t regAddr, uint8_t *regData, uint16_t len)
{
#ifdef LSM6DSO32_DEBUG
    Serial.println("spi_read()");
#endif
    SPIClass *spi = (SPIClass *)spiPtr;
    
    // Begin transaction
    spi->beginTransaction(SPISettings(DEFAULT_SPI_FREQ, MSBFIRST, SPI_MODE0));
    
    // Assert chip select
    digitalWrite(H3LIS331_ChipSelect, LOW);
    
    // Write register address
    spi->transfer(regAddr | 0xC0);  //Sets the R/W and M/S bits
    
    // Read register data
    for (size_t index = 0; index < len; index++) {
        regData[index] = spi->transfer(0xFF);
    }
    
    // Deassert chip select
    digitalWrite(H3LIS331_ChipSelect, HIGH);
    
    // End transaction
    spi->endTransaction();
    
    
    return 0;
}

//=========================================================================
// spi_write
//=========================================================================
// This function is called by the H3LIS331 API to write data to the sensor.
// First a starting register address is written to the sensor and then one
// or more data bytes are written to consecutive registers in the sensor.
// Parameters:
//   spiPtr - A pointer to the SPI bus controller object
//   regAddr - The address of the sensor register that will receive the
//             first byte of data.
//   regData - A pointer to a data buffer where the register data to be
//              written is stored.
//   len - The number of bytes that will be written
// Return:  0
//==========================================================================
static int32_t spi_write(void *spiPtr, uint8_t regAddr, const uint8_t *regData, uint16_t len)
{
#ifdef LSM6DSO32_DEBUG
    Serial.println("spi_write()");
#endif
    SPIClass *spi = (SPIClass *)spiPtr;
    
    // Begin transaction
    spi->beginTransaction(SPISettings(DEFAULT_SPI_FREQ, MSBFIRST, SPI_MODE0));
    
    // Assert chip select
    digitalWrite(H3LIS331_ChipSelect, LOW);
    
    // Write register address
    spi->transfer(regAddr);
    
    // Write the register data
    for (size_t index = 0; index < len; index++) {
        spi->transfer(regData[index]);
    }
    
    // Deassert chip select
    digitalWrite(H3LIS331_ChipSelect, HIGH);
    
    // End transaction
    spi->endTransaction();
    
    return 0;
}
