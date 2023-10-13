//==============================================================================
// File: H3LIS331.h
// Robert Parker
// 9/27/2023
// Version: 1.0
//
// This file along with H3LIS331.cpp defines a class used for interfacing with
// the H3LIS331 High G, low power 3-axis MEMS accelerometer sensor chip from
// STMicroelectronics.
// This class wraps the STMicroelectronis provided H3LIS331dl C API functions as
// contained in file h3lis331dl_reg.h.
// This implelmenetation supports only a subset of H3LIS331 functionality.
// Excluded functionality includes:
//  1. I2C serial interface is not supported. Only SPI 4-wire mode is supported.
//  2. Wake-to-sleep function not supported.
//  3. High-pass filter not supported. The high pass filter is by passed.
//  4. Reboot of device memory not supported
//==============================================================================
// H3LIS331 C API data types
// typedef enum
// {
//      H3LIS331DL_100g  = 0,
//      H3LIS331DL_200g  = 1,
//      H3LIS331DL_400g  = 3,
//  } h3lis331dl_fs_t;
//
//typedef enum
//  {
//      H3LIS331DL_ODR_OFF   = 0x00,
//      H3LIS331DL_ODR_Hz5   = 0x02,
//      H3LIS331DL_ODR_1Hz   = 0x03,
//      H3LIS331DL_ODR_2Hz   = 0x04,
//      H3LIS331DL_ODR_5Hz   = 0x05,
//      H3LIS331DL_ODR_10Hz  = 0x06,
//      H3LIS331DL_ODR_50Hz  = 0x01,
//      H3LIS331DL_ODR_100Hz = 0x11,
//      H3LIS331DL_ODR_400Hz = 0x21,
//      H3LIS331DL_ODR_1kHz  = 0x31,
//  } h3lis331dl_dr_t;



#ifndef H3LIS331_hpp
#define H3LIS331_hpp

#include <SPI.h>
#include "h3lis331dl-pid/h3lis331dl_reg.h"  // H3LIS331 C API

// Constant Definitions
#define DEFAULT_SPI_FREQ 1000000
#define G_TO_METER_PER_SEC_SQUARED 9.80665F
#define METER_PER_SEC_SQUARED_TO_G 0.101971F

// Uncomment to enable debug messages
#define H3LIS331_DEBUG

// Type Definitions
typedef enum {
    H3LIS331_NONE = 0,
    H3LIS331_X = 1,
    H3LIS331_XY = 3,
    H3LIS331_XZ = 5,
    H3LIS331_XYZ = 7,
    H3LIS331_Y = 2,
    H3LIS331_YZ = 6,
    H3LIS331_Z = 4,
} h3lis331_axis_t;

typedef enum {
    H3LIS331_X_LOW = 1,
    H3LIS331_X_HIGH = 2,
    H3LIS331_Y_LOW = 4,
    H3LIS331_Y_HIGH = 8,
    H3LIS331_Z_LOW = 16,
    H3LIS331_Z_HIGH = 32,
    H3LIS331_ANY_ACTIVE = 64
} h3lis331_interrupt_events_t;

//==============================================================================================
// Class H3LIS331
//==============================================================================================
// A class that wraps the H3LIS331 C API functions provided by STMicroelectronics.
//==============================================================================================

class H3LIS331 {
public:
    //Constructor
    H3LIS331();
    
    //Initialization
    bool begin(uint8_t cs, SPIClass *spiPtr = &SPI, uint32_t spiFrequency = DEFAULT_SPI_FREQ);
    
    //Configuration
    void setRange(h3lis331dl_fs_t range);
    h3lis331dl_fs_t getRange(void);
    void setOutputDataRate(h3lis331dl_dr_t ODR);
    h3lis331dl_dr_t getOutputDataRate(void);
    void enableAxis(h3lis331_axis_t axis);
    h3lis331_axis_t getEnabledAxis(void);
    void setBlockDataUpdate(void);
    void clearBlockDataUpdate(void);
    bool blockDataUpdateEnabled(void);
    
    //Interrupt configuration
    void setInterruptPinsConfig(h3lis331dl_i1_cfg_t pin1Source, h3lis331dl_i2_cfg_t pin2Source,
                                bool latched = false, bool activeHigh = true, bool pushPull = true);
    void interrupt1Config(uint8_t eventsEnabled, bool orEvents,
                          uint8_t threshold, uint8_t duration);
    void interrupt2Config(uint8_t eventsEnabled, bool orEvents,
                          uint8_t threshold, uint8_t duration);
    
    //Accelerometer Data
    h3lis331dl_status_reg_t getStatus(void);
    bool isDataReady(void);
    float getX_Accel(void);
    float getY_Accel(void);
    float getZ_Accel(void);
    void getAcceleration(void);
    h3lis331dl_int1_src_t getInterrupt1Source();
    h3lis331dl_int2_src_t getInterrupt2Source();
    bool isInt1Active();
    bool isInt2Active();
    
    
private:
    bool _init(void);
    stmdev_ctx_t sensorAPI_intf;    // Structure holds pointers to spi read/write functions
                                    // and spi bus object
    h3lis331dl_fs_t accelRange;
    float accelerationReading[3];   // X, Y, and Z axis acceleration in Gs
    
};

#endif /* H3LIS331_hpp */
