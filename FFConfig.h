#ifndef _FRIENDFINDERCONFIG_
#define _FRIENDFINDERCONFIG_

// Select ONE of the following devices
#include "hardware_setups/FFMK2.h"  // TTGO T-Display
// #include "hardware_setups/FFMK1.h"  // Setup file for mk1 prototype (protoboard)
// #include <hardware_setups/M0_express.h>
// #include <hardware_setups/Feather_32u4.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// #define ESP32_ADDRESS 2
// #define PROTBOARD_ADDRESS 1
// #define BEACON_ADDRESS 3
#define RF95_FREQ 915.0

#define MAC_RED "80:7D:3A:F0:E2:E3"
#define MAC_YELLOW "80:7D:3A:F0:E2:E2"
#define MAC_GREEN "80:7D:3A:BC:D3:A4"
#define MAC_BLUE "80:7D:3A:F0:E2:E4" 

#endif  // Close Library
