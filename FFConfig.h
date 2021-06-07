#ifndef _FRIENDFINDERCONFIG_
#define _FRIENDFINDERCONFIG_

#include "FriendFinder.h"

// Select ONE of the following devices
#include "hardware_setups/FFMK3.h"  // prototype on breadboard
//#include "hardware_setups/FFMK2.h"  // TTGO T-Display
// #include "hardware_setups/FFMK1.h"  // Setup file for mk1 prototype
// (protoboard) #include <hardware_setups/M0_express.h> #include
// <hardware_setups/Feather_32u4.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// #define ESP32_ADDRESS 2
// #define PROTBOARD_ADDRESS 1
// #define BEACON_ADDRESS 3
#define RF95_FREQ 915.0

// Array of Known Devices
#define MAC_RED "80:7D:3A:F0:E2:E3"
#define MAC_YELLOW "80:7D:3A:F0:E2:E2"
#define MAC_GREEN "80:7D:3A:BC:D3:A4"
#define MAC_BLUE "24:62:AB:CB:17:00" // T-DISPLAY
#define MAC_PURPLE "80:7D:3A:F0:E2:E4"


// This is the known friends...(be carefull with array size)
// Friend Address is location in array
// String knownMacAddresses[4] = {"80:7D:3A:F0:E2:E3", "80:7D:3A:F0:E2:E2",
//                                "80:7D:3A:BC:D3:A4", "80:7D:3A:F0:E2:E4"};

// uint16_t friendColors[4] = {TFT_RED, TFT_YELLOW, TFT_GREEN, TFT_BLUE};

// #define SKIP_SPACES(p, limit)  \
//       { char *lim = (limit);         \
//         while (p < lim) {            \
//           if (*p++ != ’ ’) {         \
//             p--; break; }}}

// #define KNOWN_FRIENDS \


// typedef struct p
// {
// 	char c;
// 	float x,y;
// }point;

// point P[2]={{'a',1.0,2.0},{'b',3.0,4.0}};

// This is from TFT_eSPI
// #define TFT_BLACK       0x0000      /*   0,   0,   0 */
// #define TFT_NAVY        0x000F      /*   0,   0, 128 */
// #define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
// #define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
// #define TFT_MAROON      0x7800      /* 128,   0,   0 */
// #define TFT_PURPLE      0x780F      /* 128,   0, 128 */
// #define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
// #define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
// #define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
// #define TFT_BLUE        0x001F      /*   0,   0, 255 */
// #define TFT_GREEN       0x07E0      /*   0, 255,   0 */
// #define TFT_CYAN        0x07FF      /*   0, 255, 255 */
// #define TFT_RED         0xF800      /* 255,   0,   0 */
// #define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
// #define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
// #define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
// #define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
// #define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
// #define TFT_PINK        0xFE19      /* 255, 192, 203 */ //Lighter pink, was
// 0xFC9F #define TFT_BROWN       0x9A60      /* 150,  75,   0 */ #define
// TFT_GOLD        0xFEA0      /* 255, 215,   0 */ #define TFT_SILVER 0xC618 /*
// 192, 192, 192 */ #define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
// #define TFT_VIOLET      0x915C      /* 180,  46, 226 */

#endif  // Close Library
