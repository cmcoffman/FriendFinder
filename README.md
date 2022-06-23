# FriendFinder

FriendFinder is a dead-simple locator beacon and tracker meant for offgrid use. FriendFinders broadcast their location and also recieve the broadcasts of other FriendFinders. Minimally it requires only a GPS module, an IMU (or some kind of compass), and a radio to broadcast with. The current version is using a LoRA radio module, though in theory locations could also be sent through the internet or a LoraWAN system to connect over great distances.

FriendFinder is meant be to a single-use tool which more or less just points towards Friends and indicates how far away they are. I've tested the LoRA radios which can go up to 3 miles in decent not-line-of-sight conditions. In urban conditions range was just under 1 mile. Currently I am using an ESP32 microcontroller and intend eventually for FriendFinders to communicate over the internet though their own WiFi connection, or even via bluetooth to a users phone. Added functinonality via a phone app, such as a map of Friend's locations, the ability to send text messages over the LoRA radio could be added latter. There are already some projects that do this, and FriendFinder isn't meant to replace them. Functionality outside pointing towards friends and indicating their distance will be implemented via a phone app, but the device itself should only point.

# FriendFinder-RTOS

This is a from-scratch rebuild of the FriendFinder to use FreeRTOS on the esp32. I decided to do this because it seems the best way to keep the several concurrent tasks running and using shared data. 

## Overview of tasks
Which tasks their should be and what the boundaries of them are is tricky. Ideally FriendFinder could work with arbitrary combinations of hardware. Ideally the tasks which manage hardware modules (GPS, IMU, Radio, etc) are very simple and thus easily ported to work with other kinds of GPS or IMU module. All they should do is initialize the module and control it's power state, read data from it, and store that data in the shared data resource. Determining when these things should be done is best left to a separate manager task which is abstracted away from the hardware. I have to be careful to not make this too complicated.

* GPS
  * Initialize the GPS module 
  * Read new position data from GPS module
  * Update the current position in a shared block of data
  * Manage the power state of the GPS module to be most efficient\
  * Keep and manage device time to be synced with GPS
    * use PPS pin for ultra-precise timing?

* IMU
  * Initialize the IMU
  * Read data from the IMU
    * Magnetometer
    * Accelerometer
    * Gyroscope
    * Calibration Status
    * Activity Detection from BNO085 IMU
  * Store this info in a shared data object (using queues?)
  * Manage the power state of the IMU
  * Use the IMU's interrupts to manage idle-power states?

* LoRA Radio
  * currently using the [RadioHead](https://github.com/adafruit/RadioHead) library by Mike McCauley
  * Listen for incoming messages and store them in a shared data object
  * broadcast messages of our own location
  * Manage the power state of the radio module
  * Tx and Rx done on agreed schedule (once per minute? per ten minutes?  
    based on distance?
  * Should I use RadioHead's mesh protocol?
  * What about encryption?

* Input
  * Should be little more than a button and maybe a rotary encoder
  * Reads button state and updates shared data resource
  * Light Sensor for display brightness

* Display
  * Initialize the display
  * Pull data from shared resource and generate appropriate views
  * using [Bodmer/TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) generate and update appropriate views
  * manage display's power state
  * manage display brightness based on light-levels

* WiFi/Network
  * Connect to WiFi and manage OTA updates
  * Manage power state of WiFi radio
  * Future
    * set RTC based on NTP when GPS is unavailable
    * Broadcast locations over the internet

* System Manager Task
  * Keep track of when it is time to send or listen for messages
  * Calculate headings and distances based on recieved locations
  * Manage power states of individual modules and the system as a whole
  * Keep track of identities
  * battery monitoring
  * RTOS task management
  * Monitor state of input buttons and set appropriate system modes


