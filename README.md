# FriendFinder

FriendFinder is a dead-simple locator beacon and tracker meant for offgrid use. FriendFinders broadcast their location and also recieve the broadcasts of other FriendFinders. Minimally it requires only a GPS module, an IMU (or some kind of compass), and a radio to broadcast with. The current version is using a LoRA radio module, though in theory locations could also be sent through the internet or a LoraWAN system to connect over great distances.

FriendFinder is meant be to a single-use tool which more or less just points towards Friends and indicates how far away they are. I've tested the LoRA radios which can go up to 3 miles in decent not-line-of-sight conditions. In urban conditions range was just under 1 mile. Currently I am using an ESP32 microcontroller and intend eventually for FriendFinders to communicate over the internet though their own WiFi connection, or even via bluetooth to a users phone. Added functinonality via a phone app, such as a map of Friend's locations, the ability to send text messages over the LoRA radio could be added latter. There are already some projects that do this, and FriendFinder isn't meant to replace them. Functionality outside pointing towards friends and indicating their distance will be implemented via a phone app, but the device itself should only point.


 
