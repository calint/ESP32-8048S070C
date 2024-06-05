## intention
* developing using arduino framework with visual code and platformio
* exploring the devices by developing a toy game
* developing a platform-independent toy game engine featuring:
  - smooth scrolling tile map
  - sprites in layers with pixel precision on-screen collision detection
  - intuitive definition of game objects and logic
  - decent performance

## device ESP32-8040S070C

* display: 800 x 480
* performance: ~15 frames per second

* [purchased at](https://www.aliexpress.com/item/1005004952726089.html)
* [download](http://pan.jczn1688.com/directlink/1/ESP32%20module/7.0inch_ESP32-8048S070.zip)
* [community](https://discord.com/channels/630078152038809611/1199730744424153109)

## development environment
* Visual Code 1.89.1
* PlatformIO 6.1.15
* Espressif 32 (6.7.0) > Espressif ESP32 Dev Module
* esp-idf 4.4.7-dirty
* packages:
  - framework-arduinoespressif32 @ 3.20014.231204 (2.0.14) 
  - tool-esptoolpy @ 1.40501.0 (4.5.1) 
  - toolchain-xtensa-esp32 @ 8.4.0+2021r2-patch5
* dependencies included in `/lib/`:
  - https://github.com/Bodmer/TFT_eSPI/releases/tag/V2.5.43
  - https://github.com/PaulStoffregen/XPT2046_Touchscreen/releases/tag/v1.4
  - https://github.com/TheNitek/XPT2046_Bitbang_Arduino_Library/releases/tag/v2.0.1

