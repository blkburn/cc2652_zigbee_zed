# Zigbee End Device based on the CC2652R1

This is the complete hardware and software design of a zigbee end device (ZED) designed to be run from a same CR2032 button cell battery.

Texas Instruments Code Composer and the Simplelink SDK is used for development; based on the zed_temperaturesensor_CC26X2R1_LAUNCHXL_tirtos_ccs project.

The hardware design replicates the Launchpad-CC26x2R1 development board as much as possible.

The CC2652R1 is an ultra-low power ARM Cortex process with support for a number of wireless protocols, such as; Thread, Zigbee®, Bluetooth® 5.1 Low Energy, IEEE 802.15.4, IPv6-enabled (6LoWPAN), as well as proprietary protocols including TI 15.4-Stack (2.4 GHz), and simultaneous multi-protocol operation via Dynamic Multiprotocol Manager (DMM).

Development baord power consuption is in line with published datasheet power comsumption figures, allowing the zigbee end device to last many months or years on a single CR2032 battery.

The development board can interface various I2C sensors; a pin breakout is provided to interface the many cheap sensors available on-line. An Adafruit 4-pin STEMMA connector is also provide to connect to an I2C device in their range.

A JTAG header is provided to interface XDS110 (or better) debuggers. A CC1352 Launchpad with built-in XDS110 has been used to debug the development boards.

4-pin bootloader



## Master branch is currently built with:
* CCStudio version: 10.1.1.00004 
* Simplelink_cc13x2_26x2_sdk_4_30_00_54


