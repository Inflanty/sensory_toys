# Sensory Toys

Sensory game for delayed children.

## Getting Started

Project is directed to delayed childrens (in this case cerebral palsy) it developed prioprioception. This repository include software for 
BLE modules & Arduino. Project consist of following parts:

* Docking Station - which include ESP 32 (BLE Client - central), Arduino (Player), MP3 Decoder, Inductive Charges Modules, LEDs, Speaker, Power Supplier. 
* 4 Main toys - each of them include ESP 32 (BLE Server - peripheral), IrDA Module, Inclinometers, Supply part (Battery, Charger, DCDC converter, Inductive Charger module).
* 2 Extension toys - each include Arduino, IrDA Module, Supply part (the same as in the Main toy).

### Main toy - BLE Server

![](docImg/serv.jpg) 

## How it works
### System initialization 

Configuration whole system, no toys connected yet. 

### Connecting...

Start scanning for the BLE devices, "searching" music, blinking LEDs.

### Scanning result

In this case, we have 2 options:

* All devices connected - 4 BLE servers are connected;
* Timeout - if Client will not find all of the servers after 30s from system inicialization -> delay, soft reset.

### Let's play

If All devices are connected -> "All connected" music, connected status on LEDs.

The central unit (BLE Client) reacts to notifications from the BLE Servers. We have 4 options :

* The Main toy is on motion - clarified music is playing (each of the servers have own music, track type depend of the toy construction)
* The Main toy is connected with the extension - clarified music is playing.
* The Main toy is motionless or connectionless (extension) from 3 s. - music is muting.
* Charging - The Main toy is charging via inductive receiver - if BLE Client receive this notification from all BLE Servers, will write standby mode request (low power comsumption).

### Project diagram

![Sensory Toys](docImg/sys.jpg)

## Software

Project based on FreeRT Operation System write in C. The code combines own ideas and espressif support.

## Hardware

### ESP 32

Wifi/Bluetooth module based on Xtensa® single-/dual-core 32-bit microprocessor. In this case used ESP32-DevKitC.

>ESP32 can perform as a complete standalone system or as a slave device to a host MCU, 
>reducing communication stack overhead on the main application processor. 
>ESP32 can interface with other systems to provide Wi-Fi and Bluetooth functionality 
>through its SPI / SDIO or I2C / UART interfaces.

-note from Espressif site.

### Arduino

Arduino Micro, used for uart communication in extension toys (mainly because of compact layout).
Arduino Leonardo for music player implementation.

### Supply part 

* Battery - Lip-Pol 470mAh.
* Charger - for safety battery charging.
* DCDC converter - ESP 32/Arduino supply.
* Inductive charger - for wireless charging.

### Sensor

Inclinometers - for motion detection.

### IrDA module

Complete integrated IrDA module for infrared conncting, require UART communication.


## Overview

The Game based on Sensory Game model discovered from the 1970s, the game affected on prefrontal cortex acivity. 

All intrested on the subject, I refer to articles:

* Journal of Autism and Developmental Disorders, [Vol. 33, No. 6, December 2003](http://hstrial-rfredeen.homestead.com/Article_2003_IngersollSchreibmanTranSensoryFeedbackonObjectImitation.pdf)
* NeuroImage, [29 (2006) 706 – 711](https://s3.amazonaws.com/academia.edu.documents/44775688/Sustained_decrease_in_oxygenated_hemoglo20160415-3191-bpo88k.pdf?AWSAccessKeyId=AKIAIWOWYYGZ2Y53UL3A&Expires=1513380160&Signature=Ph%2Bm9QxMDzUQb24FEDziD38F%2BAs%3D&response-content-disposition=inline%3B%20filename%3DSustained_decrease_in_oxygenated_hemoglo.pdf)
