XP Flight System

XP Flight System is a custom RC aircraft control platform built around a dedicated handheld transmitter and an STM32-based flight controller. The project combines radio control, real-time telemetry, flight sensing, servo/motor control, battery monitoring, aircraft configuration, and a custom 128×64 OLED transmitter interface.

Main Hardware

Transmitter

Arduino Nano / ATmega328P — reads pilot controls, runs the transmitter UI, stores model settings, and sends control packets.

nRF24L01+ 2.4 GHz radio — bidirectional control link with ACK-payload telemetry from the aircraft.

128×64 SSD1306 OLED — displays battery voltage and percentage, current, link quality, packet count, throttle, flight mode, trims, rates/expo, servo setup, calibration, and system status.

Joysticks, throttle input, potentiometer, switches, and button pads — pilot control inputs and menu navigation.

EEPROM — stores model configuration such as trims, rates, expo, plane type, servo assignment, and servo reversing.

Flight Controller / Receiver

STM32F103C8T6 (Blue Pill) — main flight controller responsible for radio reception, sensor processing, PWM generation, telemetry, failsafe handling, and aircraft mixing.

nRF24L01+ 2.4 GHz radio — receives the 32-byte control packet and returns telemetry to the transmitter.

MPU6050 / compatible IMU — measures acceleration and angular motion for roll, pitch, stabilization, and calibration functions.

BMP180 barometer — measures atmospheric pressure and estimates altitude.

STM32 hardware timers — generate direct PWM signals for servos and the ESC.

Power Monitoring

Battery voltage divider — scales the 3S battery voltage to a safe STM32 ADC range for live voltage telemetry.

LM358N + low-value shunt current sensor — measures motor/system current draw.

Current integration — estimates consumed battery capacity in mAh.

Battery percentage calculation — provides an estimated remaining battery percentage on the transmitter.

Aircraft Control

The system supports configurable aircraft layouts including:

Normal aircraft

Flying wing / elevon aircraft

V-tail aircraft

The transmitter can configure:

Servo-to-output assignment

Individual physical servo reversing

Aileron/elevator rates

Expo

Trims

Flight mode

Gear

Flaps / auxiliary channels

The STM32 applies the received configuration and generates the required servo and motor outputs.

Radio and Telemetry

The radio system uses nRF24L01+ automatic acknowledgement and ACK payloads so control data and telemetry share the same radio link.

Telemetry includes information such as:

Battery voltage

Battery percentage

Current draw

Consumed mAh

Link quality

Packet count

Flight-controller state

Roll and pitch information

Calibration progress

Safety and Reliability Features

Radio-link timeout and failsafe handling

Motor output kept separate from servo reversing

EEPROM model settings

Servo assignment protection

Sensor calibration status

Live link-quality monitoring

Battery monitoring and low-voltage warning support

User Interface

The transmitter uses a custom monochrome OLED interface designed specifically for a 128×64 display. The UI includes a flight dashboard, scrolling system menus, rates/expo setup, trims, aircraft setup, servo assignment, servo reversing, save/reset functions, radio-link status, and calibration screens.

The graphics use compact PROGMEM bitmap assets to keep SRAM usage low on the ATmega328P while still providing a detailed transmitter-style interface.

Project Goal

The goal of XP Flight System is to build a complete RC aircraft electronics platform from the ground up instead of relying on a commercial transmitter or flight controller. It combines embedded firmware, radio communication, sensor processing, power monitoring, aircraft control, telemetry, and user-interface design into one integrated system.
