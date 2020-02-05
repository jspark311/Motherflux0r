# Motherflux0r
A giant mash up of sensors using a Teensy4

Checking things in piece-by-piece. Not likely to ever be directly buildable without proprietary code, but pieces of it are still useful.

### Supported sensors...

    * VEML6075
    * AMG8833
    * BME280
    * TSL2561 (so angry this went EoL... What a great part...)
    * TMP102 (for PSU monitoring)
    * ICM20948
    * A custom magnetometer complex
    * An AdaFruit electret mic breakout with preamp
    * An AdaFruit linearized light sensor

### UI provided by...

    * AdaFruit SSD1331 board
    * My SX8634 capacitive touch board
    * A discrete RGB LED and a vibrator motor

Audio subsystem is being driven by the (most excellent) Teensyduino Audio library and...

    * PCM5051a DAC
    * TI LME dual audio op-amp, pressed into service as a headphone amplifier.

Operation is entirely single-threaded with co-operative scheduling, as best as I could manage it without explicitly building a faculty to do it better. Might use my scheduler class in the future.

I still have pins and flash space left on the Teensy4. And that means I'm not done grafting hardware onto this monster.

----------------------

## Driver refactoring

I reworked drivers for the following sensors to fix architectural or legibility flaws. For most of these, the re-work was so extensive that it almost qualifies as a full rewrite. They are not async yet, but the way is paved for that conversion. I might do a write up on them at a later date. Until then, my notes are in the header files.

    * VEML6075
    * AMG8833
    * BME280
    * TSL2561
    * TMP102


----------------------

#### License

Original code is Apache 2.0.

Code adapted from others' work inherits their license terms, which were preserved in the commentary where it applies.

The `assets` directory contains the source images for icons used in the program. I am under the impression that this is all free clipart.
