#define STRINGIFY(s) XSTRINGIFY(s)
#define XSTRINGIFY(s) #s  // So you can do #pragma message ("Compile_Time_Var= " STRINGIFY(Compile_Time_Var)) and see it in the compilier messages

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SSD1306Ascii.h>  // Light weight OLED library with no graphics support. See: https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h"
#include <EEPROM.h>  // Arduino on chip EEPROM (1k byte)  Used for date, time, setup parameters, more, see below.

/**
    @file   Rocket_Altitude_4_6.ino

    @brief  Firmware for the Mia, version 0.0.0 and 0.0.1, Micro Integellient Altimeter, rocket flight computer. Using the BMP581 pressure sensor and ST M24M02E EEPROM.

    @details Micro Integellient Altimete (Mia) is a rocket flight computer with several sensors, serial port interface, and EEPROM data logging.
             Mia was designed for young kids to answer the question "How high did my rocket go?" Mia answers this question with a pressure sensor and a temperature sensor.
             Less than 13mm wide, Mia fits into body tubes as small as BT-5, allowing very small and inexpensive rockets to fly Mia. As Mia evolved, a light sensor, Logging EEPROM,
             an accelerometer, expansion connector, and improved user interface was added. Mia stayed the same dimensions with these additions.
             Beyond just answering that important question "How high did my rocket go?" Mia can help older kids with STEM/STEAM projects or science fair projects.
             The light sensor provides roll rate data, Temperature, as well as improving altitude data can show the lapse rate of temperature with altitude.

             Mia was designed to help young kids stay interested in rocketry from a STEM/STEAM perspective. Flight data is logged allowing rich data for science fair projects.
             Expansion is available through a analog input connector and a QWICC connector. All firmware is open source under the MIT license. Anybody is free to ues this code
             anyway they want, without requirements of giving back, although we would love you to share your additions! :-)
             This firmware uses the BMP581 altimeter, a Vishay VEMT2523SLX01 as a light sensor for possible roll data, a muRata NXFT15XH103FEAB050 thermistor on connector P3 to
             log flight data. Then the OLED will report maximum altitude above ground level in feet.
             The 0.0.1 version adds a I2C KX134ACR accelerometer, charge status to Arduino digital input 6, and a second user button.

             Mia is supported with a host program for Windows, MacOS, Linux, and Raspberry Pi. The host program can:
              Download flight logs
              Clear the log
              Lookup your latitude and longitude and set it in the Mia
              Find the sealevel pressure from NMS over the web and set it
              Set the date
              Read the last maximum altitude
              Setup servo or buzzer feature
              Setup servo settings

*/
// Mia Block Diagram
//                                                            ┌───────────────────────┐
//                                        ┌──────────┐        │ ICSP Test Points for  │
//                                      ┌─┤ 8.00MHz  ├─┐      │programming bootloader │
// ┌────┐                               │ └──────────┘ │      └──────────┳────────────┘
// │    │   ┌──────────┐   Reset     ┌──┴──────────────┴─┐               ┃
// │    │   │          ├────────────▶│                   │◀━━━━━━━━━━━━━━┛
// │    │◀━▶│ FT230XQ  │             │                   │           ┌─────────────────┐
// │    │   │          │   Serial    │                   │◀──────────┤  Light Sensor   │
// │    │   │          │◀━━━━━━━━━━━▶│                   │           └─────────────────┘
// │USB │   └──────────┘             │                   │           ┌─────────────────┐
// │    │   ┌─────────┐  N_BatChg    │                   │◀━━━━━━━━━━┫     Buttons     │
// │    │   │ LTC4063 ├─────────────▶│                   │           └─────────────────┘
// │    ├─┬▶│ Charger │              │                   │           ┌──────────────────────────────┐
// │    │ │ │Regulator│   3.0V       │                   ├──────────▶│   Piezo Alarm/Servo (3.6V)   │
// │    │ │ │         ├──────▶       │                   │           └──────────────────────────────┘
// │    │ │ └────┬────┘              │                   │           ┌─────────────────────────────────┐
// └────┘ │      │                   │    ATMEGA328PB    │◀──────────┤ External Temperature/Analog In  │
//        │      │      3.6V         │                   │           └─────────────────────────────────┘
//        │      ├───────────▶       │                   │           ┌─────────────────────────────────┐
//        │    ──┴──                 │                   ├──────────▶│   High current MOSFET to GND    │
//        │     ─┬─  LiPO Cell       │                   │           └─────────────────────────────────┘
//        │      ▽                   │                   │  Reset    ┌─────────────────┐
//        │                          │                   ├──────────▶│  OLED Display   │
//        │USBVBus                   │                   │           └─────────────────┘
//        └─────────────────────────▶│                   │                     ▲
//                                   │                   │                     ┃
//                                   │                   │  I2C bus            ▼
//                                   │                   ┣━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//                                   └───────────────────┘  ▲             ▲               ▲              ▲
//                                                          ┃             ┃               ┃              ┃
//                                                          ▼             ▼               ▼              ▼
//                                                     ┌──────────┐  ┌──────────┐  ┌──────────────┐  ┌─────────┐
//                                                     │ Logging  │  │  BMP581  │  │   KX134ACR   │  │  QWIIC  │
//                                                     │  EEPROM  │  │ Pressure │  │ Accelerometer│  │Connector│
//                                                     │  256kB   │  │  Sensor  │  │              │  │         │
//                                                     └──────────┘  └──────────┘  └──────────────┘  └─────────┘
//
// Created with Monodraw
/*
            Five modes of operation are supported:
              FlightMode, BatteryChargeOnly, SerialHost, SealevelPressureSetMode, Set descent altitude for high current output
                1) Sea level Pressure Set Mode, using only the OLED and buttons to enter the sea level pressure.
                2) Flight mode, where we wait for lift off, log altitude, light, temperature with a thermistor on connector P3.
                3) Host connected serial utility mode, this will allow downloading of the EEPROM flight log, clearing the EEPROM, and setting the date-time.
                4) Battery charge mode, this will indicate charging for 5 seconds then turn off the OLED.
                5) Set the altitude for the high current output to be turned on.

                While in Flight mode, there are 5 flight phases:
                  1) Initialize
                  2) Wait for launch
                  3) Wait for apogee
                  4) Wait for landing
                  5) landed
                  6) Low power state

            If you are a developer, here is some helpful information
              Source code conventions:
                We use the prefix N_ for active low digital pins.
                Part way through the project we started adding a units suffix to variables to reduce unit confusion.
                Unused MCU pins must be either an output or an input with a pull up or down so the pins don't float and draw excess current.
              Programmer tips:
                1) This software keeps track of approximate date and time. This helps keep flight logs in order. Please keep the #define BIRTH_TIME_OF_THIS_VERSION up to date with every build.
                2) If you are adding software to Mia and need more FLASH space, you can:
                    A) Reduce or eliminate the instructions in DisplayInstructions() function. (Just comment out the call 'DisplayInstructions();', the linker will dead strip.)
                    B) Reduce or eliminate DoSplashScreen() function.
                    C) Reduce or eliminate DoSensorDisplayLoop() function as this is only used for diagnostics.
                    D) Reduce or eliminate the temperature look up table (about 412 bytes) and convert temperature differently, or not at all.
                    E) Remove auto-configuration support for the old Mia 0.0.0
                    F) Remove Buzzer function if not used.
                    G) Remove servo support if not used.
                    H) Revert to the standard ATMEGA328 Variant. You loose access to D23-D26 which may not matter but this saves a few hundred bytes.
                    I) Remove the host mode help command.
                    J) If a buzzer on-off pattern is not required, you can eliminate the DoBuzzer() function.
                3) Internaal EEPROM addresses 232 and up are unused.
                4) Test Point 7 has a hole on the board large enough to easily solder to, it is connected to D3. It is currently unused and available for any purpose.
                5) Many different sensors, Memory, controllers are available with a QWIIC connector that can be used with the Mia.

    @date   5/25/2025

    @note
    @verbatim
        Initial Board Bringup with a blank MCU:

          The first requirement is that you install minicore into the Arduino IDE so you can use the PB version of the ATMEGA328 used on the Mia
            The Arduino bootloader doesn't support the new PB suffix part features.
            Go to https://github.com/MCUdude/MiniCore?tab=readme-ov-file#how-to-install and carefully follow the instructions.

          The second requirement is to have a bootloader programmer.
            In this example we use a Arduino Nano ESP32 as the programmer.
            To make a bootloader programmer you need a host Arduino that is 3.3 volts, a 5V Arduino will damage the pressure sensor and accelerometer.
            We use a Nano ESP32.
            Open the programmer with the ArduinoISP software, in File -> Examples -> 11.ArduinoISP -> ArduinoISP
            Uncomment the line:
            // #define USE_OLD_STYLE_WIRING
            It will be found in the area of lines 81 to 85 depending on your IDE version.

                  Setup the Arduino IDE:     Tools -> Board -> Arduino ESP32 boards ->Arduino Nano ESP32
                                             Tools -> Port ->  MACOS: similar to /dev/cu.usbserial34859F34317C7 (Arduino Nano ESP32)     Windows:

            Now click upload so ArduinoISP is programmed into your Arduino you are using as the programmer (Nano ESP32 in my case).
            The board you just programmed can be kept as a tool to re-program a bootloader that got corrupted.

          The third step is to program the bootloader into your Mia
                  Setup the Arduino IDE:     Tools -> Board -> MiniCore -> ATMEGA328.
                                             Tools -> Clock -> External 8 MHz
                                             Tools -> BOD -> BOD 1.8V      (Brown out detection voltage)
                                             Tools -> Variant -> 328PB
                                             Tools -> Port ->   MACOS: similar to /dev/cu.usbserial-DP05P0VC     Windows:
                                             Tools -> Programmer -> Arduino as ISP or Arduino as ISP (minicore)

            Now hook up your Mia as below

                     Mia to be programmed                                    
                              ┌──────┐                                       
                        ┌─────┤ USB  ├┐ ◀───  Do not hook up USB
                        │     │      ││                                      
                        │     └──────┘│                                      
                        │  ┌───┐      │                                      
                        │  │   │      │                                      
                        │  │P2 │      │                                      
                        │  │   │      │                                      
                        │  └───┘      │                                      
                        │             │                                      
                        │             │                                      
                        │             │                                      
                        │             │                                      
                        │             │                                      
                        │         O   │                    Programmer Board  
                        │   ┌─────┐   │                                      
                        │   │     │  O│◀───  -RESET         D10              
                        │   │ MCU │   │                                      
                        │   │     │  O│◀───  SCK            D13              
                        │   └─────┘   │                                      
                        │            O│◀───  MISO           D12              
                        │             │                                      
                        │            O│◀───  MOSI           D11              
                        │             │                                      
                        │            O│◀───  PLUS3.0V       3.3V             
                        │             │                                      
                        │            O│◀───  GND            GND              
                        │             │                                      
                        │       ┌──┐  │                                      
                        │       │  │  │                                      
                        │       │P1│  │                                      
                        │       │  │  │                                      
                        │ ┌──┐  └──┘  │                                      
                        │ │P4│        │                                      
                        │ └──┘┌───┐   │                                      
                        │     │   │   │                                      
                        │     │P3 │   │                                      
                        │     │   │   │                                      
                        │     │   │   │                                      
                        │     └───┘   │                                      
                        │        ┌┐   │                                      
                        │        ││   │                                      
                        │        ││   │                                      
                        │        ││   │                                      
                        └────────┴┴───┘                                      
            The programmer board powers your Mia. If programming fails, attach a battery to your Mia, turn the ON-OFF switch on, and re-try (USB power will not substitute for this process).
            Now, with your Mia attached to your Arduino programmer board, click Tools -> Burn Bootloader.
            Disconnect your Mia from the programmer.

          The Forth step is to upload Rocket_Altitude_4.6 into your Mia.
            Open Rocket_Altitude_4.6, or the latest version.
            Connect your Mia to your computer with the micro USB connector on the Mia
                      Setup the Arduino IDE: Tools -> Port ->   MACOS: similar to /dev/cu.usbserial-DP05P0VC or similar     Windows:
                        Verify these setting are as below:
                                             Tools -> Board -> MiniCore -> ATMEGA328.
                                             Tools -> Clock -> External 8 MHz
                                             Tools -> BOD -> BOD 1.8V      (Brown out detection voltage)
                                             Tools -> Variant -> 328PB
                                             Tools -> Programmer -> Arduino as ISP or Arduino as ISP (minicore)
            Click upload.
            NOTE: Important! Every time you update your Arduino IDE libraries you need to reset the minicore options above. An Arduino library update seems to reset those selection to some default and your upload will fail.

            After the upload is complete you should see the Mia startup message appear on the OLED. If you don't see anythin on the OLED, unplug the USB, turn the Mia off and re-seat your OLED in its connector. Then turn on again.




      MCU EEPROM Format:
            The MCU EEPROM format contains: <date>,<time>,<user>,<software info>,<sensors available><sensor specific block>
            The on chip EEPROM has a date and time field. These have no way of being current except for getting updated from a host computer
            It is not intended that these are accurate dates, but they are a source of "your data log was ON or AFTER this date"

            ADDR = 0: The time field is not a real time in any sense, but is incremented 10 seconds for each power cycle/flight. This allow the "time stamp" to keep logs in order
                and you know your data log was ON or AFTER the date in the log. After reset this field is check to be after the "born on" date, and a valid number.
                The format is a standard Linux signed time in seconds, but in 64 bit format. The Linux time 0 is midnight UTC on 1 January 1970. This should be as a "TAI" value.
                UTC time is sometimes also refered to as Greenwich Mean Time (GMT) or Coordinated Universal Time.  It is the time at the Royal Observatory in Greenwich, London.
                The EEPROM location supports 64 bits to avoid the year 2038 problem, but the software is not 64 bit ready.
                The Mia host application also updates this time to the current time.

            ADDR = 8: The last Known air pressure at sea level. This is a float.

            ADDR = 12: The next available address in the external EEPROM. uint32_t

            ADDR = 16: This field is a personalization field. The user can put any thing they want here but the intent is for name and contact information.
                This field is 63 characters long with the last byte as a end of string byte.   NOT IMPLEMENTED

            ADDR = 80: This field is a software identification field. The software puts its name, version (from VersionString[]), and release date here.
                This field is up to 63 characters long with the last byte an end of string delimiter (\0).   NOT IMPLEMENTED

            ADDR = 144: The "feature available" field is 32 bits of enable bits for different features, sensors, or sensors with differnet setups.
                Bit  0  BMP390 pressure/temperature sensor
                Bit  1  Optical sensor with VEMT2523SLX01 and 1.00kΩ pull up resistor
                Bit  2  Memsic MC3416 Accelerometer
                Bit  3  Kionix KX134 Accelerometer
                Bit  4  1: 10 kΩ thermistor, muRata NXFT15XH103FEAB050, plugged in P3   0:Record voltage from P3
                Bit  5  BMP581 pressure/temperature sensor
                Bit  6  For output digital 9, SounderRequiresFrequency
                Bit  7  For output digital 9, 0=Servo output, 1=buzzer output

            ADDR = 148: The sensor specific block has the following:   NOT IMPLEMENTED
                (Should we put the BMP581 sample rate, IIR filter coeff & oversampling stuff here? No, not yet.)

                ...
            ADDR = 160: 32 bytes. This 32 byte block is formatted exactly as a logging 32 byte record.
                The first 16 bit word is 0x0001 for its record number. The second 16 bit word is the status, fixed at 0x0100.
                The next 28 bytes are a string for latitude,longitude. Zero terminated allowing 27 bytes of location (including seperating comma). Set with "o" command.
                This is the last known launch location and will be included in the begininning of the next flight log. Format: <latitude>,<longitude>,0x00
                The host application will keep this updated if location services are available, if not, manual entry is available.

            ADDR = 192: Altitude setting for high current output. This is a float.

            ADDR = 196: Last maximum altitude reached in meters. This altitude is already be corrected for field altitude. This is a float.
                Read with "a" command.

            ADDR - 200: Servo position, pre-launch. This is a byte. Any value > 180 is invalid.
                Read and set with "p" command

            ADDR - 201: Servo position, ascent. This is a byte. Any value > 180 is invalid.
                Read and set with "p" command

            ADDR - 202: Servo position, at apogee. This is a byte. Any value > 180 is invalid.
                Read and set with "p" command

            ADDR - 203: Servo position, descent. This is a byte. Any value > 180 is invalid.
                Read and set with "p" command

            ADDR - 204: Servo position, post landing. This is a byte. Any value > 180 is invalid.
                Read and set with "p" command

            ADDR - 205: Servo position, low power mode. This is a byte. Any value > 180 is invalid.
                Read and set with "p" command

            ADDR - 206: Low power buzzer rest multiplier. This is a byte. This is a factor to extend the time between buzzer sounds. 0xff is invalid.
                Read and set with "p" command

            ADDR - 207: Delay time from landing to Low power mode. This is a byte. The byte value is multiplied by 10 to get seconds of delay. 0xff is invalid. (42.5 minute max delay)
                Read and set with "p" command.
            
            ADDR - 208: Servo position, When the high current output goes below its altitude setting. This is a byte. Any value > 180 is invalid.
                Read and set with "p" command.
            
            (Space left for more byte parameters that the "p" command can set. p command writes 16 values from 200 through 215)

            ADDR - 216: Frequency 1 for the buzzer. Maximum frequency allowed is 12000 Hz.
                Read and set with "w" command

            ADDR - 218: Frequency 2 for the buzzer Maximum frequency allowed is 12000 Hz.
                Read and set with "w" command

            ADDR - 220: Milliseconds between samples during ascent and apogee
                Read and set with "w" command

            ADDR - 222: Milliseconds between samples after [apogee plus 'ServoApogeeDuration_ms'] through landing
                Read and set with "w" command

            (Space left for more word parameters that the "w" command can set. w command writes eight 16 bit values from 216 through 231)


            Sketch uses 31826 bytes (98%) of program storage space. Maximum is 32384 bytes.   This is a lot, see programmer tips above.
    @endverbatim

    @author Rich Rau with additions by Bob Rau

    @bug    We are doing testing but we don't have a formal validation test suite yet.


  By: Rich Rau and Robert Rau
  Changes: OK, time to start a change log.
          preparing to add acceleromter support, need more space:
          Removed Times font and initial Mia is now in default font (saved 100s of bytes)
          Turned some repeated code into functions
          Modified EEPROM library to remove error reporting printouts (there is no one in space to read them!)
          Fixed:
          Hold USER1 Button displayed as Hold USER1 Butt, now shows Btn
          No longer shows display while still in flight. Now waits for landing.

  Updated: 12/13/2024
  Rev.: 4.1.0
  By: Rich Rau and Robert Rau
  Changes: Start of support for latitude and longitude.

  Updated: 12/15/2024
  Rev.: 4.1.1
  By: Rich Rau and Robert Rau
  Changes: Serial dump now includes lat and lon. Updated comments. Fixed bug in writing queued measurement records at launch.

  Updated: 1/1/2025
  Rev.: 4.1.2
  By: Rich Rau and Robert Rau
  Changes: USBPowered() fixed to not use floating point (saved 24 bytes)
  
  Updated: 2/2/2025
  Rev.: 4.1.3
  By: Robert Rau
  Changes: Adding and fixing comments. Fixed up Host mode UI. Fixed logging EEPROM lib I broke. compacted USBPowered().
  
  Updated: 2/2/2025
  Rev.: 4.1.4
  By: Robert Rau
  Changes: Provided exit from host mode to flight mode without elplicitly exiting host mode (pulling the power now puts you in flight mode)
  
  Updated: 2/2/2025
  Rev.: 4.1.5
  By: Robert Rau
  Changes: IN PROCESS   fixing logging EEPROM write (lib fixed)
  
  Updated: 3/10/2025
  Rev.: 4.2.0
  By: Robert Rau
  Changes: Added accelerometer
  
  Updated: 3/10/2025
  Rev.: 4.2.01
  By: Robert Rau
  Changes: cleaned up debug stuff
  
  Updated: 4/5/2025
  Rev.: 4.2.2
  By: Robert Rau
  Changes: Removed external EEPROM library & included custom functions in .ino file.
  
  Updated: 4/6/2025
  Rev.: 4.2.3
  By: Robert Rau
  Changes: Added Mia version auto configuration. Comment clean up.
  
  Updated: 4/8/2025
  Rev.: 4.2.4
  By: Robert Rau
  Changes: Added Altitude setting for firing high current output at descent altitude. Also high current output on descent lower than threshold altitude.
  
  Updated: 4/25/2025
  Rev.: 4.2.5
  By: Robert Rau
  Changes: Fixed comments
  
  Updated: 4/28/2025
  Rev.: 4.5.0
  By: Robert Rau
  Changes: First attempt at integrating the BMP581 pressure sensor
  
  Updated: 5/1/2025
  Rev.: 4.5.1
  By: Robert Rau
  Changes: Comment clean up. "a" command.
  
  Updated: 5/3/2025
  Rev.: 4.5.2
  By: Robert Rau
  Changes: Added servo support and low power mode. 
  
  Updated: 5/4/2025
  Rev.: 4.5.3
  By: Robert Rau
  Changes: EEPROM parameter initialization. Comment cleanup. Fixed meter and feet confusion in high current altitude threshold. Fixed high current alt save. Started p, w, and u commands.
  
  Updated: 5/7/2025
  Rev.: 4.5.4
  By: Robert Rau
  Changes: Changedd startup to look for '~' so computer connections start up faster. debugging. Adding servo position for High Current altitude crossing

  Updated: 5/8/2025
  Rev.: 4.5.5
  By: Robert Rau
  Changes: Fixing servo-sounder bug. Speeded up boot time. My BMP581 functions have doubled the sample rate of the Mia, I had to change the BMP581 OSR from x32 to x16 to keep up.

  Updated: 5/15/2025
  Rev.: 4.5.6
  By: Robert Rau
  Changes: Moved user configuration report to Sensor display loop

  Updated: 5/16/2025
  Rev.: 4.5.7
  By: Robert Rau
  Changes: Updated flight mode comments

  Updated: 5/17/2025
  Rev.: 4.5.8
  By: Robert Rau
  Changes: User configuration word setup clean up.

  Updated: 5/18/2025
  Rev.: 4.5.9
  By: Robert Rau
  Changes: Sensor loop optimized. User configuration word setup clean up and redundancy removed. Flight mode picture comment revised.

  Updated: 5/22/2025
  Rev.: 4.5.10
  By: Robert Rau
  Changes: Added #define for apogee detect, changed from 15 to 5.

  Updated: 5/22/2025
  Rev.: 4.6.0
  By: Robert Rau
  Changes: Started version 4.6 branch for 2Mb logging EEPROM

  Updated: 5/24/2025
  Rev.: 4.6.1
  By: Robert Rau
  Changes: Integrated 2Mb logging EEPROM function calls. Added last altitude display on startup.

  Updated: 5/24/2025
  Rev.: 4.6.2
  By: Robert Rau
  Changes: Added external EEPROM setup. Fixed Instructions? display. Comment cleanup. Servo out doesn't work. Low power sounder has bug, times are treated as relative but they are absolute.

  Updated: 5/26/2025
  Rev.: 4.6.3
  By: Robert Rau
  Changes: Fixed servo. Comment cleanup.

  Updated: 5/27/2025
  Rev.: 4.6.4
  By: Robert Rau
  Changes: Changed BuzzerSchedule to be duration based instead of absolute time based, this makes implementing the lowpower mode eaiser. Fixed init of buzzer cycle if flown again without reset.
  Fixed logging of high current output and TP7. Changed apogee detection, APOGEE_DESCENT_THRESHOLD to 4 meters. Added sample rate throttel fixed at 25ms.

  Updated: 5/27/2025
  Rev.: 4.6.5
  By: Robert Rau
  Changes: Fixed (again) log rate throttling.

  Updated: 5/29/2025
  Rev.: 4.6.6
  By: Robert Rau
  Changes: Fixed sounder setup in SetUpMiaFromMCUEEPROM(). Fixed bug in PopulatePreLaunchQueueFlightRecord() with accelerometer data. Added variable sample rate.
           Finished w command.

  Updated: 5/31/2025
  Rev.: 4.6.7
  By: Robert Rau
  Changes: Fixed order of words in 'w' command

  Updated: 6/1/2025
  Rev.: 4.6.8
  By: Robert Rau
  Changes: Changed launch detect to two succesive increases in altitude greater that a specific value. Comment cleanup.

  Updated: 6/2/2025
  Rev.: 4.6.9
  By: Robert Rau
  Changes: Fixed buzzer at landing. Added temperature-voltage logic. Removed external EEPROM debug printouts.

  Updated: 6/3/2025
  Rev.: 4.6.10
  By: Robert Rau
  Changes: Fixed several P3/P4 typos in defines and comments. Fixed and added comments. Removed some unnecessary characters in some printouts.
           Speeded up EEPROM write so our sample rate is over 80Hz. Made BMP581 setup adapt to our faster sample rate.

  Updated: 6/5/2025
  Rev.: 4.6.11
  By: Robert Rau
  Changes: Added function WriteRecordAtSamplePeriod() so buttons would still get sampled often enough during flight phases (required if a flight doesn't terminate correctly).
           This function also checks for full EEPROM. Comment clean up. Bug fix in 'l' command if flight records only had initial records.

  Updated: 6/7/2025
  Rev.: 4.6.12
  By: Robert Rau
  Changes: Fixed record index issues with new sample rate feature. Fixed delays with initial record writes to external EEPROM. Fixed lat & log loosing their string delimiter in copy from MCU
             EEPROM to flight record. Fixed missing landing record with force write parameter. Fixed last Max Altitude bug.

  Updated: 6/8/2025
  Rev.: 4.6.13
  By: Robert Rau
  Changes: Shortened "missing landing record" message in 'd' command. Simplified SkipOLEDSplashInfo detection. Shortened prelaunch queue to 2 samples to reduce 125ms delay of logging at launch.

  Updated: 6/9/2025
  Rev.: 4.6.14
  By: Robert Rau
  Changes: Fixed sign error on accelerometer Z axis log data. Added Logging EEPROM record format comments at bottom of file. Fixed FlightStatus bug where flight phase information was getting masked.
            Comment cleanup. Fixed apogee detect so it shows up in the log on time. More cleanup in transition to flight mode. FlightStatus cleanup.

  Updated: 6/10/2025
  Rev.: 4.6.15
  By: Robert Rau
  Changes: Fixed FlightStatus logging.
 

*/
// Version
const char VersionString[] = "4.6.15\0";       //  ToDo, put in flash  see: https://arduino.stackexchange.com/questions/54891/best-practice-to-declare-a-static-text-and-save-memory
#define BIRTH_TIME_OF_THIS_VERSION 1749578131  //  Seconds from Linux Epoch. Used as default time in MCU EEPROM.
//                                                 I get this from https://www.unixtimestamp.com/  click on Copy, and paste it here. Used in MCUEEPROMTimeCheck()



//**************************************************************************************************************************************
// Constants
//**************************************************************************************************************************************

// worldly constants
#define SECONDS_TO_MILLISECONDS 1000
#define SECONDS_TO_MICROSECONDS 1000000
#define SECONDS_TO_NANOSECONDS 1000000000
#define NANOSECONDS_TO_MILLISECONDS 0.000001
#define NANOSECONDS_TO_SECONDS 0.000000001
#define MHz 1000000                                  //  MHz to Hz
const float METERS_TO_FEET = 3.280839895;            //  Conversion from meters to feet, from https://www.rapidtables.com/convert/length/meter-to-feet.html
#define AvogadrosNumber 6.0221408e+23                //  Because I could!
#define MOLAR_MASS_AIR_kgpm 0.0289644                //  Molar mass of Earth’s air [kg/mol].
#define GRAVITATIONAL_ACCELERATION_ms2 9.80665       //  Gravitational acceleration constant m/s^2.
#define UNIVERSAL_GAS_CONSTANT_jkm 8.31446261815324  //  Universal gas constant in joules per kelvin (K) per mole.
#define SPEED_OF_LIGHT_VAUUM_mps 299792458           //  Speed of light in a vacuum [m/s].
#define COULOMB_ec 6.241509e+18                      //  1 Coulomb equals 6.241509e18 elementary charges (charge of one proton)



/**************************************************************************************************
   Arduino pinout
 **************************************************************************************************/
/* DIGITAL PINS */
//  D0 is used as RxD for serial communications with host
//  D1 is used as TxD for serial communications with host
#define SERIAL_BAUD 1000000  //  1 megabuad for Host communications.
#define N_DispButton 2       //  Active low USER1 button input, must enable pull up resistor.
#define TestPoint7 3         //  Just a solder pad.  Must enable pull up resistor or make an output.
#define UnusedD4 4
#define N_OLEDReset 5        //  Active low reset pulse to OLED display, minimum reset pulse width 3 microseconds.
#define N_BatteryCharging 6  //  Connected to the -CHG signal on Mia board version 0.0.1
#define HighCurrentOut 7     //  Active high enable to high current pull down MOSFET on connector P4.
#define UnusedD8 8
#define BuzzerOut 9  //  Pin for square wave signal to Pizo sounder or servo PWM. Nominal resonant frequency is 4100 Hz for the CMT-1102-SMT Piezo.
#define UnusedD10 10
#define UnusedD11 11  //  Used for bootloader programming
#define UnusedD12 12  //  Used for bootloader programming
#define UnusedD13 13  //  Used for bootloader programming
#define UnusedD23 23  //  328PB port E bit 0, input pulled high. Will read as a 1 for first version of Mia, read as a 0 for the second version (with accelerometer). This bit is a MiniCore feature.
#define UnusedD24 24  //  328PB port E bit 1, unused, input pulled high so it won't float. This bit is a MiniCore feature.
#define UnusedD25 25  //  328PB port E bit 2, unused, input pulled high so it won't float. This bit is a MiniCore feature.
#define UnusedD26 26  //  328PB port E bit 3, unused, input pulled high so it won't float. This bit is a MiniCore feature.

/* ANALOG PINS */
#define AnalogInputP3 A0  // Analog input on connector P3. Direct connection from P3 to the A/D input with optional R14 10k ohm 0.1% pull up for a 10k ohm thermistor on P3.
#define LightSensor A1    // Vishay VEMT2523SLX01 optical sensor. Angle of sensitivity is +/- 35 degrees. Visible light through infrared. More radiation is a lower voltage.
#define N_Button2 A2      // Used as digital input. Second user button, USER2, only on second version of Mia. MCU port bit PC2. Must enable pull up resistor.
#define USBVbus A3        // This pin sees half the Vusb voltage. If Vusb is over 3.7 volts means we are either connected to a power bank for charging or a host computer.
//  A4 is used as SDA on the I2C bus
//  A5 is used as SCL on the I2C bus


/**************************************************************************************************
   Mia constants
 **************************************************************************************************/


//  CPU
//#define ArduinoClockFrequency 8000000UL   // use F_CPU to get frequency.
// Mia is kind of like an "Arduino Pro Mini 3.3V - 8MHz" equivlant-ish, but with an ATMEGA328PB, altimeter, accelerometer, light sensor, external analog input,
//    OLED display, high current output, external EEPROM, Buzzer/servo output, two buttons, USB to serial bridge, QWIIC connector, and a battery charger.
// Use minicore so you get the ATMEGA328PB features. See https://github.com/MCUdude/MiniCore

//Audio - buzzer
#define BuzzerFrequency1 4100  // Hz, this should be the maximum resonant point. Nominal resonant frequency is 4100 Hz for the CMT-1102-SMT Piezo.
#define BuzzerFrequency2 2700  // Hz, this should be the next lower resonant point
#define INTER_BUZZ_DELAY_ms 500
uint8_t SounderRequiresFrequency;
//std::array<uint16_t> BuzzerSchedule[8] = {
uint16_t BuzzerSchedule[8] = {
  // this is an array of durations (in milliseconds) and frequencies (in Hz). Index 2 and 6 must be durations of 'no tone'. This is required for low power mode to know which delays to increase.
  1500, BuzzerFrequency1,  // Do BuzzerFrequency1 until 1500ms
  INTER_BUZZ_DELAY_ms, 0,  // A half second gap
  1500, BuzzerFrequency2,  // 1.5 seconds
  INTER_BUZZ_DELAY_ms, 0   // A half second gap
};                         // then repeat

uint8_t ServoNotSounder;
uint32_t LowPowerSounderMultiplier;
uint8_t Mult;

Servo MiaServo;  // Create our servo object to optionally control a servo.
typedef union {
  unsigned long ApogeeTime_ms;
  unsigned long LandingTime_ms;
} FlightTimeStamps;
FlightTimeStamps OurFlightTimeStamps;
//bool ApogeeDetected = false;
uint8_t ServoFlightStateArray[7] = { 90, 90, 90, 90, 90, 90, 90 };  // Values must be from 0 to 180. 0:Waiting for launch 1:Ascent 2:Apogee 3:Descent 4:Landed 5:Low power 6:@ high current altitude
uint8_t ServoState;
#define ServoWaitingForLaunch_index 0
#define ServoAscent_index 1
#define ServoApogee_index 2
#define ServoDescent_index 3
#define ServoLanded_index 4
#define ServoLowPower_index 5
#define ServoHighCurrent_index 6
#define ServoApogeeDuration_ms 600  //  The servo will go to position index 2 at apogee and stay there for ServoApogeeDuration_ms milliseconds, then go to position index ServoDescent_index

// Analog channels
#define USBToTenthsVolts 60 / 1023  // This will convert an A/D reading of Vusb to a number ten times the volts. NO parenthesis, we want the multiply by the 60 first, if integer math.

// I2C Addresses
#define I2C_OLED_ADDRESS 0x3C  // 0x3C address for 128x32 OLED
//  External EEPROM is at 0x50 and 0x51
#define I2C_BMP581_ADDRESS 0x47                      // 0x47 address for BMP581
#define I2C_MC3416_ADDRESS 0x4C                      // 0x4C address for accelerometer option 1 (never tested)
#define I2C_KX134_ADDRESS 0x1E                       // 0x1E address for accelerometer option 2 (KX134ACR NOT KX134-1211, software not compatible)
#define I2C_ACCELEROMETER_ADDRESS I2C_KX134_ADDRESS  // Address for selected accelerometer


//  OLED stuff
SSD1306AsciiWire display;

// Time setup
#define MCU_EEPROM_ADDR_DEFAULT_TIME_s 0  // MCU EEPROM address of UNIX 64 bit time.
int64_t EEPROMTime;                       //  Signed 64 bit time.  (current software is just dealing with 32 bit time)

// Sample rate
uint16_t SamplePeriod_ms = 25;
uint32_t TimeStamp;
#define SAMPLE_PERIOD_PRE_APOGEE_DEFAULT_ms 25
#define SAMPLE_PERIOD_POST_APOGEE_DEFAULT_ms 50
//uint32_t LastRecordTimeStamp_ms;

// Pressure Altimeter setup
#define DEFAULT_SEALEVELPRESSURE_hPa (1013.25)         //  The International Standard Atmosphere defines standard sealevel pressure as 1013.25 hectopascal (hPa) or millibars (mb).
#define MINIMUM_SEA_LEVEL_PRESSURE_hPa (950)           //  in hectopascal (hPa) (millibars). Minimum pressure allowed for sea level.
#define MAXIMUM_SEA_LEVEL_PRESSURE_hPa (1060)          //  in hectopascal (hPa) (millibars). Maximum pressure allowed for sea level.
#define APOGEE_DESCENT_THRESHOLD 2.0                   //  We must be below maximum altitude by this value to detect we have passed apogee.
#define START_LOGGING_ALTITUDE_m 0.6                   //  Altitude threshold that we must exceed before starting logging to EEPROM.
#define MCU_EEPROM_ADDR_DEFAULT_SEALEVELPRESSURE_HP 8  //  MCU EEPROM address where sealevel pressure is stored.
float SeaLevelPressure_hPa;                            //  user adjusted sea level pressure in hectopascal (hPa) (millibars).
float fieldAltitude_m = 0.0;                           //  Launch field altitude above sea level in sensor units (meters)
float newAltitude_m = 0.0;                             //  Latest new altitude above sea level in sensor units (meters)
float maxAltitude_m = 0.0;                             //  Latest new higher altitude above sea level in meters
float LastDisplayedAltitude_m;                         //  Used to avoid re-writing the same altitude to the OLED, (above sea level)
bool SeaLevelPressureSetUpDirection = true;            //  Used for both setting sealevel pressure and altitude threshold for the high current output. True for UP and False for DOWN.
uint8_t FlightModePhaseIndex;                          //  used in flight mode to keep track of the phase of the flight
const float InvalidAltitude = -10000.0;
//float CurrentAltitude4Ago_m;  // used for pre-launch queue, apogee detection
float CurrentAltitude1Ago_m;  // used for pre-launch queue, apogee detection, and landing detection
float CurrentAltitude2Ago_m;  // used for pre-launch queue, apogee detection, and landing detection
float CurrentAltitude3Ago_m;  // used for pre-launch queue, apogee detection, and landing detection
uint32_t TimeStamp1Ago;       // used for pre-launch queue
uint32_t TimeStamp2Ago;       // used for pre-launch queue
uint32_t TimeStamp3Ago;       // used for pre-launch queue
float AltitudeHighCurrentOutSetting_ft;
float AltitudeHighCurrentOutSetting_m;
float CurrentPressure;  // used for lowest level functions
//  Pressure from BMP581 as a array of bytes from the I2C read, or as a 26 bit . 6 bit fraction in Pascals
typedef union {
  byte PressureBytes[4];
  uint32_t PressureFraction;  //  ASSUMING LITTEL END-IAN !!!!!!!
} BMP581Pressure;
BMP581Pressure LatestPressure;

//  High current output altitudes
#define DEFAULT_ALTITUDE_ft 500.0   //  default altitude to trip high current output, 800ft. above field altitude. In feet.
#define MAXIMUM_ALTITUDE_ft 5000.0  //  Maximum altitude allowed for high current output, 5000ft. above field altitude. In feet.
#define MINIMUM_ALTITUDE_ft 300.0   //  Minimum altitude allowed for high current output, 300ft. above field altitude. In feet.

// Internal MCU EEPROM stuff
#define MCU_EEPROM_USER_CONFIGURATION 144  //  location in the MCU EEPROM where the user feature configuration 32 bit word is
// UserConfiguration bits
//                Bit  0  BMP390 pressure/temperature sensor              <-- NOT USING, NOT AN OPTION ANYMORE, 0
//                Bit  1  Optical sensor with VEMT2523SLX01 and 1.00kΩ pull up resistor  <-- SOLDERED ON THE BOARD, NOT AN OPTION ANYMORE, 1
//                Bit  2  Memsic MC3416 Accelerometer                     <-- NO INTENTION TO USE, NOT AN OPTION ANYMORE, 0
//                Bit  3  Kionix KX134 Accelerometer                      <-- OPTION, autoconfigured, 0
//                Bit  4  10 kΩ thermistor, muRata NXFT15XH103FEAB050, plugged in P3 <--OPTION, default ON, 1
//                Bit  5  BMP581 pressure/temperature sensor               <-- SOLDERED ON THE BOARD, NOT AN OPTION ANYMORE, 1
//                Bit  6  For output digital 9, SounderRequiresFrequency   <--OPTION, default ON, 1
//                Bit  7  For output digital 9, 0=Servo output, 1=buzzer output  <--OPTION, default buzzer, 0
uint32_t UserConfiguration;
#define UserConfigurationRequiredFeatures 0x00000022UL             //Current production board build with defaults.
#define UserConfigurationDefaultFeatures 0x00000072UL              //Current production board build with defaults.
#define UserConfigurationHasRohmKX134Accelerometer 0x00000008UL    // KX134 mask
#define UserConfigurationP3ThermistorNotVoltage_mask 0x00000010UL  // P3 temperature/voltage mask
#define UserConfigurationSounderRequiresFrequency 0x00000040UL     // SounderRequiresFrequency mask
#define UserConfigurationSounderNotServo_mask 0x00000080UL         // Servo mask
#define MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft 192              //  AltitudeHighCurrentOut is unique in that it is stored in feet.
#define MCU_EEPROM_LAST_MAXIMUM_ALTITUDE 196                       // where the last flight's maximum altitude is stored. This value is already compensated for field altitude. In meters.
#define MCU_EEPROM_SERVO_PRE_LAUNCH 200
#define MCU_EEPROM_SERVO_ASCENT 201
#define MCU_EEPROM_SERVO_AT_APOGEE 202
#define MCU_EEPROM_SERVO_DESCENT 203
#define MCU_EEPROM_SERVO_POST_LANDING 204
#define MCU_EEPROM_SERVO_POST_LANDING_LOW_PWR 205
#define MCU_EEPROM_SERVO_HC_Out 208

#define MCU_EEPROM_LOW_POWER_BUZ_REST_MULT 206
#define LOW_POWER_SLOWDOWN_MULTIPLIER_PER_COUNT 10
#define MCU_EEPROM_DELAY_FROM_LANDING_TO_LOW_POWER_MODE 207
uint32_t DelayToLowPower_ms;
#define LowPowerBuzzerRestMultiplierDefault 0x06

#define MCU_EEPROM_BUZZER_1_FREQ 216
#define MCU_EEPROM_BUZZER_2_FREQ 218

#define MCU_EEPROM_SAMPLE_RATE_PRE_APOGEE 220
#define MCU_EEPROM_SAMPLE_RATE_POST_APOGEE 222


//  External EEPROM stuff
#define ExternalEEPROMSizeInBytes 262144U       //    262144 Bytes or 8192 flight records
#define MCU_EEPROM_EXT_EEPROM_ADDR_START 12     // Location in the MCU EEPROM where the next free address is in the external EEPROM
uint32_t EepromAddress;                         // Address within the external EEPROM
#define MCU_EEPROM_ADDR_LATITUDE_LONGITUDE 160  // Last known launch location. 24 byte text string <latitude>,<longitude>,0x00
uint16_t RecordNumber;

//  This type, Measurement, defines the format of every time-stamped sensor measurement record (pre and post launch) for the external EEPROM. See the bottom of this file for format details.
typedef union {
  struct {                     // *** This structure is for the in flight data records
    uint16_t RecordIndex;      // This is a index of each record per flight
    uint16_t Status;           // See the bottom of this file for format details.
    uint32_t current_time_ms;  // This is time from when the Mia is turned on, mission elapsed time. In milliseconds. This is not related to the Linux time stamp at MCU EEPROM addr 0.
    float Altitude;            //  Altitude above Field altitude in meters.
    float Temperature;         //  Using analog connector P3.
    uint32_t LightVoltage;
    float AccelerationX_g;  //  Not on first version of Mia.
    float AccelerationY_g;  //  Not on first version of Mia.
    float AccelerationZ_g;  //  Not on first version of Mia.
  } Record;
  byte Bytes[sizeof(Record)];  // *** This structure is for writing the records to the external EEPROM
  struct {                     // *** This structure is for the initial record before each flight.
    uint16_t RecordIndex;      // This is a index of each record per flight, for this initial record format it will be 0
    uint16_t Status;           // See the bottom of this file for format details.
    uint32_t current_time_ms;  // This is time from when the Mia is turned on, mission elapsed time. In milliseconds. This is not related to the Linux time stamp at MCU EEPROM addr 0.
    float Altitude;            //  Field altitude in meters.
    float Temperature;         //  Using analog connector P3.
    uint32_t SeaLevelPressureX4;
    int64_t LinuxDateTime;
    uint32_t spare;
  } InitRecord;
} Measurement;

Measurement current_measurement;  // This struct is populated for every pre-flight/in-flight logging EEPROM record.
uint16_t FlightStatus;

float P3Voltage_mV;             // voltage on P3 connector
uint8_t TemperatureNotVoltage;  // 1: Temperature, 0: Voltage

//unsigned long buzzTime;
unsigned long landAltBuzzTime;

enum AllOperationalModes { FlightMode,
                           BatteryChargeOnly,
                           SerialHost,
                           SealevelPressureSetMode,
                           AltitudeForHighCurrentOutput };  // Mia has 5 modes controlled by: button presses, USB power, & serial communications.
AllOperationalModes OperationalMode;


//  Host mode support
const byte numChars = 38;  // 38 characters (2 extra)
//  Example lat-log set command: o 44.699810,-116.086600<cr><0x00>     25 characters through delimiter
//  Example byte set command: p aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa<cr><0x00>   36 characters through delimiter
char receivedChars[numChars];  // an array to store the received data
uint32_t Argument1_i;          // this will need to be changed to long long before the year 2038 for the UNIX time.
char CommandCharacter;
bool newData = false;
uint8_t ReceiveBufferIndex = 0;
int i;

//void(* resetFunc) (void) = 0;   //declare reset function at address 0   CHECK THAT YOUR TARGET PROCESSOR RESETS TO ADDRESS ZERO!!!

// Mia version management
bool HasAccelerometer = false;
bool HasUser2Button = false;
//boolean HasChargeInput = false;    //  Not used in this version

int User1ButtonAfterReset;

/**********************************************************************************************************************************************
   Arduino setup
 **********************************************************************************************************************************************/
void setup() {

  pinMode(N_DispButton, INPUT_PULLUP);
  User1ButtonAfterReset = digitalRead(N_DispButton);  //  record this button state just after reset to see if we will do the display sensor loop
  // Setup serial communications. Should we only do this if connected to host?
  Serial.begin(SERIAL_BAUD);  // Start serial communication with baud rate of 1000000 (1 megabaud)
                              //while (!Serial)
  ;                           // wait for native usb.  Not needed on Mia?

  // Setup used pins
  digitalWrite(HighCurrentOut, LOW);  // High current output on P4 connector.
  pinMode(HighCurrentOut, OUTPUT);    // High current output
  //  pinMode(N_DispButton, INPUT_PULLUP);   // done right after reset
  pinMode(N_Button2, INPUT_PULLUP);  //  USER2 button (only on Mia 0.01 and later)
  pinMode(BuzzerOut, OUTPUT);
  pinMode(AnalogInputP3, INPUT);  //  If R14 is not populated and the analog input is not used this should be an INPUT_PULLUP
  pinMode(LightSensor, INPUT);
  pinMode(USBVbus, INPUT);
  pinMode(UnusedD23, INPUT_PULLUP);  //   328PB only, Will read as a 1 for Mia 0.0.0 (was unused). read as a 0 for Mia 0.01.

  // Setup unused pins
  pinMode(N_BatteryCharging, INPUT_PULLUP);  //  Mia 0.01 version only. As charging pin the pull up must NOT be used.
  pinMode(UnusedD4, INPUT_PULLUP);
  pinMode(UnusedD8, INPUT_PULLUP);
  pinMode(UnusedD10, INPUT_PULLUP);
  pinMode(UnusedD11, INPUT_PULLUP);
  pinMode(UnusedD12, INPUT_PULLUP);
  pinMode(UnusedD13, INPUT_PULLUP);
  pinMode(TestPoint7, OUTPUT);       //   Debug or user expansion output
  pinMode(UnusedD24, INPUT_PULLUP);  //   328PB only, Will read as a 1
  pinMode(UnusedD25, INPUT_PULLUP);  //   328PB only, Will read as a 1
  pinMode(UnusedD26, INPUT_PULLUP);  //   328PB only, Will read as a 1

  digitalWrite(TestPoint7, LOW);

  Wire.begin();
  Wire.setClock(400000L);

  M24M02E_Setup();

  // set board features...
  // ... first, as required by PCB version...
  if (digitalRead(UnusedD23) == LOW) {  // The MiniCore bootloader is required to access D23
    //  Mia 0.0.1
    HasUser2Button = true;
    //HasChargeInput = true;  // Digital input 6.      //  Not used in this version.
  } else {
    //  Mia 0.0.0
    HasUser2Button = false;
    //HasChargeInput = false;   //  Not used in this version.
  }

  // ... second, apply MCU EEPROM configuration ()
  SetUpMiaFromMCUEEPROM();  //  SamplePeriod_ms is setup for ascent on returning from SetUpMiaFromMCUEEPROM().

  SetupBMP581();                                 // SamplePeriod_ms must be setup before this function call.
  CurrentPressure = ReadBMP581LatestPressure();  // Flush readings from old setup in output registers.


  // Setup OLED Display
  display.begin(&Adafruit128x32, I2C_OLED_ADDRESS, N_OLEDReset);
  display.setFont(Iain5x7);  // Proportional font to get more characters per line on the OLED.
  display.clear();           // Clear display

  // Now we have to make sure our EEPROM sea level pressure value in MCU EEPROM is valid. Normaly it should be, but on a brand new Mia, it won't be, it will be 0xFFFFFFFF.
  MCUEEPROMSeaLevelPressureCheck();

  if (User1ButtonAfterReset == LOW) {
    DoSensorDisplayLoop();
  }

  display.set2X();
  display.println(F("    Mia"));  // the F() macro will keep a single copy of the text in flash, no RAM use.

  // Lets see if we are hooked to a host computer, if so we will skip the splash screen and instructions question.
  int SkipOLEDSplashInfo;
  int CharCount;
  SkipOLEDSplashInfo = 0;
  delay(100);
  CharCount = Serial.readBytes(receivedChars, numChars - 1);
  if (CharCount > 0) {
    SkipOLEDSplashInfo = 1;
  }

  if (SkipOLEDSplashInfo == 0) {
    DoSplashScreen();  //  Display   Mia    Micro   Intelligent    Altitmeter   splash screen
    delay(200);

    display.clear();  //  Now display last altitude.
    display.println(F("Last Altitude "));
    EEPROM.get(MCU_EEPROM_LAST_MAXIMUM_ALTITUDE, maxAltitude_m);
    display.print(maxAltitude_m * METERS_TO_FEET);
    display.print(F(" ft"));
    delay(3500);

    display.clear();
    display.println(F("Hold USER1 Btn"));
    display.print(F("-> Instructions"));
    delay(2500);
    if (!digitalRead(N_DispButton)) {
      DisplayInstructions();
    }
  }

  //  Initialize out date-time system.
  MCUEEPROMTimeCheck();
  IncrementMCUEEPROMTime();  //  Every reset/power up we increment our date-time by ten seconds. This won't give an accurate time but will keep flight records in chronological order.

  // Now we have to make sure the Latitude and longitude record at MCU_EEPROM_ADDR_LATITUDE_LONGITUDE is correctly initialized.
  //    The first 16 bit word is 0x0001 for its record number. The second 16 bit word is the status, fixed at 0x0100. Test Latitude and longitude follow. Last byte of test is a zero delimiter.
  EEPROM.update(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE, 0x01);
  EEPROM.update(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + 1U, 0x00);
  EEPROM.update(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + 2U, 0x00);
  EEPROM.update(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + 3U, 0x01);
  EEPROM.update(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + 31U, 0x0);  //  end of string

  //  Prepare the display for flight mode
  display.set2X();  // Get display ready for flight mode, instructions may have reduced the size to 1x.

  //  Prepare sensors and variables for flight mode.
  OperationalMode = AllOperationalModes::FlightMode;  // Default operational mode is flight mode.
  FlightModePhaseIndex = 0;
  current_measurement.Record.Status = 0x0040;
  LastDisplayedAltitude_m = InvalidAltitude;
  MCUEEPROMAltitudeCheck();

  if (HasAccelerometer) {
    if (!((AccelKX134ACRCheck() == 0) && (AccelKX134ACRInit() == 0))) {  // If accelerometer fails reset check or initialization, request the user to 'Cycle Pwr' and stop.
      display.print(F("Cycle Pwr"));
      while (1)
        ;
    }
  }
  DoBuzzer(0);
}



/**************************************************************************************************
   Functions
 **************************************************************************************************/

/**
   @brief Check validity of user configuration word in MCU EEPROM.
          Set accelerometer status
          Apply user configuration to system flags
          Sounder setup
          Servo setup

   @details 

   @param none

   @retval none
*/
void SetUpMiaFromMCUEEPROM() {
  // MCU_EEPROM_USER_CONFIGURATION
  EEPROM.get(MCU_EEPROM_USER_CONFIGURATION, UserConfiguration);
  if (UserConfiguration == 0xffffffff) {  //   See if MCU EEPROM is blank
    EEPROM.put(MCU_EEPROM_USER_CONFIGURATION, UserConfigurationDefaultFeatures);
    UserConfiguration = UserConfigurationDefaultFeatures;
  }

  //  **********  Accelerometer setup  *****************************************************************************
  Wire.beginTransmission(I2C_KX134_ADDRESS);  // see if accelerometer is installed
  if (Wire.endTransmission() == 0) {
    HasAccelerometer = true;
    UserConfiguration = UserConfiguration | UserConfigurationHasRohmKX134Accelerometer;
  }

  //  **********  Sounder setup  *****************************************************************************
  if ((UserConfigurationSounderRequiresFrequency & UserConfiguration) == 0) {
    SounderRequiresFrequency = 0;
  } else {
    SounderRequiresFrequency = 1;
  }

  //  **********  Servo setup  *****************************************************************************
  if (UserConfigurationSounderNotServo_mask & UserConfiguration) {  //  UserConfiguration bit 7, 0=Servo output, 1=buzzer output
    ServoNotSounder = 0;                                            // buzzer
  } else {
    ServoNotSounder = 1;  // servo
  }

  //  **********  Select if we are recordind temperature or voltage on P3   *****************************************************************************
  TemperatureNotVoltage = (UserConfigurationP3ThermistorNotVoltage_mask & UserConfiguration) >> 4;

  // **********  Check servo position values in EEPROM and initialize array   ************************
  uint8_t InitUtilityByte;
  EEPROM.get(MCU_EEPROM_SERVO_PRE_LAUNCH, InitUtilityByte);
  if (InitUtilityByte > 180) {
    for (i = 0; i < 6; i++) {
      EEPROM.put(MCU_EEPROM_SERVO_PRE_LAUNCH + i, 90);
    }
    EEPROM.put(MCU_EEPROM_SERVO_HC_Out + i, 90);
  }
  for (i = 0; i < 6; i++) {
    EEPROM.get(MCU_EEPROM_SERVO_PRE_LAUNCH + i, InitUtilityByte);
    ServoFlightStateArray[i] = InitUtilityByte;
  }
  EEPROM.get(MCU_EEPROM_SERVO_HC_Out, InitUtilityByte);  // This servo parameter was added later and is not in sequence as the 6 parameters in the above loop.
  ServoFlightStateArray[6] = InitUtilityByte;


  //  **********  Low power setup  *****************************************************************************
  // Check delay from landing to low power then initialize. Default is 10 seconds (EEPROM byte value = 0x01)
  EEPROM.get(MCU_EEPROM_DELAY_FROM_LANDING_TO_LOW_POWER_MODE, InitUtilityByte);
  if (InitUtilityByte == 0xff) {
    EEPROM.put(MCU_EEPROM_DELAY_FROM_LANDING_TO_LOW_POWER_MODE, 1);
    InitUtilityByte = 1;
  }
  DelayToLowPower_ms = ((uint32_t)InitUtilityByte * LOW_POWER_SLOWDOWN_MULTIPLIER_PER_COUNT * SECONDS_TO_MILLISECONDS);

  // Check low power buzzer multiplier and initialize
  EEPROM.get(MCU_EEPROM_LOW_POWER_BUZ_REST_MULT, InitUtilityByte);
  if (InitUtilityByte == 0xff) {
    EEPROM.put(MCU_EEPROM_DELAY_FROM_LANDING_TO_LOW_POWER_MODE, 1);
  }

  //  **********  more sounder and servo setup  *****************************************************************************
  if (ServoNotSounder) {
    noTone(BuzzerOut);
    MiaServo.attach(BuzzerOut);  // attaches the servo on pin 9 to the servo object
  } else {
    MiaServo.detach();
    // now setup sounder frequencies
    EEPROM.get(MCU_EEPROM_BUZZER_1_FREQ, BuzzerSchedule[1]);
    EEPROM.get(MCU_EEPROM_BUZZER_2_FREQ, BuzzerSchedule[5]);
    if ((BuzzerSchedule[1] > 12000) or (BuzzerSchedule[5] > 12000)) {  //  if out of range, fix to defaults.
      BuzzerSchedule[1] = BuzzerFrequency1;
      BuzzerSchedule[5] = BuzzerFrequency2;
      EEPROM.put(MCU_EEPROM_BUZZER_1_FREQ, BuzzerFrequency1);
      EEPROM.put(MCU_EEPROM_BUZZER_2_FREQ, BuzzerFrequency2);
    }
  }


  //  **********  setup sample rates  *****************************************************************************
  EEPROM.get(MCU_EEPROM_SAMPLE_RATE_POST_APOGEE, SamplePeriod_ms);
  if (SamplePeriod_ms == 0xffff) {
    SamplePeriod_ms = SAMPLE_PERIOD_POST_APOGEE_DEFAULT_ms;
    EEPROM.put(MCU_EEPROM_SAMPLE_RATE_POST_APOGEE, SamplePeriod_ms);
  }
  EEPROM.get(MCU_EEPROM_SAMPLE_RATE_PRE_APOGEE, SamplePeriod_ms);
  if (SamplePeriod_ms == 0xffff) {
    SamplePeriod_ms = SAMPLE_PERIOD_PRE_APOGEE_DEFAULT_ms;
    EEPROM.put(MCU_EEPROM_SAMPLE_RATE_PRE_APOGEE, SamplePeriod_ms);
  }
}


/**
   @brief Put a splash screen on the OLED. Product name and brief description.

   @details 

   @param none

   @retval none
*/
void DoSplashScreen() {
  display.clear();
  display.set2X();
  display.setScrollMode(SCROLL_MODE_AUTO);
  display.print(F("Micro"));
  delay(600);
  display.print(F("\n  Intelligent"));
  delay(800);
  display.print(F("\n     Altimeter"));
  delay(900);
  //display.setScrollMode(SCROLL_MODE_OFF);
}


/**
   @brief Displays a few pages of operating instructions on the OLED in a small font (1x).

   @details There are 7 pages of instructions that are displayed for preset times.

   @note The OLED only supports 4 lines of text with this font.
           This function takes up about 5% of the FLASH memory.

   @param none

   @retval none
*/
void DisplayInstructions() {
  display.set1X();
  display.clear();
  display.println(F("Instruction for Mia operation"));
  display.print(F("Version: "));
  display.println(VersionString);
  delay(4000);

  display.clear();
  display.println(F("Buttons:"));
  display.println(F("Left black button: USER 1"));
  if (HasUser2Button) {                                //  Only >= 0.01 versions of the Mia board has the Right black button
    display.println(F("Right black button: USER 2"));  //  used for setting high current output altitude threshold
  }
  display.println(F("Orange Button: Reset"));
  delay(7000);

  display.clear();
  display.println(F("For accurate altitude the"));
  display.println(F("user must set the current"));
  display.println(F("sea level pressure. Phone"));
  display.println(F("Apps can tell you this."));
  delay(7000);

  display.clear();
  display.println(F("To set/save sea level pres."));
  display.println(F("Long press USER1 button"));
  display.println(F("Up/Dn: Double click USER1"));
  display.println(F("Adjust: short click USER1"));
  delay(8000);

  if (HasUser2Button) {  //  Only >= 0.01 versions of the Mia board has the Right black button
    display.clear();
    display.println(F("Set/save Altitude deploy"));
    display.println(F("Long press USER2 button"));
    display.println(F("Up/Dn: Double click USER1"));
    display.println(F("Adjust: short click USER1"));
    delay(8000);
  }

  display.clear();
  display.println(F("Mia defaults to"));
  display.println(F("FLIGHT MODE. After"));
  display.println(F("your flight Mia shows"));
  display.print(F("maximum altitude in ft."));
  delay(7000);

  display.clear();
  display.println(F("USB charges the"));
  display.println(F("battery."));
  display.println(F("LED ON:  CHARGING"));
  display.println(F("LED OFF: DONE"));
  delay(7000);
}


/**
   @brief Displays all sensor data live until reset

   @note Small character set. Does not return control to Arduino. Must reset to get out of this mode.

   @param none

   @retval none
*/
void DoSensorDisplayLoop() {
  uint8_t I2CScanAddress;
  float DisplayTemperature_F;
  uint8_t AltCycle = 1;
  char Char;

  fieldAltitude_m = 0.0;

  while (HIGH) {
    maxAltitude_m = PressureToAltitude_m(ReadBMP581LatestPressure(), SeaLevelPressure_hPa);
    delay(2000);

    //FindFieldAltitude_m();
    display.clear();
    display.print(F("Alt:"));
    display.print(maxAltitude_m);
    display.print(F(" m  "));
    display.print(maxAltitude_m * METERS_TO_FEET);
    display.println(F(" ft  "));
    display.print(F("Temp:"));
    DisplayTemperature_F = getTemperatureP3();
    display.print((int16_t)(DisplayTemperature_F - 32) * 5 / 9);
    display.print(F(" C  "));
    display.print((int16_t)DisplayTemperature_F);
    display.print(F(" F  "));
    display.print((int16_t)P3Voltage_mV);
    display.println(F(" mV"));
    display.print(F("Light: "));
    display.print(((uint16_t)analogRead(LightSensor) * 3000) >> 10);
    display.println(F(" mV"));

    if (AltCycle == 1U) {
      // Display Latitude & longitude
      EEPROM.get(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + 4, Char);  // First character to print
      i = 1;
      while (Char != 0) {
        display.print((char)Char);
        EEPROM.get(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + 4 + i, Char);
        i++;
      }

    } else if (AltCycle == 2U) {
      //  from: https://playground.arduino.cc/Main/I2cScanner/
      display.print(F("I2C: "));
      for (I2CScanAddress = 1; I2CScanAddress <= 120; I2CScanAddress++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device acknowledged the address.
        Wire.beginTransmission(I2CScanAddress);
        if (Wire.endTransmission() == 0) {
          if (I2CScanAddress < 16U) {
            display.print(F("0"));
          }
          display.print(I2CScanAddress, HEX);
          display.print(F(" "));
        }
      }
    } else if (AltCycle == 3U) {
      //  Display user configuration
      display.print(F("User Config:"));
      if (ServoNotSounder != 0) {  //  if servo
        display.print(F("Servo "));
      } else {
        display.print(F("Sndr "));
        if (SounderRequiresFrequency != 0) {  // if sounder requires freq
          display.print(F("Freq "));
        } else {
          display.print(F("OnOff "));
        }
      }

    } else {
      AltCycle = 1;
    }
    AltCycle++;
    if (AltCycle > 3U) {
      AltCycle = 1U;
    }
  }
}



/**************************************************************************************************
   BMP581 Functions
 **************************************************************************************************/


/*!
 * @brief Setup the BMP581
 *    initialize all registers
 * @return  uint_t  = for all OK, 2 for fail
 */

//  Register indexes
#define BMP581_CHIP_ID_REGISTER_INDEX 0x01  // read only
#define BMP581_REV_ID_REGISTER_INDEX 0x02   // read only

#define BMP581_CHIP_STATUS_REGISTER_INDEX 0x11  // read only

#define BMP581_DRIVE_CONFIG_REGISTER_INDEX 0x13
#define BMP581_INT_CONFIG_REGISTER_INDEX 0x14
#define BMP581_INT_SOURCE_REGISTER_INDEX 0x15
#define BMP581_FIFO_CONFIG_REGISTER_INDEX 0x16
#define BMP581_FIFO_COUNT_REGISTER_INDEX 0x17  // read only
#define BMP581_FIFO_SEL_REGISTER_INDEX 0x18

#define BMP581_TEMP_DATA_XLSB_REGISTER_INDEX 0x1d   // read only
#define BMP581_TEMP_DATA_LSB_REGISTER_INDEX 0x1e    // read only
#define BMP581_TEMP_DATA_MSB_REGISTER_INDEX 0x1f    // read only
#define BMP581_PRESS_DATA_XLSB_REGISTER_INDEX 0x20  // read only
#define BMP581_PRESS_DATA_LSB_REGISTER_INDEX 0x21   // read only
#define BMP581_PRESS_DATA_MSB_REGISTER_INDEX 0x22   // read only

#define BMP581_INT_STATUS_REGISTER_INDEX 0x27  // read only
#define BMP581_STATUS_REGISTER_INDEX 0x28      // read only
#define BMP581_FIFO_DATA_REGISTER_INDEX 0x29   // read only

#define BMP581_NVM_ADDR_REGISTER_INDEX 0x2b
#define BMP581_NVM_DATA_LSB_REGISTER_INDEX 0x2c
#define BMP581_NVM_DATA_MSB_REGISTER_INDEX 0x2d

#define BMP581_DSP_CONFIG_REGISTER_INDEX 0x30
#define BMP581_DSP_IIR_REGISTER_INDEX 0x31
#define BMP581_OOR_THR_P_LSB_REGISTER_INDEX 0x32
#define BMP581_OOR_THR_P_MSB_REGISTER_INDEX 0x33
#define BMP581_OOR_RANGE_REGISTER_INDEX 0x34
#define BMP581_OOR_CONFIG_REGISTER_INDEX 0x35
#define BMP581_OSR_CONFIG_REGISTER_INDEX 0x36
#define BMP581_ODR_CONFIG _REGISTER_INDEX 0x37
#define BMP581_OSR_EFF_REGISTER_INDEX 0x38  // read only

#define BMP581_CMD_REGISTER_INDEX 0x7e

// data for registers

#define BMP581_PRESS_COMP_TEMP_COMP 0x03      //  for register 0x30 pressure and temperature compensation enables
#define BMP581_iir_flush_forced_en_TRUE 0x04  //  for register 0x30 IIR flush (Forced mode only)
#define BMP581_IIR_FILTER_BYPASS_ALL 0x00     //  for register 0x31


#define BMP581_PRESSURE_EN (0b1 << 6)  //  for register 0x36 pressure enable

#define BMP581_OSR_P_x8 (0b011 << 3)   //  for registers 0x36 pressure over-sampling rate (OSR) configuration
#define BMP581_OSR_P_x16 (0b100 << 3)  //  for registers 0x36 pressure over-sampling rate (OSR) configuration
#define BMP581_OSR_P_x32 (0b101 << 3)  //  for registers 0x36 pressure over-sampling rate (OSR) configuration
#define BMP581_OSR_T_x2 0x01           //  for registers 0x36 temperature over-sampling rate (OSR) configuration

#define BMP581_INT_CONFIG_int_en_ENABLED (1 << 3)  //  for registers 0x14 interrupt config. enable interrupt output pin
#define BMP581_INT_CONFIG_int_od_PUSHPULL 0        //  for registers 0x14 interrupt config. open drain disable
#define BMP581_INT_CONFIG_pad_int_drv_xxxxxx 0     //  for registers 0x14 interrupt config. pad drive  <-- data sheet error

//                   Noise & Data rates for continious mode
//Oversampling  setting osr_p  Pressure      Temperature   Typical    Typical ODR in
//                             oversampling  oversampling  pressure   CONTINUOUS
//                                                         RMS noise  mode
//                                                         at 100kPa
//----------------------------------------------------------------------------------
//Lowest power   000           ×1            ×1            0.78 Pa    498 Hz
//               001           ×2            ×1            0.58 Pa    374 Hz
//Std res        010           ×4            ×1            0.41 Pa    255 Hz
//               011           ×8            ×1            0.30 Pa    155 Hz  <-- chosen for Mia for ascent sample periods of 13ms and shorter.
//High res       100           ×16           ×1            0.21 Pa     87 Hz  <-- chosen for Mia for ascent sample periods of 14ms and longer.
//               101           ×32           ×2            0.15 Pa     46 Hz
//               110           ×64           ×4            0.11 Pa     24 Hz
//Highest res    111          ×128           ×8            0.08 Pa     12 Hz

#define BMP581_CONTINIOUS_MODE 0x03  //  for register 0x37 Output data rate (ODR) configuration

#define BMP581_SOFT_RESET 0xb6  //  for register 0x7e  CMD register

uint8_t SetupBMP581() {
  delay(3);  // wait Tstartup, power up time


  // HAVE TO SETUP INTERRUPT PER PAGE 46 OF DATA SHEET (after Bosch gets back to us about INT_CONFIG.pad_int_drv)
  Wire.beginTransmission(I2C_BMP581_ADDRESS);
  Wire.write(BMP581_INT_CONFIG_REGISTER_INDEX);                                                                             //  start at the interrupt configuration register, 0x14
  Wire.write(BMP581_INT_CONFIG_int_en_ENABLED | BMP581_INT_CONFIG_int_od_PUSHPULL | BMP581_INT_CONFIG_pad_int_drv_xxxxxx);  //  0x14    interrupt configuration register, see data sheet sections 6.2 & 7.5
  if (Wire.endTransmission() != 0) {
    return 2;  //
  }

  Wire.beginTransmission(I2C_BMP581_ADDRESS);
  Wire.write(BMP581_DSP_CONFIG_REGISTER_INDEX);  //  start at the DSP configuration register
  Wire.write(BMP581_PRESS_COMP_TEMP_COMP);       //  0x30    DSP configuration register, compensate both pressure and temperature
  Wire.write(BMP581_IIR_FILTER_BYPASS_ALL);      //  0x31 IIR bypass
  if (Wire.endTransmission() != 0) {
    return 2;  //
  }

  // Skipping out of range registers
  Wire.beginTransmission(I2C_BMP581_ADDRESS);
  Wire.write(BMP581_OSR_CONFIG_REGISTER_INDEX);  //  start at the OSR Config register
  if (SamplePeriod_ms >= 14) {
    Wire.write(BMP581_PRESSURE_EN | BMP581_OSR_P_x16 | BMP581_OSR_T_x2);  //  0x36    Over sample rate register
  } else {
    Wire.write(BMP581_PRESSURE_EN | BMP581_OSR_P_x8 | BMP581_OSR_T_x2);  //  0x36    Over sample rate register
  }
  Wire.write(BMP581_CONTINIOUS_MODE);  //  0x37    Output data rate (ODR) configuration
  if (Wire.endTransmission() != 0) {
    return 2;  //
  }
  return 0;
}

/*!
 * @brief Read the last pressure from the BMP581 in millibars (hPa)
 */
float ReadBMP581LatestPressure() {
  Wire.beginTransmission(I2C_BMP581_ADDRESS);
  Wire.write(BMP581_PRESS_DATA_XLSB_REGISTER_INDEX);  //  start at the pressure XLSB register
  Wire.requestFrom(I2C_BMP581_ADDRESS, 3);            // request 3 bytes of pressure
  uint8_t x;
  x = 0;
  while (Wire.available() && (x < 4)) {  // peripheral may send less than requested
    LatestPressure.PressureBytes[x] = Wire.read();
    x++;
  }
  if (Wire.endTransmission() != 0) {
    return 2;  //
  }
  LatestPressure.PressureBytes[3] = 0;                       // the forth byte was not written to from I2C
  return ((float)LatestPressure.PressureFraction) / 6400.0;  //   / 64 for Pa, and then / 100 for millibar (hPa).
}


/*!
 * @brief Convert last pressure reading to altitude

   Pass in the current pressure (in millibars) and sealevel pressure (in millibars).
   Calculates altitude in meters.
   Formula from Bosch BMP180 data sheet and also used in Adafruit's BMP3XX library.

    @param[in]  Pressure_mb  float    Pressure at current altitude
    @param[in]  SeaLevelPressure_hPa  float    Sea-level pressure in hPa

    @return Altitude in meters
*/
float PressureToAltitude_m(float Pressure_mb, float SeaLevelPressure_hPa) {
  return 44330.0 * (1.0 - pow(Pressure_mb / SeaLevelPressure_hPa, 0.1903));
}


/**
   @brief This function is required to be called while the Mia is at ground level. It takes several pressure readings, averages them, and sets fieldAltitude_m to the averaged value.

   @details There are NUMBER_OF_READINGS_TO_AVERAGE pressure measurements taken. Then averaged and converted to an altitude.

   @note fieldAltitude_m is in meters

   @param[out] fieldAltitude_m as a global

   @retval none
*/
#define NUMBER_OF_READINGS_TO_AVERAGE 30
void FindFieldAltitude_m() {
  //  Find field ground level, average 30 readings

  float CurrentPressure;
  CurrentPressure = 0;
  for (i = 0; i < NUMBER_OF_READINGS_TO_AVERAGE; i++) {
    CurrentPressure = CurrentPressure + ReadBMP581LatestPressure();
    delay(30);
  }
  CurrentPressure = CurrentPressure / (float)NUMBER_OF_READINGS_TO_AVERAGE;  //  Find the average pressure.
  fieldAltitude_m = PressureToAltitude_m(CurrentPressure, SeaLevelPressure_hPa);
  LastDisplayedAltitude_m = InvalidAltitude;  //  Invalidate last displayed max altitude.
  maxAltitude_m = fieldAltitude_m;            //  Update our maximum altitude.
}


/**
   @brief Startup check. Check the sea level pressure value in MCU EEPROM, if out of range, set to a nominal value. Then initialize the global SeaLevelPressure_hPa.

   @details We check that the floating point number is not a NAN or INF and we check that we are within our upper and lower limits.

   @note Sea level pressure units are hPa (millibars)

   @param[out] Sealevel pressure in MCU EEPROM may be updated. SeaLevelPressure_hPa updated.

   @retval none
*/
void MCUEEPROMSeaLevelPressureCheck() {
  EEPROM.get(MCU_EEPROM_ADDR_DEFAULT_SEALEVELPRESSURE_HP, SeaLevelPressure_hPa);
  if (isnan(SeaLevelPressure_hPa) || isinf(SeaLevelPressure_hPa)) {  //  if invalid presssure, initialize
    EEPROM.put(MCU_EEPROM_ADDR_DEFAULT_SEALEVELPRESSURE_HP, (float)DEFAULT_SEALEVELPRESSURE_hPa);
    SeaLevelPressure_hPa = (float)DEFAULT_SEALEVELPRESSURE_hPa;
  } else {
    // so SavedSeaLevelPressure is a valid number, now make sure it is in range of pressures on this planet!
    if ((SeaLevelPressure_hPa < MINIMUM_SEA_LEVEL_PRESSURE_hPa) || (SeaLevelPressure_hPa > MAXIMUM_SEA_LEVEL_PRESSURE_hPa)) {
      EEPROM.put(MCU_EEPROM_ADDR_DEFAULT_SEALEVELPRESSURE_HP, (float)DEFAULT_SEALEVELPRESSURE_hPa);
      SeaLevelPressure_hPa = (float)DEFAULT_SEALEVELPRESSURE_hPa;
    }
  }
  EEPROM.get(MCU_EEPROM_ADDR_DEFAULT_SEALEVELPRESSURE_HP, SeaLevelPressure_hPa);
}


/**
   @brief Startup check. Check the Altitude value for the high current output

   @details We check that the floating point number is not a NAN or INF and we check that we are within our upper and lower limits.

   @note Altitude units are in feet

   @param[out] Altitude in MCU EEPROM may be updated.

   @retval none
*/
void MCUEEPROMAltitudeCheck() {
  EEPROM.get(MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft, AltitudeHighCurrentOutSetting_ft);
  if (isnan(AltitudeHighCurrentOutSetting_ft) || isinf(AltitudeHighCurrentOutSetting_ft)) {  //  if invalid presssure, initialize
    EEPROM.put(MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft, (float)DEFAULT_ALTITUDE_ft);
    AltitudeHighCurrentOutSetting_ft = (float)DEFAULT_ALTITUDE_ft;
  } else {
    // so AltitudeHighCurrentOutSetting_ft is a valid number, now make sure it is in range of safe altitudes
    if ((AltitudeHighCurrentOutSetting_ft < MINIMUM_ALTITUDE_ft) || (AltitudeHighCurrentOutSetting_ft > MAXIMUM_ALTITUDE_ft)) {
      EEPROM.put(MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft, (float)DEFAULT_ALTITUDE_ft);
      AltitudeHighCurrentOutSetting_ft = (float)DEFAULT_ALTITUDE_ft;
    }
  }
}


/**
   @brief This checks, and fixes if necessary, the Linux/UNIX 'current time' in MCU EEPROM

   @details   MCU EEPROM date is corrected if the EEPROM date us earlier than the date the software was built (per BIRTH_TIME_OF_THIS_VERSION)

   @param[in]  none

   @retval   none
*/
void MCUEEPROMTimeCheck() {
  EEPROM.get(MCU_EEPROM_ADDR_DEFAULT_TIME_s, EEPROMTime);
  if ((EEPROMTime < BIRTH_TIME_OF_THIS_VERSION)) {  //     don't need to check for empty EEPROM, 0xffffffffffffffff, since that is negitive
    EEPROM.put(MCU_EEPROM_ADDR_DEFAULT_TIME_s, BIRTH_TIME_OF_THIS_VERSION);
  }
}


/**
   @brief  Increments the 'current time' by 10 seconds. This is done for every reset/power up and for every new flight.

   @details

   @param[in]  none

   @retval    none
*/
void IncrementMCUEEPROMTime() {
  EEPROM.get(MCU_EEPROM_ADDR_DEFAULT_TIME_s, EEPROMTime);
  EEPROMTime = EEPROMTime + 10;
  EEPROM.put(MCU_EEPROM_ADDR_DEFAULT_TIME_s, EEPROMTime);
}


//void SetLinuxDate(int64_t NewDate) {
//    EEPROM.put(MCU_EEPROM_ADDR_DEFAULT_TIME_s, NewDate);
//}


/**
   @brief Startup check. Check the logging pointer to the next external EEPROM address stored in MCU EEPROM, if out of range, set to 0.

   @details

   @param[in]  none

   @retval    none
*/
void InitExtEEPROMAddress() {
  uint32_t ExternalEEPROMAddressSaved;
  EEPROM.get(MCU_EEPROM_EXT_EEPROM_ADDR_START, ExternalEEPROMAddressSaved);
  if (ExternalEEPROMAddressSaved >= ExternalEEPROMSizeInBytes) {  //  see if it is invalid
    EEPROM.put(MCU_EEPROM_EXT_EEPROM_ADDR_START, (uint32_t)0);
    EepromAddress = 0;
  }
  EepromAddress = ExternalEEPROMAddressSaved;
}



/**************************************************************************************************
   Mia buzzer (sounder) function
 *************************************************************************************************
   @brief Starts, continues, or stops a buzzer sequence.

   @details

   @note  The tone array gets modified by the low power mode and flight mode phase 0 initialization.

   @param BuzzerEnable uint8_t  0=Turn buzzer off   1=Start Buzzer sequence   250=Return buzzer status
   @retval uint8_t Buzzer status
*/
uint8_t DoBuzzer(uint8_t BuzzerEnable) {
  static uint8_t BuzzerWasEnabled;
  //static unsigned long BuzzerStartTime;  // this is the start time of the buzz schedule
  const int BuzzerScheduleLength = 8;
  static int BuzzerArrayIndex;
  static unsigned long BuzzEndTime;
  uint16_t BuzzFreq;
  if (!ServoNotSounder) {
    //MiaServo.detach();

    if (BuzzerEnable == 250) {  // check return status flag (250)
      return BuzzerWasEnabled;  // Return buzzer status if parameter was 250.

    } else if (BuzzerEnable == 0) {
      //  Turn off the Sounder.
      BuzzerWasEnabled = 0;
      noTone(BuzzerOut);
      return 0;

    } else {
      if (SounderRequiresFrequency) {
        unsigned long CurrentTime = millis();

        if (BuzzerWasEnabled == 0) {
          // First turn on after being off. Grab our millisecond timer as our starting reference.
          BuzzerWasEnabled = 1;
          BuzzerArrayIndex = 0;  // Start at the beginning of the array of tones.
          BuzzEndTime = (unsigned long)BuzzerSchedule[0] + CurrentTime;
          tone(BuzzerOut, BuzzerSchedule[1]);
          return 0;
        }

        if (BuzzEndTime > CurrentTime) {
          // Still not done with this note.
          return 1;
        } else {
          // done with this note, lets index to the next
          BuzzerArrayIndex = BuzzerArrayIndex + 2;
          if (BuzzerArrayIndex >= BuzzerScheduleLength) {
            //  We went off the end of the array of tones, restart back at the beginning.
            BuzzerArrayIndex = 0;  // start at the beginning
          }
        }
        BuzzEndTime = (unsigned long)BuzzerSchedule[BuzzerArrayIndex] + CurrentTime;
        BuzzFreq = BuzzerSchedule[BuzzerArrayIndex + 1];
        //Serial.println(BuzzFreq);
        if (BuzzFreq == 0) {
          noTone(BuzzerOut);
        } else {
          tone(BuzzerOut, BuzzFreq);
        }
        return 0;
      } else {
        digitalWrite(BuzzerOut, BuzzerEnable);
      }
    }
  }
  return -1;
}



//  **************************************************************************************************************************
//  Flight mode functions           Flight mode functions              Flight mode functions           Flight mode functions
//  **************************************************************************************************************************

/**
   @brief Checks the operational condition of the accelerometer (KX134ACR).

   @details

   @note 

   @retval uint8_t  0:working accelerometer   1:No accelerometer option  2:Accelerometer needs power cycle per Rohm application note AN011E rev 001, KX134ACR-LBZ Power-on Procedure.
*/
#define ACCEL_KX134ACR_WHO_AM_I_ADDR 0x0F
#define ACCEL_KX134ACR_CNTL1_ADDR 0x18
#define ACCEL_KX134ACR_CNTL2_ADDR 0x19
#define ACCEL_KX134ACR_INC4_ADDR 0x1F
#define ACCEL_KX134ACR_MAN_WAKE_ADDR 0x2C
#define ACCEL_KX134ACR_LP_CNTL_ADDR 0x35
#define ACCEL_KX134ACR_BUF_CNTL2_ADDR 0x3B

#define ACCEL_KX134ACR_SWReset 0x80
#define ACCEL_KX134ACR_WHO_AM_I_VALUE 0xCC

uint8_t AccelKX134ACRCheck() {
  delay(50);  // Wait Tpu, power up time.

  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(0x7f);  // Undocumented register address per Rohm application note AN011E rev 001, KX134ACR-LBZ Power-on Procedure.
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    return 2;  //  The accelerometer failed to ack, needs power cycle.
  }

  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_CNTL2_ADDR);  // CNTL2 register address for software reset.
  Wire.write(ACCEL_KX134ACR_SWReset);
  if (Wire.endTransmission() != 0) {
    return 2;  //  The accelerometer failed to ack the software reset request, needs power cycle
  }

  delay(2);  //  Wait for software reset to complete.

  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_WHO_AM_I_ADDR);  // Read who am I register for chip operation detect.
  if (Wire.endTransmission() != 0) {
    return 2;  //  The accelerometer failed to ack the who am I register address, needs power cycle
  }

  uint8_t WAI_Result;
  Wire.requestFrom(I2C_ACCELEROMETER_ADDRESS, 1);  // Request 1 byte from KX134ACR's WHO_AM_I register.
  while (Wire.available()) {                       // Peripheral may send less than requested.
    WAI_Result = Wire.read();
  }  // receive a byte
  Wire.endTransmission();

  if (WAI_Result != ACCEL_KX134ACR_WHO_AM_I_VALUE) {
    return 2;
  }
  return 0;
}



/**
   @brief Initializes the accelerometer (KX134ACR).

   @details

   @note 

   @retval uint8_t  0:good   1:fail
*/
#define ACCEL_KX134ACR_OWUF 6  // 50 Hz output data rate for the Wake-up function and the High-pass filter outputs (0.781 to 100 Hz).
#define ACCEL_KX134ACR_OSA 6   // 50 Hz output data rate for Acceleration output data (0.781 to 25600 Hz).
#define ACCEL_KX134ACR_OBTS 6  // 50 Hz output data rate for the Back-to-sleep and the High-pass filter outputs (0.781 to 100 Hz).
#define ACCEL_KX134ACR_AVC 1   // 2 samples are averaged.
#define ACCEL_KX134ACR_8G 0
#define ACCEL_KX134ACR_16G 1
#define ACCEL_KX134ACR_32G 2
#define ACCEL_KX134ACR_64G 3  // Selected range for Mia

uint8_t AccelKX134ACRInit() {
  if (!HasAccelerometer) {
    return 1;  //  The board has no option for an accelerometer
  }
  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_CNTL1_ADDR);   // CNTL1 register address for start register address
  Wire.write(0);                           //  CNTL1 standby mode for setup (This register should already be 0x00 from reset)
  Wire.write(0);                           //  CNTL2
  Wire.write(0xa8 | ACCEL_KX134ACR_OWUF);  //  CNTL3
  Wire.write(0x40 | ACCEL_KX134ACR_OSA);   //  ODCNTL
  Wire.write(0);                           //  INC1
  Wire.write(0);                           //  INC2
  if (Wire.endTransmission() != 0) {
    return 1;  //  The accelerometer failed to ack a write during configuration
  }

  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_MAN_WAKE_ADDR);  // MAN_WAKE register address for start register address
  Wire.write(0x02);                          //  MAN_WAKE
  Wire.write(0x40 | ACCEL_KX134ACR_OBTS);    //  BTS_CNTL
  if (Wire.endTransmission() != 0) {
    return 1;  //  The accelerometer failed to ack a write during configuration.
  }

  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_LP_CNTL_ADDR);       // LP_CNTL register address for start register address.
  Wire.write(0x8b | (ACCEL_KX134ACR_AVC << 4));  //  LP_CNTL: 2 samples averaged.
  if (Wire.endTransmission() != 0) {
    return 1;  //  The accelerometer failed to ack a write during configuration.
  }

  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_BUF_CNTL2_ADDR);  // BUF_CNTL2 register address for start register address.
  Wire.write(0x41);                           //  BUF_CNTL2: sample buffer is disabled, 16 bit samples, interrupt disabled, stream mode.
  if (Wire.endTransmission() != 0) {
    return 1;  //  The accelerometer failed to ack a write during configuration.
  }

  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_CNTL1_ADDR);         // CNTL1 register address to make the accelerometer active.
  Wire.write(0xc0 | (ACCEL_KX134ACR_64G << 3));  //  CNTL1: high resolution operating mode.
  if (Wire.endTransmission() != 0) {
    return 1;  //  The accelerometer failed to ack a write during configuration.
  }
  return 0;  // All good.
}


/**
   @brief Reads all 3 axis of the accelerometer (KX134ACR) and populates the logging EEPROM structure.

   @details

   @note 

   @retval uint8_t  0:good   !=0:fail
*/
#define ACCEL_KX134ACR_XOUTL_ADDR 0x06

uint8_t AccelKX134ACRRead() {
  if (!HasAccelerometer) {
    return 1;  //  The board has no option for an accelerometer.
  }
  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_XOUTL_ADDR);  // Starting address for 6 bytes of acceleration data
  if (Wire.endTransmission() != 0) {
    return 1;  //  The accelerometer failed to ack the who am I register address, needs power cycle.
  }

  int16_t Accel_Result;
  int16_t AccelX = 0;                              //  The Mia default orientation is USB connector down, so the X axis is populated from the Z registers. See comments at very end of file for orientation.
  int16_t AccelY = 0;                              //  The Mia default orientation is USB connector down, so the Y axis is populated from the Y registers.
  int16_t AccelZ = 0;                              //  The Mia default orientation is USB connector down, so the Z axis is populated from the X registers.
  Wire.requestFrom(I2C_ACCELEROMETER_ADDRESS, 6);  // Request 6 bytes from KX134ACR's X, Y, and Z registers.
  uint8_t x = 0;
  AccelX = 0;
  AccelY = 0;
  AccelZ = 0;
  while (Wire.available()) {  // Peripheral may send less than requested.
    x++;
    Accel_Result = Wire.read();
    Accel_Result = Accel_Result & 0xff;
    switch (x) {
      case 1:
        AccelZ = Accel_Result;  //  XOUTL
        break;
      case 2:
        AccelZ = AccelZ | (Accel_Result << 8);  //  XOUTH
        break;
      case 3:
        AccelY = Accel_Result;  //  YOUTL
        break;
      case 4:
        AccelY = AccelY | (Accel_Result << 8);  //  YOUTH
        break;
      case 5:
        AccelX = Accel_Result;  //  ZOUTL
        break;
      case 6:
        AccelX = AccelX | (Accel_Result << 8);  //  ZOUTH
        break;
    }
  }
  Wire.endTransmission();

  float mgTog = 0.001953125;
  current_measurement.Record.AccelerationX_g = (float)AccelX * mgTog;
  current_measurement.Record.AccelerationY_g = (float)AccelY * mgTog;
  current_measurement.Record.AccelerationZ_g = (float)AccelZ * mgTog;

  return x - 6;
}


/**
   @brief Reads only the high byte of X axis of the accelerometer, Mia -Z axis (KX134ACR)    CURRENTLY NOT USED

   @details

   @note 

   @retval int8_t  Mia Z acceleration
*/
#define ACCEL_KX134ACR_XOUTH_ADDR 0x07
int8_t AccelKX134ACRReadXOnly() {
  int8_t AccelZ;
  Wire.beginTransmission(I2C_ACCELEROMETER_ADDRESS);
  Wire.write(ACCEL_KX134ACR_XOUTH_ADDR);  // Starting address for 6 bytes of acceleration data
  Wire.endTransmission();
  Wire.requestFrom(I2C_ACCELEROMETER_ADDRESS, 1);  // request 2 bytes from KX134ACR's X registers
  AccelZ = 2;                                      // default to Mia Z axis to -64g if no reading is available
  while (Wire.available()) {                       // peripheral may send less than requested
    AccelZ = Wire.read();
  }
  Wire.endTransmission();
  return -AccelZ;
}

/**
 * @brief Write a record if we are at the next sample period.
 *
 *
 * @return 
 */
void WriteRecordAtSamplePeriod(uint8_t ForceWriteNow) {
  if (EepromAddress + 64 > ExternalEEPROMSizeInBytes) {  // Check for room in the external EEPROM (Leaving room for a final record).
    FlightModePhaseIndex = 3;                            // There is not enough room in the external EEPROM for the next record, so we switch to display altitude only mode and return.
    return;
  }
  if ((millis() >= (TimeStamp1Ago + SamplePeriod_ms)) || ForceWriteNow) {
    writeByteArray(EepromAddress, current_measurement.Bytes, sizeof(current_measurement.Record));  //  Writing out pre-populated measurement record.
    TimeStamp1Ago += SamplePeriod_ms;
    EepromAddress = EepromAddress + sizeof(current_measurement.Record);  //  Increment to starting address of next write.
    RecordNumber++;
  }
}



//  *****************************************************************************************************************
//  External EEPROM functions      External EEPROM functions      External EEPROM functions      External EEPROM functions
//  *****************************************************************************************************************
// for M24M02E      Data sheet used: DS14157 - Rev 2 - August 2024

//  The M24M02E consumes I2C addresses 0x50, 0x51, 0x52, 0x53, 0x58.
//  If a second device is used (on QWIIC connector), it must have its address programmed, and locked, off the Mia so the onboard EEPROM address is not changed. It will consume I2C addresses 0x54, 0x55, 0x56, 0x57, 0x5c

#define M24M02E_DEVICE_TYPE_IDENTIFIER_BITS_CTRL 0b1011  // per data sheet section 4.1 and 5.5 table 9, for all non data read-write operations
#define M24M02E_DEVICE_TYPE_IDENTIFIER_BITS_MEM 0b1010   // per data sheet section 5.5 table 9, for all data read-write operations on the 256 byte region
#define M24M02E_DEVICE_TYPE_IDENTIFIER_Lock 0b1          // per data sheet section 4.1, read only
#define M24M02E_DEVICE_TYPE_IDENTIFIER_A15_A13 0b111     // per data sheet section 4.1

#define M24M02E_DEVICE_TYPE_IDENTIFIER_REGISTER_VALUE ((M24M02E_DEVICE_TYPE_IDENTIFIER_BITS_CTRL << 4) | M24M02E_DEVICE_TYPE_IDENTIFIER_Lock)  // per data sheet section 4.1, read only

#define M24M02E_C2_DEFAULT_VALUE 0b0                       // per data sheet section 4.2 TABLE 6. For I2C addresses 0x50, 0x51, 0x52, 0x53, 0x58.
#define M24M02E_CONFIGURABLE_DEVICE_ADDRESS_A15_A13 0b110  // per data sheet section 4.2 and section 5.5 table 10.

#define M24M02E_SOFTWARE_WRITE_PROTECTION_A15_A13 0b101  // per data sheet section 4.3 and section 5.5 table 10.

#define M24M02E_DEVICE_SELECT_CODE_CTRL ((M24M02E_DEVICE_TYPE_IDENTIFIER_BITS_CTRL << 3) | (M24M02E_C2_DEFAULT_VALUE << 2))     // For all non data read-write accesses, building a 7 bit field as a I2C device address
#define M24M02E_DEVICE_SELECT_CODE_MEM_BASE ((M24M02E_DEVICE_TYPE_IDENTIFIER_BITS_MEM << 3) | (M24M02E_C2_DEFAULT_VALUE << 2))  // For all data read-write accesses (no A16, A17), building a 7 bit field as a I2C device address

#define M24M02E_WRITE_TIME_ms 4  // This is a data sheet maximum from section 9 table 19. Polling may offer shorter write times.

#define M24M02E_DEVICE_ID(UserEEPROMAddress, M24M02E_DEVICE_MEM_ID) ((UserEEPROMAddress >> 16U) | M24M02E_DEVICE_MEM_ID)

/**
 * @brief The function polls the M24M02E to see if the write cycle is complete
 *
 *
 * @return Write completed on time (milliseconds to complete), Write failed to complete (-1), Write completed late (-2)
 */
int8_t M24M02E_PollForWriteDone() {
  unsigned long BeginTime_ms = millis();
  unsigned long GiveUpTime_ms = BeginTime_ms + M24M02E_WRITE_TIME_ms + 3;  //  We give the M24M02E three extra milliseconds to finish because returning without an I2C acknoledge leaves the M24M02E I2C in an unknown state.
  unsigned long Now_ms = millis();
  while (GiveUpTime_ms >= Now_ms) {
    Wire.beginTransmission(M24M02E_DEVICE_SELECT_CODE_MEM_BASE);
    if (Wire.endTransmission() == 0) {
      //  Good news: the write is complete. Other news, we are now mid command with the M24M02E (the M24M02E has received and ACKed its device ID), so we must end the I2C transmission.
      //digitalWrite(TestPoint7, LOW);  //  trigger scope at end of write
      //digitalWrite(TestPoint7, HIGH);
      //digitalWrite(TestPoint7, LOW);
      Wire.endTransmission();  //  send a re-start and a stop per figure 16 in the M24M02E data sheet
      Wire.endTransmission(true);
      if ((Now_ms - BeginTime_ms) <= M24M02E_WRITE_TIME_ms) {
        return Now_ms - BeginTime_ms;  //  Success
      } else {
        return -2;  // Write complete but not within data sheet maximum
      }
    }
    Now_ms = millis();
  }
  return -1;  //  Write failed, the M24M02E is in an unknown state.
}


/**
 * @brief The function verifies the device address (by access) and sets the lock bit if it isn't set. The software device type and protection registers are also checked.
 *
 *
 * @return success (0), access failure (1), DTR register mismatch (2), CDA register fail (3),  or software write protection error (4)
 */
int8_t M24M02E_Setup() {
  uint8_t ReadByte;
  ReadByte = 0x55;

  //Serial.println(F("Entered EEPROM setup"));
  // First, DTR register and access check
  Wire.beginTransmission(M24M02E_DEVICE_SELECT_CODE_CTRL);
  Wire.write(M24M02E_DEVICE_TYPE_IDENTIFIER_A15_A13 << 5U);
  Wire.write(0);
  if (Wire.endTransmission() != 0) {
    //Serial.println(F("DTR i2c access failed"));
    return 1;
  }
  Wire.requestFrom(M24M02E_DEVICE_SELECT_CODE_CTRL, 1);  // request the 1 byte device type identifier
  ReadByte = Wire.read();
  if (ReadByte != M24M02E_DEVICE_TYPE_IDENTIFIER_REGISTER_VALUE) {
    //Serial.print(F("DTR register value wrong "));
    //Serial.println(ReadByte, HEX);
    return 2;
  }
  //Serial.print(F("DTR register value good "));
  //Serial.println(ReadByte, HEX);

  //  Next, CDA reg. First check it, if not protected, fix it. And lastly read/reread and verify address and protected.
  Wire.beginTransmission(M24M02E_DEVICE_SELECT_CODE_CTRL);
  Wire.write(M24M02E_CONFIGURABLE_DEVICE_ADDRESS_A15_A13 << 5U);
  Wire.write(0);
  if (Wire.endTransmission() != 0) {
    return 1;
  }
  Wire.requestFrom(M24M02E_DEVICE_SELECT_CODE_CTRL, 1);  // Request the 1 byte configuration device address reg.
  ReadByte = Wire.read();
  if (ReadByte != 0b0001) {
    // We need to lock the C2 bit to zero (Fix)
    Wire.beginTransmission(M24M02E_DEVICE_SELECT_CODE_CTRL);
    Wire.write(M24M02E_CONFIGURABLE_DEVICE_ADDRESS_A15_A13 << 5U);
    Wire.write(0);
    Wire.write(0b0001);  // lock the register with C2=0
    Wire.endTransmission();
    delay(M24M02E_WRITE_TIME_ms);  // Wait for the write to complete
  }

  // verify
  Wire.beginTransmission(M24M02E_DEVICE_SELECT_CODE_CTRL);
  Wire.write(M24M02E_CONFIGURABLE_DEVICE_ADDRESS_A15_A13 << 5U);
  Wire.write(0);
  if (Wire.endTransmission() != 0) {
    return 1;
  }
  Wire.requestFrom(M24M02E_DEVICE_SELECT_CODE_CTRL, 1);  // Request the 1 byte configuration device address reg.
  ReadByte = Wire.read();
  if (ReadByte != 0b0001) {  //  C2 must be zero and DAL (protect) must be 1.
    return 3;
  }

  //  Last software protection
  Wire.beginTransmission(M24M02E_DEVICE_SELECT_CODE_CTRL);
  Wire.write(M24M02E_SOFTWARE_WRITE_PROTECTION_A15_A13 << 5U);
  Wire.write(0);
  if (Wire.endTransmission() != 0) {
    return 1;
  }
  Wire.requestFrom(M24M02E_DEVICE_SELECT_CODE_CTRL, 1);  // request the 1 byte software write protection reg.
  ReadByte = Wire.read();
  if (ReadByte != 0) {
    return 4;
  }
  //Wire.endTransmission(stop);
  return 0;
}



/**
 * @brief The function reads a specified number of bytes from the specified address and stores them in the
 * specified array
 *
 * @note Reads across 64k boundrys are not supported
 *
 * @param address The address of the first byte to read.
 * @param data The array of data to be written to the EEPROM.
 * @param indexCount The number of bytes to read.
 *
 * @return The data read from the EEPROM.
 */
int8_t readByteArray(uint32_t address, uint8_t data[], uint8_t indexCount) {
  memset(data, 0, indexCount);

  Wire.beginTransmission((uint8_t)M24M02E_DEVICE_ID(address, M24M02E_DEVICE_SELECT_CODE_MEM_BASE));
  Wire.write((uint8_t)((address & 0xFFFF) >> 8U));
  Wire.write((uint8_t)(address & 0xFF));
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)M24M02E_DEVICE_ID(address, M24M02E_DEVICE_SELECT_CODE_MEM_BASE), indexCount);
  uint8_t x;
  uint8_t ReturnFailValue;
  ReturnFailValue = 0;
  for (x = 0; x < indexCount; x++) {
    if (Wire.available() > 0)  // 20240616
    {
      data[x] = Wire.read();
    } else {
      ReturnFailValue = 1;
    }
  }
  return ReturnFailValue;
}


/**
 * @brief This function writes a byte array to the specified address in a M24M02E
 *
 * @note Writes across 64k boundrys are not supported. The Arduino Wire library (and the MiniCore Wire library) have a byte buffer that is 32 bytes long for AVR based boards.
 *       With the two address bytes, this only leaves 30 bytes for data per write. So larger data arrays have to be broken up before the actual write.
 *
 * @param address The address of the first byte to write to.
 * @param data The data to be written to the EEPROM.
 * @param indexCount The number of bytes to write.
 *
 * @return 0:Write completed. 1:Data too long to fit in transmit buffer. 2:Received NACK on transmit of address. 3:Received NACK on transmit of data. 4:Other error. 5:Timeout. 6:Exceeded EEPROM size.
 */
#define ArduinoWireBufferSize 32  // this size is different for other processors on Arduino boards, can we use TWI_BUFFER_SIZE?
#define EEPROMWireAddressOverhead 2
#define EEPROMWriteMaximumChunkSize (ArduinoWireBufferSize - EEPROMWireAddressOverhead)
int8_t writeByteArray(uint32_t address, uint8_t data[], uint8_t indexCount) {
  int16_t ByteCountRemaining;
  uint32_t CurrentAddress;
  uint8_t ArrayPointer;
  uint8_t ErrorReturn;
  ErrorReturn = 0;
  CurrentAddress = address;
  ArrayPointer = 0;
  for (ByteCountRemaining = indexCount; ByteCountRemaining > 0; ByteCountRemaining = ByteCountRemaining - EEPROMWriteMaximumChunkSize) {
    //M24M02E_PollForWriteDone();  // See data sheet section 6.2.6
    Wire.beginTransmission((uint8_t)M24M02E_DEVICE_ID(CurrentAddress, M24M02E_DEVICE_SELECT_CODE_MEM_BASE));
    Wire.write((uint8_t)((CurrentAddress & 0x00FF00) >> 8U));  // AND off address MSB & bottom byte
    Wire.write((uint8_t)(CurrentAddress & 0xFF));

    //digitalWrite(TestPoint7, LOW);    //  Debug/validation trigger scope at beginning of write
    //digitalWrite(TestPoint7, HIGH);
    //digitalWrite(TestPoint7, LOW);
    uint8_t x;
    for (x = 0; x < ByteCountRemaining; x++) {
      if ((CurrentAddress + x) >= ExternalEEPROMSizeInBytes) {
        ErrorReturn = 6;
        break;
      }
      Wire.write(data[ArrayPointer + x]);
    }
    ErrorReturn = ErrorReturn + Wire.endTransmission(true);  //  Terminate the transfer and start the actual write cycle with a I2C STOP condition per section 6.1.2 in the data sheet.
    if (ErrorReturn != 0) {
      break;
    }
    M24M02E_PollForWriteDone();  // See data sheet section 6.2.6    should I move this to the begining so the write happens in parallel to other software running?????
    CurrentAddress = CurrentAddress + EEPROMWriteMaximumChunkSize;
    ArrayPointer = ArrayPointer + EEPROMWriteMaximumChunkSize;
  }
  return ErrorReturn;
}




/**
   @brief Reads temperature or voltage depending on TemperatureNotVoltage. For temperature, a thermistor on the P3 connector is required.
          Calibrated for a muRata NXFT15XH103FA2B sensor.

   @details

   @note none

   @retval float  Temperature in degrees F
*/
float getTemperatureP3() {
  uint16_t P3ADRaw;
  float Temperature;
  P3ADRaw = analogRead(AnalogInputP3);
  P3Voltage_mV = P3ADRaw * 3000.0 / 1023.0;  // Voltage, im millivolts, on connector P3.
  if (TemperatureNotVoltage) {
    Temperature = ADToTemperature(P3ADRaw);  //  Return temperature on P3.
  } else {
    Temperature = P3ADRaw * 0.001;  //  Return volts on P3.
  }
  return Temperature;
}



/**
   @brief  getAltitude   read of pressure from sensor then convert to altitude

   @details

   @note

   @param[in]
   @param[in]
   @param[out]

   @retval
*/
void getAltitude() {
  CurrentPressure = ReadBMP581LatestPressure();
  newAltitude_m = PressureToAltitude_m(CurrentPressure, SeaLevelPressure_hPa);
  if (maxAltitude_m < newAltitude_m) {  // If new altitude is greater then max altitude
    maxAltitude_m = newAltitude_m;      // Move new altitude into max altitude
  }
}


/**
   @brief  Displays the maximum altitude converted to above ground level.

   @details  Does not update display if it has not changed.

   @note  Does not do a clear screen to reduce flicker

   @param[in]  maxAltitude_m as global
   @param[in]  LastDisplayedAltitude_m as global

   @retval none
*/
void displayAltitude() {
  if (maxAltitude_m != LastDisplayedAltitude_m) {
    display.clearField(0, 0, 1);      // home cursor
    display.println(F("Altitude "));  //  takes 1ms plus 0.62ms per character
    display.print((maxAltitude_m - fieldAltitude_m) * METERS_TO_FEET);
    display.print(F(" ft   "));
    LastDisplayedAltitude_m = maxAltitude_m;
  }
}


/**
   @brief  Collect sensor readings and time stamp into rocket flight record structure

   @details

   @note

   @param[in] uint16_t RecordIndexValue - The record index number for this flight (index 0 is initial conditions)

   @retval none
*/
void PopulateFlightRecord(uint16_t RecordIndexValue) {
  current_measurement.Record.RecordIndex = RecordIndexValue;
  TimeStamp = millis();
  current_measurement.Record.current_time_ms = TimeStamp;
  getAltitude();  // grab the latest result (closest to our time stamp)
  current_measurement.Record.Temperature = getTemperatureP3();
  if (HasAccelerometer) {
    AccelKX134ACRRead();
  } else {
    float Zero_float = 0.0;
    current_measurement.Record.AccelerationX_g = Zero_float;
    current_measurement.Record.AccelerationY_g = Zero_float;
    current_measurement.Record.AccelerationZ_g = Zero_float;
  }
  if (RecordIndexValue == 0) {
    // initial, pre-flight record
    current_measurement.Record.Altitude = fieldAltitude_m;                                                                             // for the initial record this is the field altitude
    current_measurement.Record.Status = 0x01 | (0x02 & (digitalRead(HighCurrentOut) << 1)) | (0x08 & (digitalRead(TestPoint7) << 3));  //  This is the bits: 0:InitialRecord (with field altitude and sea level pressure). 1:PD7_On. 2:BuzzerOn. 3:PD3_TP7. 8:Second Init record with lat & lon
    //   4:Altitude in feet. 5: Temperature in °C.   15: Landing detected   14: Apogee detected
    current_measurement.InitRecord.SeaLevelPressureX4 = int(SeaLevelPressure_hPa * 4.0);  //   fix   Why save in moon units????  it is a 32 bit data type, save it as a float.  FIX FIX FIX host too
    current_measurement.InitRecord.LinuxDateTime = EEPROMTime;
    current_measurement.InitRecord.spare = 0;
  } else {
    // flight record
    current_measurement.Record.Altitude = newAltitude_m - fieldAltitude_m;                                                                     // for the initial record this is the field altitude
    current_measurement.Record.Status = (0x02 & (digitalRead(HighCurrentOut) << 1)) | (0x08 & (digitalRead(TestPoint7) << 3)) | FlightStatus;  //  See bottom of this file for format.
    current_measurement.Record.LightVoltage = ((uint32_t)analogRead(LightSensor) * (uint32_t)3000) >> 10;                                      // the shift introduces only 1 part in 1023 error.       (uint32_t)analogRead(LightSensor) * (uint32_t)3000 / (uint32_t)1023.0;  // uint32_t light millivolts
  }
}



/**
   @brief  Collect sensor readings from 3 queued, pre-launch detected measurements

   @details

   @note

   @param[in] uint16_t RecordIndexValue - The record index number for this flight (index 0 is initial conditions)

   @retval none
*/
void PopulatePreLaunchQueueFlightRecord(uint16_t RecordIndexValue, uint32_t TimeStamp) {
  current_measurement.Record.RecordIndex = RecordIndexValue;
  current_measurement.Record.current_time_ms = TimeStamp;
  //LastRecordTimeStamp_ms = TimeStamp;
  current_measurement.Record.Temperature = getTemperatureP3();
  if (HasAccelerometer) {
    AccelKX134ACRRead();
  } else {
    current_measurement.Record.AccelerationX_g = (float)0.0;
    current_measurement.Record.AccelerationY_g = (float)0.0;
    current_measurement.Record.AccelerationZ_g = (float)0.0;
  }
  current_measurement.Record.Altitude = newAltitude_m - fieldAltitude_m;                                                                     // for the initial record this is the field altitude
  current_measurement.Record.Status = (0x02 & (digitalRead(HighCurrentOut) << 1)) | (0x08 & (digitalRead(TestPoint7) << 3)) | FlightStatus;  // See bottom of this file for format.
  current_measurement.Record.LightVoltage = (uint32_t)analogRead(LightSensor) * (uint32_t)3000 / (uint32_t)1023.0;                           // uint32_t light millivolts
}



/**
   @brief Convert A/D reading from thermistor to degrees F.

   @details
           To generate this table we used:
           python3 SensorTableTranslator.py -v 3.0 -b 8 -m 100 -d uint16_t NXFT15XH103FEAB050ConversionF.csv

           With these details:
           Options:
             ADVref=3.0
             ADBits=8
             SensorMultiplier=100.0
             DeclarationType='uint16_t'
             TableFileName='NXFT15XH103FEAB050ConversionF.csv'

           Usage:
           index = ADreading - 13;
           SensorDataFromArray = SensorTable[index];
           SensorReading = SensorDataFromArray / 100.0;


   @note

   @param[in] uint16_t ADReading 10 bit value from A/D converter

   @retval Float  Temperature in degrees F
*/
float ADToTemperature(uint16_t ADReading) {
  uint16_t index;
  float SensorDataFromArray;
  float SensorReading;
  uint8_t TwoLSB;
  float NextSensorDataFromArray;

  const int16_t SensorTable[/*230*/] = {
    25644,  //  Sensor=256.4364025364855    //  sorry for the silly number of significant digits in the temperature °F in comments (unformated python output)
    25035,  //  Sensor=250.3485977621176
    24474,  //  Sensor=244.74400417600373
    23955,  //  Sensor=239.54660220328043
    23470,  //  Sensor=234.70250529418323
    23017,  //  Sensor=230.16670725568304
    22591,  //  Sensor=225.91200982396128
    22189,  //  Sensor=221.89422765146674
    21809,  //  Sensor=218.088828879905
    21448,  //  Sensor=214.4819480414604
    21106,  //  Sensor=211.05840734941333
    20781,  //  Sensor=207.8111946186404
    20470,  //  Sensor=204.70315431130103
    20174,  //  Sensor=201.73628020493857
    19888,  //  Sensor=198.8840640006293
    19614,  //  Sensor=196.14211767048116
    19350,  //  Sensor=193.50152282777367
    19094,  //  Sensor=190.9381237493336
    18846,  //  Sensor=188.46326344619715
    18608,  //  Sensor=186.0789121086869
    18377,  //  Sensor=183.76962779703322
    18153,  //  Sensor=181.52739837241955
    17936,  //  Sensor=179.35716172745776
    17725,  //  Sensor=177.25327636612744
    17521,  //  Sensor=175.20912585142256
    17322,  //  Sensor=173.22454316021177
    17129,  //  Sensor=171.29301093771582
    16941,  //  Sensor=169.41006946326164
    16757,  //  Sensor=167.57483325970188
    16579,  //  Sensor=165.78593132798403
    16404,  //  Sensor=164.03862725223595
    16233,  //  Sensor=162.33056616609875
    16066,  //  Sensor=160.66026109880735
    15903,  //  Sensor=159.0260958207472
    15742,  //  Sensor=157.42381249258764
    15585,  //  Sensor=155.85006559234864
    15431,  //  Sensor=154.3077421728793
    15280,  //  Sensor=152.79797412120675
    15132,  //  Sensor=151.31924429356118
    14987,  //  Sensor=149.86632097018423
    14844,  //  Sensor=148.43817711447886
    14703,  //  Sensor=147.03391851901674
    14566,  //  Sensor=145.65779740639783
    14431,  //  Sensor=144.3066025618047
    14298,  //  Sensor=142.97578651490372
    14166,  //  Sensor=141.66410399071196
    14038,  //  Sensor=140.37739296226482
    13912,  //  Sensor=139.12320615215546
    13789,  //  Sensor=137.88927968166965
    13668,  //  Sensor=136.67501601892602
    13548,  //  Sensor=135.47917449669504
    13430,  //  Sensor=134.29546110375307
    13313,  //  Sensor=133.13076766818202
    13198,  //  Sensor=131.98261006230592
    13084,  //  Sensor=130.84402670509948
    12972,  //  Sensor=129.72408020237216
    12862,  //  Sensor=128.61588256804794
    12752,  //  Sensor=127.51801837484403
    12644,  //  Sensor=126.43924635216499
    12537,  //  Sensor=125.3673196308773
    12431,  //  Sensor=124.31147455512851
    12327,  //  Sensor=123.26544767506415
    12223,  //  Sensor=122.22864568334265
    12121,  //  Sensor=121.20664868231005
    12019,  //  Sensor=120.18903868788165
    11919,  //  Sensor=119.18931982955952
    11819,  //  Sensor=118.19334846236981
    11721,  //  Sensor=117.21143594839494
    11624,  //  Sensor=116.235625607915
    11527,  //  Sensor=115.26988631803526
    11431,  //  Sensor=114.31255753535744
    11336,  //  Sensor=113.36319882140224
    11242,  //  Sensor=112.42327841442663
    11149,  //  Sensor=111.48920621032975
    11056,  //  Sensor=110.56494458065487
    10965,  //  Sensor=109.64508274733637
    10874,  //  Sensor=108.7359476003904
    10783,  //  Sensor=107.83070843335936
    10694,  //  Sensor=106.9351117476275
    10604,  //  Sensor=106.0428130609981
    10516,  //  Sensor=105.15975967460781
    10428,  //  Sensor=104.2801626844214
    10341,  //  Sensor=103.40755651243214
    10254,  //  Sensor=102.5382176983061
    10168,  //  Sensor=101.67575304206785
    10082,  //  Sensor=100.81766584661838
    9997,   //  Sensor=99.96569945647478
    9912,   //  Sensor=99.11953762448181
    9828,   //  Sensor=98.27744212882403
    9744,   //  Sensor=97.4418146617794
    9661,   //  Sensor=96.60856048887564
    9578,   //  Sensor=95.78316552307767
    9496,   //  Sensor=94.95825657775644
    9414,   //  Sensor=94.14236111820234
    9333,   //  Sensor=93.32646565864825
    9252,   //  Sensor=92.51846090643679
    9171,   //  Sensor=91.71190358730695
    9091,   //  Sensor=90.91053660308398
    9011,   //  Sensor=90.11244238114426
    8932,   //  Sensor=89.31725551131696
    8853,   //  Sensor=88.52728425793785
    8774,   //  Sensor=87.73788098277942
    8696,   //  Sensor=86.95506729794741
    8617,   //  Sensor=86.17225361311542
    8540,   //  Sensor=85.39600406961908
    8462,   //  Sensor=84.6216064275399
    8385,   //  Sensor=83.84983653036478
    8308,   //  Sensor=83.08120694421737
    8231,   //  Sensor=82.31316783703942
    8155,   //  Sensor=81.54972980763189
    8079,   //  Sensor=80.78629177822438
    8003,   //  Sensor=80.02613737952461
    7927,   //  Sensor=79.26704287405352
    7851,   //  Sensor=78.50960052257673
    7775,   //  Sensor=77.75480026128837
    7700,   //  Sensor=77.0
    7625,   //  Sensor=76.24871601894694
    7550,   //  Sensor=75.49743203789389
    7475,   //  Sensor=74.74800796255255
    7400,   //  Sensor=73.99980277710354
    7325,   //  Sensor=73.25206947444838
    7251,   //  Sensor=72.5062433956272
    7176,   //  Sensor=71.76041731680601
    7102,   //  Sensor=71.01627146172585
    7027,   //  Sensor=70.27258603191031
    6953,   //  Sensor=69.52943196416827
    6879,   //  Sensor=68.78720417769343
    6804,   //  Sensor=68.04497639121857
    6731,   //  Sensor=67.30530011860233
    6657,   //  Sensor=66.56578843208273
    6583,   //  Sensor=65.82675504111596
    6509,   //  Sensor=65.08818979062019
    6435,   //  Sensor=64.34966201983669
    6361,   //  Sensor=63.611646267313574
    6287,   //  Sensor=62.87363051479046
    6214,   //  Sensor=62.13547060097673
    6140,   //  Sensor=61.39722574275701
    6066,   //  Sensor=60.65885139342571
    5992,   //  Sensor=59.919928640248315
    5918,   //  Sensor=59.18100588707092
    5844,   //  Sensor=58.44124031502726
    5770,   //  Sensor=57.70120130587174
    5696,   //  Sensor=56.96054686150758
    5622,   //  Sensor=56.21860092536465
    5548,   //  Sensor=55.476654989221714
    5473,   //  Sensor=54.73253489635838
    5399,   //  Sensor=53.9881642966142
    5324,   //  Sensor=53.242434572939565
    5250,   //  Sensor=52.49522378792536
    5175,   //  Sensor=51.74775557639031
    5100,   //  Sensor=50.996844791693476
    5025,   //  Sensor=50.24593400699664
    4949,   //  Sensor=49.49138026929523
    4874,   //  Sensor=48.73505233905808
    4798,   //  Sensor=47.977243199298364
    4722,   //  Sensor=47.215852441631945
    4645,   //  Sensor=46.454461683965526
    4569,   //  Sensor=45.68785711537803
    4492,   //  Sensor=44.920850875769766
    4415,   //  Sensor=44.15012880830348
    4338,   //  Sensor=43.376734517842486
    4260,   //  Sensor=42.60160071066088
    4182,   //  Sensor=41.82136550768396
    4104,   //  Sensor=41.04113030470703
    4025,   //  Sensor=40.25352364162701
    3947,   //  Sensor=39.46550676559768
    3867,   //  Sensor=38.67190570937394
    3788,   //  Sensor=37.87546712409127
    3708,   //  Sensor=37.07534777882923
    3627,   //  Sensor=36.26977598506799
    3546,   //  Sensor=35.46253257893403
    3465,   //  Sensor=34.647044398279036
    3383,   //  Sensor=33.83155621762403
    3301,   //  Sensor=33.00571302083172
    3218,   //  Sensor=32.17945299559238
    3135,   //  Sensor=31.34651420039443
    3051,   //  Sensor=30.511722417796822
    2967,   //  Sensor=29.669266122186112
    2882,   //  Sensor=28.822242171330306
    2797,   //  Sensor=27.9686890386034
    2711,   //  Sensor=27.108645759492767
    2624,   //  Sensor=26.24285941547856
    2537,   //  Sensor=25.368760020157474
    2449,   //  Sensor=24.489433811850958
    2360,   //  Sensor=23.600371544258422
    2271,   //  Sensor=22.70611086248834
    2180,   //  Sensor=21.801039372835607
    2089,   //  Sensor=20.890267036129067
    1997,   //  Sensor=19.968224727112624
    1904,   //  Sensor=19.039217647620948
    1810,   //  Sensor=18.099025270944775
    1715,   //  Sensor=17.1497895419703
    1619,   //  Sensor=16.190324443506842
    1522,   //  Sensor=15.218774482012682
    1424,   //  Sensor=14.238936556205598
    1324,   //  Sensor=13.2390626858366
    1223,   //  Sensor=12.232727335181062
    1120,   //  Sensor=11.203935653506004
    1017,   //  Sensor=10.168998970331117
    911,    //  Sensor=9.114855197689048
    805,    //  Sensor=8.04739957102209
    697,    //  Sensor=6.967235206299286
    586,    //  Sensor=5.863787882583192
    475,    //  Sensor=4.749656225945663
    361,    //  Sensor=3.6127927064819096
    246,    //  Sensor=2.456306769304698
    129,    //  Sensor=1.2855336232221874
    9,      //  Sensor=0.08535248441691645
    -113,   //  Sensor=-1.135292153573689
    -236,   //  Sensor=-2.37482931825508
    -364,   //  Sensor=-3.645701750527924
    -493,   //  Sensor=-4.941968458600362
    -625,   //  Sensor=-6.261899052616535
    -760,   //  Sensor=-7.608613108306915
    -899,   //  Sensor=-8.99771419239021
    -1041,  //  Sensor=-10.41883510283085
    -1186,  //  Sensor=-11.874696068185434
    -1336,  //  Sensor=-13.368599531824122
    -1489,  //  Sensor=-14.904187882310922
    -1648,  //  Sensor=-16.489419516827034
    -1812,  //  Sensor=-18.12686970595048
    -1981,  //  Sensor=-19.817694563474813
    -2156,  //  Sensor=-21.566984467371114
    -2337,  //  Sensor=-23.378499679585293
    -2525,  //  Sensor=-25.259553783024458
    -2721,  //  Sensor=-27.2174811623955
    -2925,  //  Sensor=-29.262172958721084
    -3140,  //  Sensor=-31.410741809968773
    -3366,  //  Sensor=-33.66551663227616
    -3603,  //  Sensor=-36.03694537962716
  };

  index = (ADReading >> 2) - 13;  // we only use the top 8 bits of the A/D reading for our table lookup so the table isn't enormous, then interpolate with the 2 LSBs below.
  SensorDataFromArray = float(SensorTable[index]);
  SensorReading = SensorDataFromArray / 100.0;

  // Now interpolate for the 2 LSB of the AD reading
  TwoLSB = ADReading & 0x03;

  // Now interpolate between table points using the 2 least significant bits of the A/D reading
  if (TwoLSB == 0) {
    return SensorReading;
  }
  NextSensorDataFromArray = float(SensorTable[index + 1]);
  if (TwoLSB == 2) {
    return (float)(SensorDataFromArray + NextSensorDataFromArray) / 200.0;
  }
  if (TwoLSB == 1) {
    return (float)(SensorDataFromArray + NextSensorDataFromArray + NextSensorDataFromArray + NextSensorDataFromArray) / 400.0;
  }
  if (TwoLSB == 3) {
    return (float)(SensorDataFromArray + SensorDataFromArray + SensorDataFromArray + NextSensorDataFromArray) / 400.0;
  }
  return 0;
}


//  *****************************************************************************************************************
//  Host mode functions           Host mode functions              Host mode functions           Host mode functions
//  *****************************************************************************************************************

/**
    @brief  DecStrLongLong() - Convert a unsigned integer to a C string scaled by 0.1 ^ n by decimal point placement. Intended for UNIX int64 date after 2038.
    A 64 bit value can convert up to 20 decimal digits.
    @param val - unsigned long long - Value to print, if cnt isn't large enough val is clipped.
    @param str - char* - String to fill with decimal value with decimal point placed where specified
    @param cnt - unsigned integer - Count of digits to print (not .)
    @param dp - unsigned integer - Digits to right of decimal point

    @retval dig - Number of characters in decimal string (1-21)
*/
/*
  unsigned DecStrLongLong(uint64_t val, char* str, unsigned cnt, unsigned dp)
  {
  if (cnt < 1) {
    return (0);
  }
  else if (cnt > 20) {
    cnt = 20;
  }

  if (dp > 7) {
    dp = 7;
  }

  unsigned dig = cnt;
  uint64_t max = 10;  // initialize smallest base 10 decade
  uint64_t nval = 0;

  // Calculate the max value of val
  while ((--dig > 0) && (val >= max)) max *= 10; //  dig will be the digits you don't need (per the length cnt)

  if (val >= max) {
    val = max - 1;   //  Clip val to maximum number based on cnt digits.
  }

  cnt -= dig;   // subtract the number of digits you don't need
  if (cnt < 1) cnt = 1; // we always need at least one digit.
  dig = cnt;
  str[cnt] = '\0';  // stick string terminator at end of the string
  if (dp != 0)
  {
    if (cnt <= dp) {
      cnt = dp + 1;
    }

    if (++cnt < 3) {
      cnt = 3;
    }
    str[cnt] = '\0';  // place end of string delimiter
    dig = cnt;
    do      //  start at least significant digit
    {
      nval = val / 10;
      str[--cnt] = (char)(val - (nval * 10)) + '0';   // get low order
      val = nval;
    } while (--dp > 0);
    str[--cnt] = '.';
  }

  do
  {
    nval = val / 10;
    str[--cnt] = (char)(val - (nval * 10)) + '0';
    val = nval;
  } while (cnt > 0);

  return (dig);
  } */


void PrintReturnLinefeed() {
  Serial.print("\r\n");
}


void PrintPrompt() {
  Serial.print("\r\n>");
}


void PrintComma() {
  Serial.print(",");
}


void PrintQuestionMarks() {
  Serial.print("??\n");
}


/**
   @brief  Prints out the serial port a single line of measurements formatted as a line in a csv file.

   @details

   @note

   @param[in] measurement

   @retval none
*/
void print_record(Measurement& measurement) {  // CSV format record dump
  Serial.print(measurement.Record.RecordIndex);
  PrintComma();
  Serial.print(measurement.Record.Status, HEX);
  PrintComma();
  Serial.print(measurement.Record.current_time_ms);
  PrintComma();
  float alt;
  alt = measurement.Record.Altitude * METERS_TO_FEET;
  if (alt < 0) {
    alt = 0;
  }
  Serial.print(alt);
  PrintComma();
  Serial.print(measurement.Record.Temperature);
  PrintComma();
  if (measurement.Record.RecordIndex == 0) {
    Serial.print(((float)measurement.InitRecord.SeaLevelPressureX4) / 4.0);
  } else {
    Serial.print(measurement.Record.LightVoltage);
  }
  PrintComma();
  Serial.print(measurement.Record.AccelerationX_g);
  PrintComma();
  Serial.print(measurement.Record.AccelerationY_g);
  PrintComma();
  Serial.print(measurement.Record.AccelerationZ_g);
  PrintReturnLinefeed();
}


/**
   @brief   Prints out the serial port a single line of initial measurements & parameters formatted as a line in a csv file.

   @details

   @note

   @param[in]  measurement

   @retval  none
*/
void PrintInitRecord(Measurement& measurement) {  // CSV format record dump
  uint32_t ShortEEPROMTime;

  Serial.print(measurement.Record.RecordIndex);
  PrintComma();
  Serial.print(measurement.Record.Status, HEX);
  PrintComma();
  Serial.print(measurement.Record.current_time_ms);
  PrintComma();
  Serial.print(measurement.Record.Altitude * METERS_TO_FEET);
  PrintComma();
  Serial.print(measurement.Record.Temperature);
  PrintComma();
  Serial.print(((float)measurement.InitRecord.SeaLevelPressureX4) / 4.0);
  PrintComma();
  ShortEEPROMTime = measurement.InitRecord.LinuxDateTime & 0xffffffff;
  Serial.print(ShortEEPROMTime);
  PrintComma();
  Serial.print(measurement.InitRecord.spare);
}


/**
   @brief

   @details

   @note

   @param[in]
   @param[in]
   @param[out]

   @retval
*/
void PrintCSVFile(uint32_t StartEEPROMAddress) {  // CSV format file dump
  uint32_t i;
  uint32_t ShortEEPROMTime;
  uint16_t StatusField;
  uint32_t LastCurrent_time_ms;
  bool FirstPassThroughLoop;

  //   Header
  Serial.print(F(",,with ver. ,"));  //  Header for the CSV formatted data dump
  Serial.println(VersionString);
  Serial.print(F(",,Mia Linux time,"));
  EEPROM.get(MCU_EEPROM_ADDR_DEFAULT_TIME_s, EEPROMTime);  // Date stamp for the CSV formatted data dump
  ShortEEPROMTime = (int32_t)(EEPROMTime & 0xffffffff);
  Serial.print(ShortEEPROMTime);
  PrintReturnLinefeed();

  // Initial conditions
  i = StartEEPROMAddress;
  Serial.println(F("Initial conditions"));
  Serial.println(F("Index,Status,Milliseconds,Field Alt,Temp,Sea Level Pres,Flight Linux Time,spare,Latitude,Longitude"));  //  Dump a CSV file of the logs
  readByteArray(i, current_measurement.Bytes, sizeof(current_measurement.Record));
  i = i + sizeof(current_measurement.Record);
  PrintInitRecord(current_measurement);

  // Now check for location initial record.
  uint8_t LocIndex;
  char LocCharacter;
  readByteArray(i, current_measurement.Bytes, sizeof(current_measurement.Record));
  i = i + sizeof(current_measurement.Record);
  if ((current_measurement.Bytes[2] == (byte)0) && (current_measurement.Bytes[3] == (byte)1)) {  // checking for status = 0x0100
    // We have a location initial record.
    PrintComma();
    LocCharacter = current_measurement.Bytes[4];
    LocIndex = 4;
    while ((LocCharacter != '\0') && (LocIndex < 31)) {
      Serial.print(LocCharacter);
      LocIndex++;
      LocCharacter = (char)current_measurement.Bytes[LocIndex];
    }
  }

  PrintReturnLinefeed();


  // Flight Log output
  Serial.println(F("Index,Status,Milliseconds,Alt,Temp,Light,X,Y,Z"));  //  Column headers for CSV file of the logs

  readByteArray(i, current_measurement.Bytes, sizeof(current_measurement.Record));  //  read just to init LastCurrent_time_ms
  LastCurrent_time_ms = current_measurement.Record.current_time_ms;
  FirstPassThroughLoop = false;
  StatusField = 0;
  while ((StatusField & 0x40) == 0) {  // loop until a 'last record of flight' status is found
    readByteArray(i, current_measurement.Bytes, sizeof(current_measurement.Record));
    i = i + sizeof(current_measurement.Record);
    print_record(current_measurement);

    // Print out warning if it looks like there is a missing end of flight record
    if ((FirstPassThroughLoop == false) && (((current_measurement.Record.current_time_ms - LastCurrent_time_ms) > 60) || (current_measurement.Record.current_time_ms < LastCurrent_time_ms))) {
      Serial.println(F("No landing record"));
    }

    StatusField = current_measurement.Record.Status;
    LastCurrent_time_ms = current_measurement.Record.current_time_ms;
    FirstPassThroughLoop = true;
  }
}



#define USB_VOLTS_THRESHOLD_TO_ADCOUNTS ((37U * 1023U) / 60U)  //  threshold = 3.7 volts * 1023 (full scale counts) / 6 (full scale volts)

/**
   @brief USBPowered

   @details

   @retval boolean TRUE if on USB power, FALSE if not on USB power
*/
bool USBPowered() {
  //return false;  //   DEBUG
  uint16_t USBADRaw = analogRead(USBVbus);
  return (USBADRaw > USB_VOLTS_THRESHOLD_TO_ADCOUNTS);
}


/**
   @brief Capture command line in host mode

   @details Backspace is supported. End of line is a 'return' character. Returned string is zero delimited.

   @bug  This may fail with high character rates that may cause more than one character to be read by Serial.read().

   @param[in]
   @param[in]
   @param[out] receivedChars as global

   @retval none
*/
void CaptureCommandLine() {
  char endMarker = '\r';
  char BackSpace = '\010';  //  control H, 0x08
  char ReceivedCharacter;

  while (Serial.available() > 0 && newData == false) {
    ReceivedCharacter = Serial.read();  //  Need to wrap this is a little queue, de-queue function because when a computer sends a command at
    //  a megabaud we may get more than 1 character. ???  TODO    Change to Serial.readBytes(ReceivedCharacter, 1) ???

    if (ReceivedCharacter != endMarker) {
      if (ReceivedCharacter == BackSpace) {
        if (ReceiveBufferIndex > 0) {  //  backspace support
          ReceiveBufferIndex--;
          Serial.print(F("\010\040\010"));  // Remove the character we just backspaced over
        }
      } else {
        Serial.print(ReceivedCharacter);
        receivedChars[ReceiveBufferIndex] = ReceivedCharacter;
        ReceiveBufferIndex++;
        if (ReceiveBufferIndex >= numChars) {
          ReceiveBufferIndex = numChars - 1;
        }
      }
    } else {
      receivedChars[ReceiveBufferIndex] = '\0';  // terminate the string   // 'return' support
      newData = true;
    }
    if (!USBPowered()) {  //   If the USB cable has been pulled, we go back to flight mode
      //SetUpMiaFromMCUEEPROM();
      OperationalMode = AllOperationalModes::FlightMode;
    }
    return;
  }
}


/**
   @brief Convert a hexidecimal character to a 4 bit integer

   @note

   @param[in]  hex character

   @retval 
*/
int HexToNibble(char hexChar) {
  if (hexChar >= '0' && hexChar <= '9') {
    return hexChar - '0';
  } else if (hexChar >= 'A' && hexChar <= 'F') {
    return hexChar - 'A' + 10;
  } else if (hexChar >= 'a' && hexChar <= 'f') {
    return hexChar - 'a' + 10;
  } else {
    return 0;  // Invalid hex character
  }
}



/**
   @brief Convert a hexidecimal string to an integer

   @details  Limited to a 32 bit integer

   @note

   @param[in]  ptr, pointer to location of hex characters in receivedChars
   @param[in] Length, the number of hex characters to convert to an integer
   @param[out]  the resulting integer

   @retval
*/
uint32_t HexidecimalStringToInteger(char* ptr, int Length) {
  uint32_t ConvertedInteger;
  int8_t j;
  for (j = 0; j < Length; j++) {
    ConvertedInteger = ConvertedInteger << 4;
    ConvertedInteger = ConvertedInteger | HexToNibble(*ptr);
    ptr++;
  }
  return ConvertedInteger;
}


/**
   @brief

   @details

Host commands:
  a               Get the last maximum altitude, displayed in feet above launch field altitude.
  l               List all the flight records in logging EEPROM.
  d n             Dump a comma seperated value file of a flight number. The flight numbers corraspond to the flight numbers in the 'l' command.
  c 1234          Clear logging EEPROM memory. The 1234 is meaningless but is required to prevent an easy typo from clearing all your data.
  s               Show the URL for a date/time web site. In this case we show www.unixtimestamp.com
  s nnnnnnnnnn    Set date/time to n...n. Must use UNIX intarnal time format, seconds from January 1, 1970. See www.unixtimestamp.com
  o llll...llll   Set latitude and longitude. This is simply capturing the text, no itepetation of the data is done. 23 characters maximum.
  u               Display user configuration long word.
                       Bit  0  BMP390 pressure/temperature sensor
                       Bit  1  Optical sensor with VEMT2523SLX01 and 1.00kΩ pull up resistor
                       Bit  2  Memsic MC3416 Accelerometer
                       Bit  3  Kionix KX134 Accelerometer
                       Bit  4  1: 10 kΩ thermistor, muRata NXFT15XH103FEAB050, plugged in P3 (Mia provides a 10.0 k 1% pull up ressistor)   0:Record voltage from P3
                       Bit  5  BMP581 pressure/temperature sensor
                       Bit  6  For output digital 9, SounderRequiresFrequency
                       Bit  7  For output digital 9, 0=Servo output, 1=buzzer output
  u hhhhhhhh      Set user configuration long word. See bit assignments above. Must be exactly 8 hex characters of data.
  p               Read byte parameters.
  p hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh        Set byte parameters. Must be exactly 32 hex characters of data. The first byte is at MCU EEPROM address 200 and each byte
                                            after that is at the next larger address.
  w               Read word parameters.
  w hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh        Set word parameters. Must be exactly 32 hex characters of data. The first word in the parameter is at MCU EEPROM address 231:230.
                                            Each word after that is at DECREASING addresses.
  t nnnnnn        Set SeaLevel Pressure in Pascals, not millibars. A pressure of 1013.25mb is entered as 101325 with no decimal point.
  e               Exit to charge mode.
  h               Help command.



   @note

   @param[in]
   @param[in]
   @param[out]

   @retval
*/
void ProcessCommand() {
  uint32_t LastStartingIndex;
  uint32_t i;
  int32_t ShortEEPROMTime;
  uint16_t FlightNumber;
  uint16_t Status;
  bool ReadyForEndRecord;
  float MaxAltitude_m;
  uint16_t NumberOfRecordsInFlight;
  uint32_t LastEEPROMAddressInLog;
  uint8_t x;
  int16_t j;

  if (newData == true) {

    PrintReturnLinefeed();
    //  isolate command character and parameter
    CommandCharacter = receivedChars[0];
    if ((ReceiveBufferIndex >= 3) && (CommandCharacter != 'o')) {  // The o command is just a string of text (lat & lon), so don't convert to numerical argument.
      Argument1_i = atol(receivedChars + 2);                       // This will need to support long long (64 bit) before the year 2038 for the date.
    } else {
      Argument1_i = 0;
    }

    //  Clear EEPROM   example: c 1234   (really all we do is reset the address pointer back to the beginning)
    if (CommandCharacter == 'c') {
      if (Argument1_i == 1234) {
        EEPROM.put(MCU_EEPROM_EXT_EEPROM_ADDR_START, (uint32_t)0);
        //Serial.println(F("Log cleared"));
      } else {
        PrintQuestionMarks();
      }
    }

    // list all flights
    else if (CommandCharacter == 'l') {
      Serial.print(F("Flight No., Field Alt (ft), Temperature (F), Sea Level Pres (mb), Flight Date, Number of records, Peak Altitude"));
      FlightNumber = 1;
      i = 0;
      Status = 0;
      MaxAltitude_m = 0.0;
      EEPROM.get(MCU_EEPROM_EXT_EEPROM_ADDR_START, LastEEPROMAddressInLog);  //  get the address where the next log would start
      while ((Status != 0xffff) && (i < ExternalEEPROMSizeInBytes) && (i < LastEEPROMAddressInLog)) {
        readByteArray(i, current_measurement.Bytes, sizeof(current_measurement.Record));
        Status = current_measurement.InitRecord.Status;
        if (((current_measurement.InitRecord.Status & 0x1) == 1) && (current_measurement.InitRecord.Status != 0xffff)) {  // if begin record...
          MaxAltitude_m = 0.0;
          if (FlightNumber <= 9) {
            Serial.print(" ");
          }
          Serial.println("");
          Serial.print(FlightNumber);
          Serial.print(F(":           "));
          Serial.print(current_measurement.InitRecord.Altitude * METERS_TO_FEET);
          Serial.print(F("      "));
          Serial.print(current_measurement.InitRecord.Temperature);
          Serial.print(F("      "));
          Serial.print((float)current_measurement.InitRecord.SeaLevelPressureX4 / 4.0);
          Serial.print(F("        "));
          ShortEEPROMTime = current_measurement.InitRecord.LinuxDateTime & 0xffffffff;
          Serial.print(ShortEEPROMTime);
          Serial.print(F("       "));
          LastStartingIndex = i;
          FlightNumber++;
          ReadyForEndRecord = true;
        } else if (((current_measurement.InitRecord.Status & 0x40) == 0x40) && (current_measurement.InitRecord.Status != 0xffff) && ReadyForEndRecord) {  //  if end record
          //  found last record of flight
          NumberOfRecordsInFlight = (i - LastStartingIndex) / sizeof(current_measurement.Record);
          Serial.print(NumberOfRecordsInFlight);

          if (NumberOfRecordsInFlight < 100) {  //  line up our columns
            Serial.print(" ");
          }
          if (NumberOfRecordsInFlight < 1000) {
            Serial.print(" ");
          }

          Serial.print("             ");
          Serial.print(int(MaxAltitude_m * METERS_TO_FEET));
          ReadyForEndRecord = false;
        } else {
          if (current_measurement.Record.Altitude > MaxAltitude_m) {
            MaxAltitude_m = current_measurement.Record.Altitude;
          }
        }
        i = i + sizeof(current_measurement.Record);
      }  //  end of while loop
      Serial.print(F("\n\rBytes used:"));
      EEPROM.get(MCU_EEPROM_EXT_EEPROM_ADDR_START, LastStartingIndex);
      Serial.print(LastStartingIndex);
      Serial.print(F("    Bytes left:"));
      Serial.print(ExternalEEPROMSizeInBytes - LastStartingIndex);
      Serial.print(F("    Percent used:"));
      Serial.print((LastStartingIndex * 100) / ExternalEEPROMSizeInBytes);
    }

    // Dump CSV file format of flight
    else if ((CommandCharacter == 'd') && (Argument1_i > 0)) {

      i = 0;
      if (Argument1_i != 1) {  //  if we want the first flight, it is always at address zero....

        // Otherwise lets start searching for the flight we want.
        i = i + sizeof(current_measurement.Record);
        FlightNumber = 1;
        while (FlightNumber != (uint16_t)Argument1_i) {
          readByteArray(i, current_measurement.Bytes, sizeof(current_measurement.Record));
          if (((current_measurement.InitRecord.Status & 0x40) == 0x40)) {
            // end of flight record found
            FlightNumber++;
          }
          i = i + sizeof(current_measurement.Record);
        }  //  end of while
      }
      PrintCSVFile(i);
    }

    // set date
    else if ((CommandCharacter == 's') && (Argument1_i > BIRTH_TIME_OF_THIS_VERSION)) {
      EEPROM.put(MCU_EEPROM_ADDR_DEFAULT_TIME_s, Argument1_i);
    }

    // print date link for 'set date'
    else if ((CommandCharacter == 's') && (Argument1_i == 0)) {
      Serial.println(F("www.unixtimestamp.com"));
    }

    // set barametric pressure at sea level. Units: pascals (not hPa or millibars)
    else if ((CommandCharacter == 't')) {
      if (ReceiveBufferIndex >= 3) {
        if ((Argument1_i > ((uint32_t)MINIMUM_SEA_LEVEL_PRESSURE_hPa * 100)) && (Argument1_i < ((uint32_t)MAXIMUM_SEA_LEVEL_PRESSURE_hPa * 100))) {
          SeaLevelPressure_hPa = (float)Argument1_i / 100.0;
          EEPROM.put(MCU_EEPROM_ADDR_DEFAULT_SEALEVELPRESSURE_HP, SeaLevelPressure_hPa);
        } else {
          PrintQuestionMarks();
        }
      } else {
        //Serial.println(F("OK"));  //  Pressure value appears to be there
      }
    }

    // set latitude and longitude (really just a string of text but should be <latitude>,<longitude>)
    else if ((CommandCharacter == 'o')) {
      uint16_t IndexToLatLon;

      IndexToLatLon = 2;  // the text is copied from receivedChars + 2 to MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + 4
      while ((receivedChars[IndexToLatLon] != 0) && (IndexToLatLon < (27U + 2U))) {
        EEPROM.put(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + IndexToLatLon + 2, receivedChars[IndexToLatLon]);  //  Starting at MCU EEPROM address plus 4 to skip over record and status fields.
        IndexToLatLon++;
      }
      EEPROM.put(MCU_EEPROM_ADDR_LATITUDE_LONGITUDE + IndexToLatLon + 2, '\0');
    }

    // Read last max altitude
    else if ((CommandCharacter == 'a')) {
      EEPROM.get(MCU_EEPROM_LAST_MAXIMUM_ALTITUDE, maxAltitude_m);  // This max altitude we are reading is already compensated for field altitude, because we could now be at a different location and field altitude.
      Serial.print(maxAltitude_m * METERS_TO_FEET);
      Serial.println(" ft.");
    }

    // Read and set altitude threshold for high current output.
    else if ((CommandCharacter == 'b')) {
      if (Argument1_i != 0) {
        EEPROM.put(MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft, (float)Argument1_i);  //  AltitudeHighCurrentOut is unique in that it is stored in feet.
      } else {
        EEPROM.get(MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft, maxAltitude_m);  // This is really in feet.
        Serial.print(maxAltitude_m);
        Serial.println(" ft.");
      }
    }

    // Read and set servo positions, buzzer, low power byte parameters, and spare bytes. 16 bytes maximum, all hexidecimal characters, no spaces.
    else if ((CommandCharacter == 'p')) {
      uint8_t MyNumber;
      if (ReceiveBufferIndex == 1) {
        // print out current EEPROM values
        for (i = 0; i < 16; i++) {
          EEPROM.get(MCU_EEPROM_SERVO_PRE_LAUNCH + i, x);
          SerialPrint2DigitHex(x);
        }
        Serial.println("");
      } else {
        if (ReceiveBufferIndex != 34) {
          PrintQuestionMarks();
        } else {
          char* pointer = receivedChars;
          pointer++;  //   Increment past the command and space characters.
          pointer++;
          for (j = 0; j < 16; j++) {
            MyNumber = HexidecimalStringToInteger(pointer, 2);  // Pass in the pointer pointing to the beginning of the hex string.
            EEPROM.put(MCU_EEPROM_SERVO_PRE_LAUNCH + j, MyNumber);
            pointer++;
            pointer++;
          }
        }
      }
    }

    // Read and set buzzer frequencies and spare words. 8 words maximum, all hexidecimal characters, no spaces
    else if ((CommandCharacter == 'w')) {
      uint8_t MyNumber;
      if (ReceiveBufferIndex == 1) {
        // print out current EEPROM values
        for (j = 15; j >= 0; j--) {  //  Start at MSB so words come out in normal significants.
          EEPROM.get(MCU_EEPROM_BUZZER_1_FREQ + j, x);
          SerialPrint2DigitHex(x);
        }
        Serial.println("");
      } else {
        if (ReceiveBufferIndex != 34) {
          PrintQuestionMarks();
        } else {
          char* pointer = receivedChars;
          pointer++;
          pointer++;
          for (j = 15; j >= 0; j--) {
            MyNumber = HexidecimalStringToInteger(pointer, 2);  // pass in the pointer pointing to the beginning of the hex string
            //Serial.println(MyNumber, HEX);
            EEPROM.put(MCU_EEPROM_BUZZER_1_FREQ + j, MyNumber);
            pointer++;
            pointer++;
          }
        }
      }
    }

    // Read and set UserConfiguration bits. 1 32 bit word, 8 hexidecimal characters, no spaces
    else if ((CommandCharacter == 'u')) {
      uint32_t MyNumber;
      if (ReceiveBufferIndex == 1) {
        // print out current EEPROM values
        for (j = 3; j >= 0; j--) {  //  Start at MSB so bytes come out in normal significants.
          EEPROM.get(MCU_EEPROM_USER_CONFIGURATION + j, x);
          SerialPrint2DigitHex(x);
        }
        Serial.println("");
      } else {
        if (ReceiveBufferIndex != 10) {
          PrintQuestionMarks();
        } else {
          char* pointer = receivedChars;
          pointer++;  //   Increment past the command and space characters.
          pointer++;
          MyNumber = HexidecimalStringToInteger(pointer, 8);  // Pass in the pointer pointing to the beginning of the hex string.
          MyNumber = MyNumber | UserConfigurationRequiredFeatures;
          //Serial.println(MyNumber);
          EEPROM.put(MCU_EEPROM_USER_CONFIGURATION, MyNumber);
        }
      }
    }

    // print help
    else if ((CommandCharacter == 'h') || (CommandCharacter == '?')) {
      Serial.println(F("Mia help\n"));
      Serial.println(F("Cmd    Desc"));
      Serial.println(F("a      Last max alt"));
      Serial.println(F("l      List flight records in log"));
      Serial.println(F("d n    Dump file of flight #"));
      Serial.println(F("c 1234 Clear log (1234 is required)"));
      Serial.println(F("s      Show date/time URL"));
      Serial.println(F("s n..n Set date/time"));
      Serial.println(F("o llll Set lat,lon"));      //
      Serial.println(F("u hhhh Set user config"));  //
      Serial.println(F("p hhhh Set byte param"));   //
      Serial.println(F("w hhhh Set word param"));
      Serial.println(F("t n..n Set SeaLevel Pres. (Pa)"));
      Serial.println(F("e      Exit"));
      Serial.println(F("h      Help"));
      Serial.println(F("ERROR  ??"));
    }

    // exit
    else if (CommandCharacter == 'e') {
      //FlightModePhaseIndex = 0;
      OperationalMode = AllOperationalModes::BatteryChargeOnly;
      //current_measurement.Record.Status = 0x0040;
    }

    else {
      PrintQuestionMarks();
    }

    newData = false;
    ReceiveBufferIndex = 0;

    if (CommandCharacter != 'e') {
      PrintPrompt();  //  prompt
    }
  }  //  end of if(NewData.....
}



//**************************************************************************************************
//**************************************************************************************************
//   Mia Main Arduino Loop functions
//**************************************************************************************************
//**************************************************************************************************


/**
   @brief  Displays an 'L' in the upper right of the OLED

   @param[in] none

   @retval  none
*/
void DisplayLandedIndication() {
  display.clearField(120, 0, 1);
  display.print(F("L"));  //   landed indication in top right corner of display
}


/**
   @brief  Prints on the serial port a 2 digit Hex integer, adding leading zero if necessary

   @details

   @note

   @param[in] uint8_t Value to print

   @retval  none
*/
void SerialPrint2DigitHex(uint8_t IntForOutput) {
  if (IntForOutput < 16) {
    Serial.print("0");
  }
  Serial.print(IntForOutput, HEX);
}



/***************************************************************************************************************************
   Mia button functions
 ***************************************************************************************************************************
 */
// Button interface  (supports "click", "double click", and "long click")
#define ButtonClickShortMin 100     // minimum, milliseconds
#define ButtonClickLongMin 950      // minimum, milliseconds
#define ButtonBetweenClicksMax 600  // maximum, milliseconds

#define SHORT_CLICK_FOUND 3U
#define DOUBLE_CLICK_FOUND 5U
#define LONG_CLICK_FOUND 6U
#define BUTTON_PRESSED_AT_RESET 8U
#define BUTTON_DECODE_ERROR 249U
#define BUTTON_STATE_INIT 250U


/**
   @brief  Reads the button code of the DISP button. We detect: one short click, double click, and long click.

   @details

   @note

   @param[in] int8_t ResetButtonDecode  -1 will wait for the button to be released and reset the button decode state machine to 0.

   @retval  uint8_t decoded state of button: click, double click, or long click.
*/
uint8_t ReadDISPButton(int8_t ResetButtonDecode = 0) {
  static uint8_t ReadDISPButtonCallBefore; /**  startup.c should init to 0    */
  static uint8_t DispButtonState;
  if (ResetButtonDecode == -1) {
    DispButtonState = 7;  // Wait for button to be released, no op.
    return 0;
  }
  if (ReadDISPButtonCallBefore == 0) {
    DispButtonState = BUTTON_STATE_INIT;
    ReadDISPButtonCallBefore = 1;
  }
  DispButtonState = ReadButton(N_DispButton, 0, DispButtonState);
  return DispButtonState;
}


/**
    @brief  Reads the second button, USER2, on the second version of Mia. We detect: one short click, double click, and long click.

   @details

   @note

   @param[in] int8_t ResetButtonDecode  TRUE will reset the button decode state machine.

   @retval uint8_t ButtonUSER2State
*/
uint8_t ReadButtonUSER2(int8_t ResetButtonDecode = 0) {
  static uint8_t ReadButtonUSER2CallBefore; /**  startup.c should init to 0    */
  static uint8_t ButtonUSER2State;
  if (ResetButtonDecode == -1) {
    ButtonUSER2State = 7;  // wait for button to be released, no op.
    return 0;
  }
  if (ReadButtonUSER2CallBefore == 0) {
    ButtonUSER2State = BUTTON_STATE_INIT;
    ReadButtonUSER2CallBefore = 1;
  }
  ButtonUSER2State = ReadButton(N_Button2, 0, ButtonUSER2State);
  return ButtonUSER2State;
}


/**
    @brief  Reads either button pin passed in from one of the ReadButtonUSER<n> functions for each button. We detect: one short click, double click, and long click.

    @details
    
    @verbatim

   GLITCH FILTER
     ButtonState: --------------------------------------------------------------_______----------------------
     StateMachine:  0                                                           1      0
                   No press greater than ButtonBetweenClicksMax                pressed shorter than ButtonClickShortMin
     Returns 0 (no press)

   SINGLE CLICK
     ButtonState: --------------------------------------------------------------____________-----------------------------------------------------------------------------
     StateMachine:  0                                                           1           2                                         3                             0    returns to 0 after the 3 is read
                   No press greater than ButtonBetweenClicksMax               pressed longer than                                     ^ Returns single click after ButtonBetweenClicksMax
                                                                              ButtonClickShortMin and
                                                                              shorter than ButtonClickLongMin
     Returns 3 (single click)

   DOUBLE CLICK
     ButtonState: --------------------------------------____________________-------------------------------------------------______________________________--------------------------------------
     StateMachine:  0                                   1                   2                                                4                             5                      0
                   No press greater than                pressed longer than                Released shorter                 pressed longer than            ^ Returns double click after Button released
                   ButtonBetweenClicksMax               ButtonClickShortMin and            than ButtonBetweenClicksMax      ButtonClickShortMin and
                                                        shorter than ButtonClickLongMin                                     shorter than ButtonClickLongMin
                                                                                                                            If longer than ButtonClickLongMin we go to state 7 (report nothing)
                                                                                                                            If shorter than ButtonClickShortMin we go to state 0 (report nothing)
     Returns 5 (double click)


   Long CLICK
     ButtonState: ------------------------------------------------------------__________________________________________________-----------------------------------------------------------------------------
     StateMachine:  0                                                         1                                      9     9    6   <- Report long click
                   No press greater than ButtonBetweenClicksMax               pressed longer than                    ^     ^ Continue waiting
                                                                              ButtonClickLongMin                     | Wait for end of click after ButtonClickLongMin
     Returns 6 (long click)

    We do not use this feature: If we are called the first time and the button is pressed, we return 8. This allows detecting one of the user buttons pressed right out of reset.

    @endverbatim

    @param ButtonIOPin Arduino pin name
    @param PressedState Wheather the button is a 0 or a 1 when pressed

    @retval Detected button function

*/
uint8_t ReadButton(uint8_t ButtonIOPin, uint8_t PressedState, uint8_t ButtonState) {
  static unsigned long ButtonPressState1StartTime;     //  Time stamp state 1 started in milliseconds
  static unsigned long ButtonPressState4StartTime;     //  Time stamp state 4 started in milliseconds
  static unsigned long ButtonBetweenPressesStartTime;  //  Duration we were in state 2 in milliseconds
  unsigned long Duration;
  unsigned long CurrentTime;
  unsigned long ElapsedTime;

  uint8_t CurrentPress = 0x01 & ((PressedState == 0) ? digitalRead(ButtonIOPin) : !digitalRead(ButtonIOPin));  // reguardless of button phase, report 1 for unpressed, 0 for pressed.
  //Serial.print(F("Button "));
  //Serial.print(CurrentPress);
  //Serial.print(F("    "));
  // First, check to see if button is pressed out of reset  (This is not useful due to too many required delays after reset.)   REMOVE???
  if (ButtonState == BUTTON_STATE_INIT) {
    if (CurrentPress == 0) {
      return BUTTON_PRESSED_AT_RESET;
    } else {
      return 0U;
    }
  }
  CurrentTime = millis();
  //Serial.print(F("State "));
  //Serial.print(ButtonState);
  //Serial.println(F("    "));
  switch (ButtonState) {
    case 0U:
      //Serial.print(F("C0"));
      if (CurrentPress == 1) {
        return 0;  // button wasn't pressed and still isn't, stay in ButtonState 0
      } else {
        // so button was just pressed because we were in ButtonState 0
        ButtonPressState1StartTime = CurrentTime;
        //Serial.print(F("Sta 1"));
        return 1;  //  return that button is down but we don't have a judgment on clicks yet
      }
      break;

    case 1U:
      if (CurrentPress == 1) {
        // button was released. We need to find out how long it was pressed.
        Duration = CurrentTime - ButtonPressState1StartTime;
        if (Duration < ButtonClickShortMin) {
          // Duration is shorter than a click time, so it was a glitch
          return 0U;
        } else {
          // so this is a valid short click
          ButtonBetweenPressesStartTime = CurrentTime;  // we must remember this for state
          return 2U;                                    // We don't know if another short click is coming, state 2 will monitor
        }
      } else {
        // so button is still pressed
        ElapsedTime = CurrentTime - ButtonPressState1StartTime;
        if (ElapsedTime > ButtonClickLongMin) {
          //return LONG_CLICK_FOUND;  // Return long click
          return 9U;  // Return waiting for button to be released
        } else {
          return 1U;
        }
      }
      break;

    case 2U:
      if (CurrentPress == 1) {
        // so the button is still released, lets see if we have exceeded the maximum time between clicks
        ElapsedTime = CurrentTime - ButtonBetweenPressesStartTime;
        if (ElapsedTime > ButtonBetweenClicksMax) {
          ButtonState = 0;  // So just a single click
          return SHORT_CLICK_FOUND;
        } else {
          // so we are still waiting to see if there is another click, so stay in state 2
          return 2U;  // no new news yet
        }
      } else {
        // So we have a new button press,
        ElapsedTime = CurrentTime - ButtonBetweenPressesStartTime;
        if (ElapsedTime <= ButtonBetweenClicksMax) {
          // this second click started within the double click time, so on to detecting a double click or not.
          ButtonPressState4StartTime = CurrentTime;
          return 4U;
        } else {
          // well, this is unlikely, our state machine hasn't had time to detect this before now and return to state 0.
          // So, we definitely have to report a short click. The state machine will then return to state 0 and then detect the button
          //   down and start a little late timing this new second click.
          return SHORT_CLICK_FOUND;
        }
      }
      break;

    case SHORT_CLICK_FOUND:  // case 3
      // Having reported a short click, we just return to state 0 and wait for a new button press
      return 0U;
      break;

    case 4U:
      // we are now qualifing a second click to be a short click to see if it is a double click
      ElapsedTime = CurrentTime - ButtonPressState4StartTime;
      if (CurrentPress == 1) {
        // button just released
        if (ElapsedTime < ButtonClickShortMin) {
          // glitch click, report no button activity
          return 0U;
        } else {
          if (ElapsedTime > ButtonClickLongMin) {
            // Second click is too long, report nothing and wait for button to return high
            return 7U;
          } else {
            // so the press duration is just right, report double click
            return DOUBLE_CLICK_FOUND;
          }
        }
      } else {
        return 4U;
      }
      break;

    case DOUBLE_CLICK_FOUND:  // case 5
      // Having reported a double click, we just return to state 0 and wait for a new button press
      return 0U;
      break;

    case LONG_CLICK_FOUND:  // case 6
      // Having reported a long click, we just return to state 0 and wait for a new button press
      return 0U;
      break;

    case 7U:
      // We are just waiting for the button to be un-pressed
      if (CurrentPress == 1) {
        //  finally unpressed!
        return 0U;
      } else {
        // still pressed, continue waiting
        return 7U;
      }
      break;

    case 9U:  // case 9
      // Having detected a long click, we just wait for release
      // We are just waiting for the button to be un-pressed
      if (CurrentPress == 1) {
        //  finally unpressed!
        return LONG_CLICK_FOUND;
      } else {
        return 9U;
      }
      break;

    case BUTTON_PRESSED_AT_RESET:  //  case 8
      // Just reported button pressed at reset, so just wait for button to be released
      return 7U;
      break;

    case BUTTON_STATE_INIT:  //  case 250

      return 0U;
      break;

    case BUTTON_DECODE_ERROR:  // case 249
      // we reported error
      if (CurrentPress == 1) {
        // so if not pressed, we are done, just wait for next press
        return 0U;
      } else {
        // button pressed but we have no idea what has gone wrong in our decoding, so just wait for it to be released
        return 7U;
      }
      break;

    default:
      //Serial.print("Def");
      // we must be having a bad day, we should never get here
      return BUTTON_DECODE_ERROR;
  }
  return ButtonState;
}



//  *****************************************************************************************************************
//  Set Sea Level adjust mode functions           Set Sea Level adjust mode functions              Set Sea Level adjust mode functions
//  *****************************************************************************************************************

/**
   @brief

   @retval
*/
void InitSeaLevelPressureSetMode() {

  OperationalMode = AllOperationalModes::SealevelPressureSetMode;
  display.set2X();
  display.clear();                      // This takes 33ms to run @ 400 kHz I2C speed!
  display.println(F("Set sea level"));  //  takes 1ms plus 0.62ms per character
  display.println(F("pressure mode"));
  delay(1000);
  display.clear();  // This takes 33ms to run @ 400 kHz I2C speed!

  //  See: https://github.com/greiman/SSD1306Ascii/tree/master/examples/SixAdcFieldsWire
  if (SeaLevelPressureSetUpDirection) {
    uint32_t TemporaryPressure;

    // SeaLevelPressure_hPa sometimes get borked, and the decimal part is not xxx.0, xxx.25, xxx.5, or xxx.75 for some reason, we enforce that first.
    TemporaryPressure = (uint32_t)(SeaLevelPressure_hPa * 4);  // Make sure 4x sea level pressure is an integer.
    SeaLevelPressure_hPa = ((float)TemporaryPressure / 4.0);   //  Restore our xxx.0, xxx.25, xxx.5, or xxx.75 units.

    // display UP and current sea level pressure
    display.print(F("UP  "));  //  takes 1ms plus 0.62ms per character

  } else {
    // display DN and current sea level pressure
    display.print(F("DN  "));  //  takes 1ms plus 0.62ms per character
  }
  SeaLevelDisplayFinish();
  //  clear out user clicks
  (void)ReadDISPButton(-1);  //  reset button decode and start fresh
}

void SeaLevelDisplayFinish() {
  display.print(SeaLevelPressure_hPa);
  display.println(F(" mb  "));
  fieldAltitude_m = 0.0;
  maxAltitude_m = PressureToAltitude_m(ReadBMP581LatestPressure(), SeaLevelPressure_hPa);
  display.print(F("Field "));
  display.print((int16_t)(maxAltitude_m * METERS_TO_FEET));
  display.print(F(" ft"));
}


//  *****************************************************************************************************************
//  Altitude set adjust mode functions           Altitude set adjust mode functions               Altitude set adjust mode functions
//  *****************************************************************************************************************

/**
   @brief InitHighCurrentOutAltitude

*/
void InitHighCurrentOutAltitude() {

  OperationalMode = AllOperationalModes::AltitudeForHighCurrentOutput;
  EEPROM.get(MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft, AltitudeHighCurrentOutSetting_ft);  // This altitude is unique in that it is in feet
  display.set2X();
  display.clear();                     // This takes 33ms to run @ 400 kHz I2C speed!
  display.println(F("Set Altitude"));  //  takes 1ms plus 0.62ms per character
  display.println(F("mode in feet"));
  delay(1000);
  display.clear();  // This takes 33ms to run @ 400 kHz I2C speed!

  //  See: https://github.com/greiman/SSD1306Ascii/tree/master/examples/SixAdcFieldsWire
  if (SeaLevelPressureSetUpDirection) {

    // display UP and current sea level pressure
    display.print(F("UP  "));  //  takes 1ms plus 0.62ms per character
    display.print(AltitudeHighCurrentOutSetting_ft);

  } else {
    // display DN and current sea level pressure
    display.print(F("DN  "));  //  takes 1ms plus 0.62ms per character
    display.print(AltitudeHighCurrentOutSetting_ft);
  }
  //  clear out user clicks
  (void)ReadDISPButton(-1);  //  reset button decode and start fresh
}



/**************************************************************************************************
   Arduino loop
 **************************************************************************************************/
//  Default (power up) mode is flight mode
//
//  If in "set sea level" mode, USB power comming or going won't change mode
//  Host mode is entered from charge mode and receiving a "return" (0x0d) character.
//  Host mode ignores buttons
//  Host mode blanks display
//
//  Charge mode blanks display after indicating entry into charge mode (CHARGING...). This is done so that the charger can detect
//        end of charge of a few milliamps, otherwise it’s detecting all of the current for a operating which is about 13 to 15 mA so it’ll never go below 5 milliamps.
//
//            ┌───────┐
//            │ Loop  │
//            └───┬───┘
//                │
//                ▼
//   ┌─────────────────────────┐
//   │      Button Decode      │
//   │                         │
//   │  Set Operational Mode   │
//   └────────────┬────────────┘
//                │
//                ▼
// ┌─────────────────────────────┐
// │   Operational Mode Decode   │
// │                             │
// │         Flight Mode         │
// │     Battery Charge Only     │
// │         Serial Host         │
// │ Sealevel Pressure Set Mode  │
// └─────────────────────────────┘
//
//
//
//
void loop() {

  //uint8_t DirectionCharacter;
  char NewCharacter;
  uint16_t CharactersToRead;
  uint8_t ThisUSER1Click;
  uint8_t ThisUSER2Click;
  static unsigned long OLEDBlankTime;
  float AltitudeDelta;
  float LandingAltitude_m;

  //  TWBR = 2;


  // Make sure, that if we have left flight mode, that we have an end record in our external EEPROM.
  if (((current_measurement.Record.Status & 0x40) != 0x40) && (OperationalMode != AllOperationalModes::FlightMode)) {
    // Then we didn't write an end record.
    if (EepromAddress + 32 > ExternalEEPROMSizeInBytes) {
      EepromAddress = ExternalEEPROMSizeInBytes - 32;
    }
    current_measurement.Record.Status = 0x0040 | 0x2000;  //  Last record of current flight OR Abnormal termination
                                                          // This is the bits: 0: initial record (with field altitude and sea level pressure). 1:PD7_On. 2:BuzzerOn. 3:PD3_TP7. 4:Altitude in feet. 5: Temperature in °C.
                                                          //   6: Last record of current flight   8: Second initial record (with lat & lon)   9-11: Servo position    12: Abnormal termination  14: Apogee detected  15: Landing detected.
    current_measurement.Record.current_time_ms = millis();
    writeByteArray(EepromAddress, current_measurement.Bytes, sizeof(current_measurement.Record));  //  Writing out pre-populated measurement record.
  }

  // Decode USER1 (DISP) button command, if any
  ThisUSER1Click = ReadDISPButton();
  switch (ThisUSER1Click) {

    case SHORT_CLICK_FOUND:
      if (OperationalMode == AllOperationalModes::FlightMode) {

      } else if (OperationalMode == AllOperationalModes::BatteryChargeOnly) {

      } else if (OperationalMode == AllOperationalModes::SerialHost) {

      } else if (OperationalMode == AllOperationalModes::SealevelPressureSetMode) {
        display.clear();  // This takes 33ms to run @ 400 kHz I2C speed!
        if (SeaLevelPressureSetUpDirection) {
          SeaLevelPressure_hPa = SeaLevelPressure_hPa + 0.25;
          // display UP and current sea level pressure
          //display.clearField(0, 0, 1);
          display.print(F("UP  "));  //  takes 1ms plus 0.62ms per character
        } else {
          // display DN and current sea level pressure
          SeaLevelPressure_hPa = SeaLevelPressure_hPa - 0.25;
          //display.clearField(0, 0, 1);
          display.print(F("DN  "));  //  takes 1ms plus 0.62ms per character
        }
        SeaLevelDisplayFinish();

      } else if (OperationalMode == AllOperationalModes::AltitudeForHighCurrentOutput) {  //   Altitude set mode
        display.clear();                                                                  // This takes 33ms to run @ 400 kHz I2C speed!
        if (SeaLevelPressureSetUpDirection) {
          AltitudeHighCurrentOutSetting_ft = AltitudeHighCurrentOutSetting_ft + 20.0;
          // display UP and current sea level pressure
          //display.clearField(0, 0, 1);
          display.print(F("UP  "));  //  takes 1ms plus 0.62ms per character
          display.print(AltitudeHighCurrentOutSetting_ft);
          display.print(F("  "));
        } else {
          // display DN and current sea level pressure
          AltitudeHighCurrentOutSetting_ft = AltitudeHighCurrentOutSetting_ft - 20.0;
          //display.clearField(0, 0, 1);
          display.print(F("DN  "));  //  takes 1ms plus 0.62ms per character
          display.print(AltitudeHighCurrentOutSetting_ft);
          display.print(F("  "));
        }
      } else {
        //error
        //Serial.println(F("click in error"));
        OperationalMode = AllOperationalModes::FlightMode;
        FlightModePhaseIndex = 0;
      }
      break;



    case LONG_CLICK_FOUND:
      noTone(BuzzerOut);
      if (OperationalMode == AllOperationalModes::FlightMode) {
        InitSeaLevelPressureSetMode();
      } else if (OperationalMode == AllOperationalModes::BatteryChargeOnly) {
        InitSeaLevelPressureSetMode();
      } else if (OperationalMode == AllOperationalModes::SerialHost) {
        //  Ignored in serial host mode
      } else if (OperationalMode == AllOperationalModes::SealevelPressureSetMode) {
        // a long click in Sea level pressure Set Mode will Write pressure to EEPROM, reset field altitude, & exit that mode. Depending on USB power we go to flight mode or charge mode
        EEPROM.put(MCU_EEPROM_ADDR_DEFAULT_SEALEVELPRESSURE_HP, SeaLevelPressure_hPa);
        display.set2X();
        display.clear();
        display.print(F("SAVED"));  //  takes 1ms plus 0.62ms per character
        delay(600);
        display.clear();  //  don't want OLED display noise on the power supply when we get field altitude
        delay(50);
        FindFieldAltitude_m();
        ReadDISPButton(-1);  //  reset button decode and start fresh
        if (USBPowered()) {
          OperationalMode = AllOperationalModes::BatteryChargeOnly;
        } else {
          OperationalMode = AllOperationalModes::FlightMode;
          FlightModePhaseIndex = 0;
          current_measurement.Record.Status = 0x0040;
        }

      } else if (OperationalMode == AllOperationalModes::AltitudeForHighCurrentOutput) {
        // a long click in Sea level pressure Set Mode will Write pressure to EEPROM, reset field altitude, & exit that mode. Depending on USB power we go to flight mode or charge mode
        EEPROM.put(MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft, AltitudeHighCurrentOutSetting_ft);
        display.set2X();
        display.clear();
        display.print(F("SAVED"));  //  takes 1ms plus 0.62ms per character
        delay(600);
        display.clear();  //  don't want OLED display noise on the power supply when we get field altitude
        delay(50);
        FindFieldAltitude_m();
        ReadDISPButton(-1);  //  reset button decode and start fresh
        if (USBPowered()) {
          OperationalMode = AllOperationalModes::BatteryChargeOnly;
        } else {
          OperationalMode = AllOperationalModes::FlightMode;
          FlightModePhaseIndex = 0;
        }

      } else {
        //error
        OperationalMode = AllOperationalModes::FlightMode;
        FlightModePhaseIndex = 0;
      }
      break;



    case DOUBLE_CLICK_FOUND:
      if (OperationalMode == AllOperationalModes::FlightMode) {
        DoBuzzer(0);
        display.clear();  // keep 3.0V power as clean as possible, reduce OLED noise.
        IncrementMCUEEPROMTime();
        delay(50);
        FindFieldAltitude_m();
        digitalWrite(HighCurrentOut, LOW);
        FlightModePhaseIndex = 0;
        current_measurement.Record.Status = 0x0040;
      } else if (OperationalMode == AllOperationalModes::BatteryChargeOnly) {

      } else if (OperationalMode == AllOperationalModes::SerialHost) {

      } else if (OperationalMode == AllOperationalModes::SealevelPressureSetMode) {  //  change the direction of the adjustment of sea level pressure
        SeaLevelPressureSetUpDirection = !SeaLevelPressureSetUpDirection;
        display.clear();  // This takes 33ms to run @ 400 kHz I2C speed!
        if (SeaLevelPressureSetUpDirection) {
          // display UP and current sea level pressure
          //display.clearField(0, 0, 1);
          display.print(F("UP  "));  //  takes 1ms plus 0.62ms per character
        } else {
          // display DN and current sea level pressure
          //display.clearField(0, 0, 1);
          display.print(F("DN  "));  //  takes 1ms plus 0.62ms per character
        }
        SeaLevelDisplayFinish();

      } else if (OperationalMode == AllOperationalModes::AltitudeForHighCurrentOutput) {  //  Altitude setting for high current output
        SeaLevelPressureSetUpDirection = !SeaLevelPressureSetUpDirection;
        display.clear();  // This takes 33ms to run @ 400 kHz I2C speed!
        if (SeaLevelPressureSetUpDirection) {
          // display UP and current sea level pressure
          //display.clearField(0, 0, 1);
          display.print(F("UP  "));  //  takes 1ms plus 0.62ms per character
          display.print(AltitudeHighCurrentOutSetting_ft);
          display.print(F("  "));  //  takes 1ms plus 0.62ms per character
        } else {
          // display DN and current sea level pressure
          //display.clearField(0, 0, 1);
          display.print(F("DN  "));  //  takes 1ms plus 0.62ms per character
          display.print(AltitudeHighCurrentOutSetting_ft);
          display.print(F("  "));  //  takes 1ms plus 0.62ms per character
        }
      } else {
        //error
        OperationalMode = AllOperationalModes::FlightMode;
        FlightModePhaseIndex = 0;
        current_measurement.Record.Status = 0x0040;
      }
      break;


    default:
      //  so no button clicking

      if (OperationalMode == AllOperationalModes::FlightMode) {
        if (USBPowered()) {  //  We leave flight mode if plugged into USB power
          DoBuzzer(0);
          MiaServo.detach();
          //digitalWrite(TestPoint7, LOW); //  DEBUG
          // USB power plugged in, Leave flight mode for charging mode
          OperationalMode = AllOperationalModes::BatteryChargeOnly;
          display.set2X();
          display.clear();
          //if ((digitalRead(N_BatteryCharging) == HIGH) && (HasChargeInput)) {      //  Not used in this version
          //  display.print(F("DONE "));
          //}
          display.print(F("CHARGING"));  //  takes 1ms plus 0.62ms per character
          //delay(60);
          OLEDBlankTime = millis() + 5000;  //  we blank the display after 5 seconds, this allows for proper charging of the battery without the extra current running the OLED.
          //LastDisplayedAltitude_m = InvalidAltitude;  //  invalidate last displayed max altitude
        } else {
          //  ***************************************************************************************************
          //  Flight mode           Flight mode              Flight mode           Flight mode
          //  ***************************************************************************************************
          //  flight mode will prepare an initial record (with field altitude) but not log it until we have actually launched (per requirements of START_LOGGING_ALTITUDE_m)
          //  The different phases of a flight are identified from the pressure sensor and time and kept track of in FlightModePhaseIndex.
          //    FlightModePhaseIndex:
          //      0  A brief phase we go through to set things up so we can wait for launch. Logging EEPROM starting address is fetched, collect initial flight record data. Init display. Init servo.
          //      1  The pre-launch phase where we wait for our altitude to start increasing. During this wait we are collecting a queue of measurements so we actually log data from just before launch.
          //      2  After detecting launch we advance to 2, flight phase. Continious sensor readings are saved to the logging EEPROM.
          //      3  If the logging EEPROM gets full, we go to phase 3, display maximum altitude only, no logging.
          //      4  We advance to 4, descent, after detecting apogee.
          //      5  We get here after detecting landing. We wait for MCU_EEPROM_DELAY_FROM_LANDING_TO_LOW_POWER_MODE seconds then transition to low power mode.
          //      6  Low power mode
          //
          /*                                                                                                                                                      
                                                                                                                                                        
                                                                               .─────.                 FlightModePhaseIndex=4                           
                                                                             ,'       `.◀───────────       Apogee detect. See APOGEE_DESCENT_THRESHOLD
                                                                           ,'           `.           Optional, servo to apogee.                         
                                                                          ╱               ╲                                                             
                                                                         ╱                 ╲                                                            
                                                                        ;                   :◀───────────────  FlightModePhaseIndex=4                   
                                                                        ;                   :      ◿=◺        Apogee plus 'ServoApogeeDuration_ms' milliseconds. (400ms)           
                                                                       ;                     :     \|/       Optional, servo to descent.                 
                                                                       ;                    ◣:      |                                                   
                                                                      ;                     ▆▆▆▆▆───┴▷                                                  
                                                                      ;                     ◤ :                                                         
                                                                     ;                         :                                                        
                                                                     ;                         :                                                        
                                                                    ;                           :                                                       
                                                                    │                           │                                                       
                                                                    ;                           :                                                       
                                                                   ;                             :                    FlightModePhaseIndex=6            
                                                                   │                             │◀─────────── Altitude (adjustable) threshold for      
                                                                   │                             │                     High current output.             
                                                                   │                             │                Optional, servo to HC output.         
                                                                   ;                             :                                                      
                                                                  ;                               :                                                     
                                                                  │                               │                                                     
                                   FlightModePhaseIndex=2         │                               │                                 NOTE:               
    FlightModePhaseIndex=1                 Ascent                 │                               │                        FlightModePhaseIndex=3       
      Waiting for launch          Optional, servo to ascent       │                               │                       EEPROM full, no logging.      
                                                  │               │                               │                                                     
                       │                  __      │               △                               │                                                     
                       └──────────────▶ ╱│  │     └─────────────▶ █                               │                                                     
                                         │  │         △           █                               │                                                     
         FlightModePhaseIndex=0  ─────▶  │__│─ ─ ─ ─ ─█─ ─ ─ ─ ▶  █        FlightModePhaseIndex=5 │        FlightModePhaseIndex=6                       
        Initialization for flight       ╱  ╱         ◢█◣         ◢█◣              Landed.         │       After programmable delay:                     
     This phase only lasts 0.02 sec.         RSO                  │     Optional, servo to landed.│           Low Power Phase.                          
      Optional, servo to pre-launch          TABLE                │                        |      │     Optional, servo to low power.                   
                                                             Launch Pad                    ╰─────▶|◀─────┘                                              

*/
          if (FlightModePhaseIndex == 0) {
            //  ^^^^^^^^^^^^^^^^^^^^^^^   Initialize for flight.  We get here from 3 places: power up, USB power removal. First collect initial data.
            SetUpMiaFromMCUEEPROM();  //  SamplePeriod_ms is setup on returning from SetUpMiaFromMCUEEPROM().
            //ApogeeDetected = false;
            display.clear();  // Keep 3.0V power as clean as possible during the findinng of field altitude, reduce OLED noise on 3.0 volt power.
            delay(50);
            FindFieldAltitude_m();  // Get a fresh field altitude.

            EEPROM.get(MCU_EEPROM_EXT_EEPROM_ADDR_START, EepromAddress);
            EEPROM.get(MCU_EEPROM_ADDR_DEFAULT_TIME_s, EEPROMTime);  // Date stamp for the CSV formatted data dump
            PopulateFlightRecord(0);                                 // Collect all initial record data, don't write it yet since we don't know if we will really fly.
            IncrementMCUEEPROMTime();
            LastDisplayedAltitude_m = InvalidAltitude;  //  Invalidate last displayed max altitude.

            BuzzerSchedule[2] = INTER_BUZZ_DELAY_ms;  // Restore buzzer from low power mode.
            BuzzerSchedule[6] = INTER_BUZZ_DELAY_ms;


            if (ServoNotSounder) {  //   if servo...
              MiaServo.write(ServoFlightStateArray[ServoWaitingForLaunch_index]);
            }

            FlightStatus = 0;  // initialize flight status word of logging flight record
            AltitudeHighCurrentOutSetting_m = AltitudeHighCurrentOutSetting_ft / METERS_TO_FEET;
            FlightModePhaseIndex = 1;  // advance to wait for launch phase
            TimeStamp1Ago = millis();
            //Serial.println("FlightModePhaseIndex = 1");


          } else if (FlightModePhaseIndex == 1) {
            //  ^^^^^^^^^^^^^^^^^^^^^^^   Wait for launch phase
            //  Flight phase 1, wait for altitude to start going up.

            // Keep a queue of the last three altitudes and timestamps. These have to get updated at the sample rate.


            if (millis() >= (TimeStamp1Ago + SamplePeriod_ms)) {

              //CurrentAltitude3Ago_m = CurrentAltitude2Ago_m;  //  all above sea level measurements
              CurrentAltitude2Ago_m = CurrentAltitude1Ago_m;
              CurrentAltitude1Ago_m = newAltitude_m;  //

              //TimeStamp3Ago = TimeStamp2Ago;
              TimeStamp2Ago = TimeStamp1Ago;
              TimeStamp1Ago += SamplePeriod_ms;
            }
            getAltitude();

            //  Launch Detection: Dual check as noise filter. See if new altitude is enough higher than second previous altitude to detect launch
            if ((((newAltitude_m - CurrentAltitude1Ago_m) >= START_LOGGING_ALTITUDE_m) && ((CurrentAltitude1Ago_m - CurrentAltitude2Ago_m) >= (START_LOGGING_ALTITUDE_m / 2))) || (newAltitude_m > fieldAltitude_m + 2.0)) {
              //  *** OK, passed launch detect, things get busy here. ***************************************************************************
              // This is our "at launch" list:  1) Write 1st initial record. 2) Write 2nd initial record.  3) Write the 2 queued up records. 4) First live record. (See bottom of this file for format).
              {
                // 1) Write 1st initial record that has already been populated....
                WriteRecordAtSamplePeriod(1);
                //  2) ...and 2nd initial record with lat & lon....
                for (i = 0; i < 32; i++) {
                  CommandCharacter = current_measurement.Bytes[i];
                  EEPROM.get(i + MCU_EEPROM_ADDR_LATITUDE_LONGITUDE, CommandCharacter);
                  current_measurement.Bytes[i] = CommandCharacter;
                  if ((CommandCharacter == 0) && (i > 10)) {  // If this is the end of string delimiter, stop copying, we got all we need.
                    break;
                  }
                }
                current_measurement.Record.Status = 0x0100;
                WriteRecordAtSamplePeriod(1);

                // 3) ...the 2 queued up records....
                newAltitude_m = CurrentAltitude2Ago_m;
                PopulatePreLaunchQueueFlightRecord(2, TimeStamp2Ago);
                WriteRecordAtSamplePeriod(1);
                newAltitude_m = CurrentAltitude1Ago_m;

                FlightStatus = ServoAscent_index << 9;

                PopulatePreLaunchQueueFlightRecord(3, TimeStamp1Ago);
                WriteRecordAtSamplePeriod(1);

                // 4) our first live record
                getAltitude();  //  Get altitude but don't display it, as we are flying, there is nobody to see it.
                RecordNumber = 4;
                PopulateFlightRecord(RecordNumber);


                WriteRecordAtSamplePeriod(0);
                //RecordNumber = 2;
                display.clear();  //  no need to draw power with the OLED since we are flying, and we want to keep the 3.0V bus clean

                // servo update
                if (ServoNotSounder) {
                  MiaServo.write(ServoFlightStateArray[ServoAscent_index]);
                }
                FlightModePhaseIndex = 2;  // Advance to flight phase
                //Serial.println("FlightModePhaseIndex = 2");
              }
            } else {
              displayAltitude();  //  Want to see the altitude while preping the rocket.
            }
          }


          else if (FlightModePhaseIndex == 2) {
            //  ^^^^^^^^^^^^^^^^^^^^^^^   Flight phase, looking for apogee
            //   flight phase with logging to external EEPROM
            //CurrentAltitude4Ago_m = CurrentAltitude3Ago_m;
            CurrentAltitude3Ago_m = CurrentAltitude2Ago_m;
            CurrentAltitude2Ago_m = CurrentAltitude1Ago_m;  // Record next to last altitude for peak altitude detection.
            CurrentAltitude1Ago_m = newAltitude_m;          // Record last altitude for peak altitude detection.
            getAltitude();                                  // Get our current altitude and update maximum altitude.

            // servo update
            //if ((ApogeeDetected == false) && (CurrentAltitude4Ago_m > newAltitude_m)) {
              // This is the peak of APOGEE!!
              // We have two apogee detections, a more sensitive one for servo position and a less sensitive one for flight phase management.
              //Serial.print(F("Phase 2, apogee det 1, apogee detect true, MET_ms="));
              //Serial.println(OurFlightTimeStamps.ApogeeTime_ms);
              //ApogeeDetected = true;
            //}

            if (newAltitude_m < (maxAltitude_m - APOGEE_DESCENT_THRESHOLD)) {  //   If we are past apogee (we have descended 2 meters (6 feet) from our peak altitude) advance to descent phase.
              FlightModePhaseIndex = 4;
              Serial.print(F("Phase 2, apogee det 2, apogee detect true, MET_ms="));
              Serial.println(OurFlightTimeStamps.ApogeeTime_ms);
              if ((ServoNotSounder)) {
                MiaServo.write(ServoFlightStateArray[ServoApogee_index]);
              }
              FlightStatus = FlightStatus & 0xf1ff;
              FlightStatus = FlightStatus | (ServoApogee_index << 9) | 0x4000;
              OurFlightTimeStamps.ApogeeTime_ms = millis();  //  We record our apogee time for the next servo position change at a fixed time later. See ServoApogeeDuration_ms
              //ApogeeDetected = true;
            }

            PopulateFlightRecord(RecordNumber);
            WriteRecordAtSamplePeriod(0);

          } else if (FlightModePhaseIndex == 3) {
            //  ^^^^^^^^^^^^^^^^^^^^^^^   Monitor mode. Ran out of EEPROM, just report.
            // Flight phase but we ran out of EEPROM, so just report altitude to OLED.
            display.clearField(100, 0, 3);
            display.print(F("FUL"));                        //   EEPROM FULL indication in top right corner of display.
            CurrentAltitude3Ago_m = CurrentAltitude2Ago_m;  //  all above sea level measurements
            CurrentAltitude2Ago_m = CurrentAltitude1Ago_m;
            CurrentAltitude1Ago_m = newAltitude_m;
            getAltitude();
            //displayAltitude();  // Don't use OLED to keep 3.0V as clean as possible. Only used for debug.
            AltitudeDelta = CurrentAltitude3Ago_m - newAltitude_m;
            LandingAltitude_m = newAltitude_m - fieldAltitude_m;
            if ((abs(AltitudeDelta) < 0.5) && (abs(LandingAltitude_m) < 10.0)) {
              DisplayLandedIndication();
              //  We don't update FlightStatus since nothing is getting recorded.
              FlightModePhaseIndex = 5;
              //Serial.println("FlightModePhaseIndex = 5");
              //Serial.print("Flight Phase=");
              //Serial.println(FlightModePhaseIndex);
            }
          }


          else if (FlightModePhaseIndex == 4) {
            //  ^^^^^^^^^^^^^^^^^^^^^^^   Post apogee looking for landing.
            CurrentAltitude3Ago_m = CurrentAltitude2Ago_m;  //  NOTE: all above sea level measurements.
            CurrentAltitude2Ago_m = CurrentAltitude1Ago_m;
            CurrentAltitude1Ago_m = newAltitude_m;
            getAltitude();  //   Get current altitude but don't display maximum altitude until we have landed because nobody can see it yet.


            // High current output altitude threshold check.
            if (AltitudeHighCurrentOutSetting_m > (newAltitude_m - fieldAltitude_m)) {
              digitalWrite(HighCurrentOut, HIGH);  // High current output ON.
              FlightStatus = FlightStatus & 0xf1ff;
              FlightStatus = FlightStatus | ServoHighCurrent_index << 9;
              if (ServoNotSounder) {
                MiaServo.write(ServoFlightStateArray[ServoHighCurrent_index]);
              }
            }
            //displayAltitude();    // Only for calibration or debug.
            // We are past apogee, waiting to detect landing.

            // Update MCU EEPROM for last maximum altitude
            float LastMaximumAltitude;
            EEPROM.get(MCU_EEPROM_LAST_MAXIMUM_ALTITUDE, LastMaximumAltitude);
            if (LastMaximumAltitude != (maxAltitude_m - fieldAltitude_m)) {
              EEPROM.put(MCU_EEPROM_LAST_MAXIMUM_ALTITUDE, (maxAltitude_m - fieldAltitude_m)); 
            }

            AltitudeDelta = CurrentAltitude3Ago_m - newAltitude_m;
            LandingAltitude_m = newAltitude_m - fieldAltitude_m;

            // Sample rate update and possible servo position update
            if ((OurFlightTimeStamps.ApogeeTime_ms + ServoApogeeDuration_ms) <= millis()) {
              Serial.print(F("Phase 4, finished apogee to descent,  MET_ms="));
              Serial.println(millis());
              EEPROM.get(MCU_EEPROM_SAMPLE_RATE_POST_APOGEE, SamplePeriod_ms);  //  Change to descent sensor data sample period.
              FlightStatus = FlightStatus & 0xf1ff;
              FlightStatus = FlightStatus | (ServoDescent_index << 9);
              if (ServoNotSounder) {
                MiaServo.write(ServoFlightStateArray[ServoDescent_index]);
              }
            }

            if ((abs(AltitudeDelta) < 1.0) && (abs(LandingAltitude_m) < 9.0)) {
              // We have landed on a planet!  Save our last record.

              FlightStatus = FlightStatus & 0xf1ff;
              FlightStatus = FlightStatus | 0x8040 | (ServoLanded_index << 9);
              PopulateFlightRecord(RecordNumber);
              WriteRecordAtSamplePeriod(1);

              EEPROM.put(MCU_EEPROM_EXT_EEPROM_ADDR_START, (uint32_t)EepromAddress);  //  Update external EEPROM next address in MCU EEPROM.

              digitalWrite(HighCurrentOut, LOW);  // High current output OFF.

              // Update display
              displayAltitude();
              DisplayLandedIndication();

              // servo update
              if (ServoNotSounder) {
                MiaServo.write(ServoFlightStateArray[ServoLanded_index]);
              }
              OurFlightTimeStamps.LandingTime_ms = millis();
              FlightModePhaseIndex = 5;
              //Serial.println("FlightModePhaseIndex = 5");

            } else {
              PopulateFlightRecord(RecordNumber);
              WriteRecordAtSamplePeriod(0);
            }
          }


          else if (FlightModePhaseIndex == 5) {
            //  ^^^^^^^^^^^^^^^^^^^^^^^   Landed, start buzzer or position servo and wait for transition to low power mode.

            //  Check and see if we go to low power flight mode yet
            DoBuzzer(1);

            if ((OurFlightTimeStamps.LandingTime_ms + DelayToLowPower_ms <= millis())) {

              // In low power mode. Shut off display, move servo to low power position, increase time between times sounder is on.
              display.clear();
              if (ServoNotSounder) {
                // servo update
                MiaServo.write(ServoFlightStateArray[ServoLowPower_index]);
              } else {

                EEPROM.get(MCU_EEPROM_LOW_POWER_BUZ_REST_MULT, Mult);
                LowPowerSounderMultiplier = BuzzerSchedule[2] * Mult;
                if (LowPowerSounderMultiplier > 0xfffe) {
                  LowPowerSounderMultiplier = 0xfffe;
                }
                BuzzerSchedule[2] = (uint16_t)LowPowerSounderMultiplier;

                LowPowerSounderMultiplier = BuzzerSchedule[6] * Mult;
                if (LowPowerSounderMultiplier > 0xfffe) {
                  LowPowerSounderMultiplier = 0xfffe;
                }
                BuzzerSchedule[6] = (uint16_t)LowPowerSounderMultiplier;
              }
              FlightModePhaseIndex = 6;
              //Serial.println("FlightModePhaseIndex = 6");
            } else {
              //  Landed but not in low power mode yet, just let buzzer/servo do what they are doing.
              //displayAltitude();   //  display maximum altitude after landing and before low power mode.
            }


          } else if (FlightModePhaseIndex == 6) {
            DoBuzzer(1);
            //  ^^^^^^^^^^^^^^^^^^^^^^^   Staying in  low power mode.
          }

          //  invalid FlightModePhaseIndex
          else {
            //  something went wrong, not a valid flight phase.
          }
        }  //   end of "not USB powered" in flight mode
      }    // end of flight mode

      else if (OperationalMode == AllOperationalModes::BatteryChargeOnly) {
        //Serial.println(F("T"));  //  DEBUG
        if (!USBPowered()) {

          //  USB power just went away
          display.clear();  //   get rid of the 'CHARGING...' display right away.
          OperationalMode = AllOperationalModes::FlightMode;
          FlightModePhaseIndex = 0;
          current_measurement.Record.Status = 0x0040;
          // delay so power can settle down after USB getting unplugged and OLED clearing, we need an accurate field altitude
        } else {

          //  USB power is still applied.
          if (millis() > OLEDBlankTime) {  //  See if it is time to blank the OLED display.
            display.clear();               // This takes 33ms to run @ 400 kHz I2C speed.
          }

          // now check to see if we are going to host mode
          CharactersToRead = Serial.available();
          if (CharactersToRead > 0) {
            NewCharacter = (Serial.read() & 0x7f);  //  We will fall down through this path in the Arduino loop looking to see if we go into host mode.
            if (NewCharacter == 0x0d) {             //  We are waiting for the host user to hit a return (Enter) character.
              MiaServo.detach();
              OperationalMode = AllOperationalModes::SerialHost;
              display.clear();
              //display.print(F("HOST"));
              PrintPrompt();  //  Command prompt.
            }
          }
        }
      } else if (OperationalMode == AllOperationalModes::SerialHost) {
        FlightModePhaseIndex = 0;
        if (!USBPowered()) {
          OperationalMode = AllOperationalModes::FlightMode;
        }

        CaptureCommandLine();
        ProcessCommand();
      } else if (OperationalMode == AllOperationalModes::SealevelPressureSetMode) {
        // Only need to change things after clicks.
      } else if (OperationalMode == AllOperationalModes::AltitudeForHighCurrentOutput) {
        // Only need to change things after clicks.
      } else {
        //error
        OperationalMode = AllOperationalModes::FlightMode;
        FlightModePhaseIndex = 0;
      }

  }  // End of button 1 click switch statement.

  ThisUSER2Click = ReadButtonUSER2();
  switch (ThisUSER2Click) {
    case LONG_CLICK_FOUND:
      noTone(BuzzerOut);
      if (OperationalMode == AllOperationalModes::FlightMode) {
        InitHighCurrentOutAltitude();
      } else if (OperationalMode == AllOperationalModes::BatteryChargeOnly) {
        InitHighCurrentOutAltitude();
      } else if (OperationalMode == AllOperationalModes::SerialHost) {
        //  Ignored in serial host mode
      } else if (OperationalMode == AllOperationalModes::AltitudeForHighCurrentOutput) {
        // a long click in Sea level pressure Set Mode will Write pressure to EEPROM, reset field altitude, & exit that mode. Depending on USB power we go to flight mode or charge mode
        EEPROM.put(MCU_EEPROM_ADDR_AltitudeHighCurrentOut_ft, AltitudeHighCurrentOutSetting_ft);
        display.set2X();
        display.clear();
        display.print(F("SAVED"));  //  Takes 1ms plus 0.62ms per character.
        delay(600);
        display.clear();  //  Clear all OLED pixels so we reduce electrical noise on the 3.0 volt power supply for the pressure sensor when we get field altitude.
        delay(50);
        FindFieldAltitude_m();
        ReadDISPButton(-1);  //  Reset "button decode" and start fresh.
        if (USBPowered()) {
          OperationalMode = AllOperationalModes::BatteryChargeOnly;
        } else {
          OperationalMode = AllOperationalModes::FlightMode;
          FlightModePhaseIndex = 0;
        }
      } else {
        //error
        OperationalMode = AllOperationalModes::FlightMode;
        FlightModePhaseIndex = 0;
      }
      break;
  }
}


/*
  Logging EEPROM format. All records are 32 bytes.

            / ┌─────────────────────────────────────────────────┐┌─────────────────────────────────────────────────┐
           │  │                  Record Index.                  ││                   Status bits                   │
           │  │      Record index 0 is launch conditions.       ││  A collection of 15 bits indicating states of   │
           │  │                                                 ││ internal and external conditions, and phases of │
           │  │                                                 ││                   the flight.                   │
           │  │<------------------ uint16_t ------------------> ││<------------------ uint16_t ------------------> │
           │  └─────────────────────────────────────────────────┘└─────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                                Mission elapse time in milliseconds                                 │
           │  │ <------------------------------------------- uint32_t -------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                                   Field altitude above sealevel.                                   │
           │  │ <-------------------------------------------- float ---------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                                         Temperature in °F                                          │
  Record 0 │  │ <-------------------------------------------- float ---------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │               Sealevel pressure in millibars (hPa) times 4. Gives 1/4mb resolution.                │
           │  │ <------------------------------------------- uint32_t -------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                 Unix/Linux Date-Time. Seconds since Jan 01 1970 (UTC). First half.                 │
           │  │ <---------------------------------------------------------------------------------- int64_t ------ │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                Unix/Linux Date-Time. Seconds since Jan 01 1970 (UTC). Second half.                 │
           │  │ -------------------------------------------------------------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                                               Spare.                                               │
           │  │ <--------------------------------------------- N/A ----------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
            \


            /
           │  ┌─────────────────────────────────────────────────┐┌─────────────────────────────────────────────────┐
           │  │                  Record Index.                  ││                   Status bits                   │
           │  │  Record index 1 is the launch location string.  ││  A collection of 15 bits indicating states of   │
           │  │                                                 ││ internal and external conditions, and phases of │
           │  │                                                 ││                   the flight.                   │
           │  │<------------------ uint16_t ------------------> ││<------------------ uint16_t ------------------> │
           │  └─────────────────────────────────────────────────┘└─────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                       Latitude and longitude string. Characters 1 through 4.                       │
           │  │<--------------------------------------------- char ----------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                       Latitude and longitude string. Characters 5 through 8.                       │
           │  │<--------------------------------------------- char ----------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
  Record 1 │  │                      Latitude and longitude string. Characters 9 through 12.                       │
           │  │<--------------------------------------------- char ----------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                      Latitude and longitude string. Characters 13 through 16.                      │
           │  │<--------------------------------------------- char ----------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                      Latitude and longitude string. Characters 17 through 20.                      │
           │  │<--------------------------------------------- char ----------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                      Latitude and longitude string. Characters 21 through 24.                      │
           │  │<--------------------------------------------- char ----------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │ Latitude and longitude string. Characters 25 through 27. The last byte of the string must be zero. │
           │  │<--------------------------------------------- char ----------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
            \



            / ┌─────────────────────────────────────────────────┐┌─────────────────────────────────────────────────┐
           │  │                  Record Index.                  ││                   Status bits                   │
           │  │ Record Indices 2 through the end are flight log ││  A collection of 15 bits indicating states of   │
           │  │           data at the sample period.            ││ internal and external conditions, and phases of │
           │  │                                                 ││                   the flight.                   │
           │  │<------------------ uint16_t ------------------> ││<------------------ uint16_t ------------------> │
           │  └─────────────────────────────────────────────────┘└─────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                                Mission elapse time in milliseconds                                 │
           │  │ <------------------------------------------- uint32_t -------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                                       Altitude above ground                                        │
           │  │ <-------------------------------------------- float ---------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
  Records  │  │                                         Temperature in °F                                          │
  2 to end │  │ <-------------------------------------------- float ---------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │                              Voltage from light sensor in millivolts.                              │
           │  │ <------------------------------------------- uint32_t -------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │        Acceleration in the X direction. Negitive with the display pointing down. Units: g.         │
           │  │ <-------------------------------------------- float ---------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │Acceleration in the Y direction. Positive with the ON-OFF switch up and the buttons down. Units: g. │
           │  │ <-------------------------------------------- float ---------------------------------------------> │
           │  └────────────────────────────────────────────────────────────────────────────────────────────────────┘
           │  ┌────────────────────────────────────────────────────────────────────────────────────────────────────┐
           │  │ Acceleration in the Z direction. Positive with the USB connector down and the pressure sensor up.  │
           │  │                                             Units: g.                                              │
           │  │ <-------------------------------------------- float ---------------------------------------------> │
            \ └────────────────────────────────────────────────────────────────────────────────────────────────────┘






  Status bits

             ┌─────────────────────────────────────────────────────────────── 15   Landing detected
             │   ┌─────────────────────────────────────────────────────────── 14   Apogee detected
             │   │   ┌─────────────────────────────────────────────────────── 13   Unused
             │   │   │   ┌─────────────────────────────────────────────────── 12   Abnormal termination
             │   │   │   │       ┌─────────────────────────────────────────── 11:9 Flight phase (servo position index)
             │   │   │   │       │       ┌─────────────────────────────────── 8    Location Record
             │   │   │   │       │       │   ┌─────────────────────────────── 7    Unused
             │   │   │   │       │       │   │   ┌─────────────────────────── 6    Last record of current flight
             │   │   │   │       │       │   │   │   ┌─────────────────────── 5    Log Temperature units = °C
             │   │   │   │       │       │   │   │   │   ┌─────────────────── 4    Log Altitude units = feet
             │   │   │   │       │       │   │   │   │   │   ┌─────────────── 3    Test point 7 (Digital 3)
             │   │   │   │       │       │   │   │   │   │   │   ┌─────────── 2    Buzzer on
             │   │   │   │       │       │   │   │   │   │   │   │   ┌─────── 1    State of high current output (Digital 7)
             │   │   │   │       │       │   │   │   │   │   │   │   │   ┌─── 0    Initial record (must be index 0)
             │   │   │   │   ┌───┼───┐   │   │   │   │   │   │   │   │   │
           ┌─┴─┬─┴─┬─┴─┬─┴─┳─┴─┬─┴─┬─┴─┬─┴─┳─┴─┬─┴─┬─┴─┬─┴─┳─┴─┬─┴─┬─┴─┬─┴─┐
           │15 │14 │13 │12 ┃11 │10 │ 9 │ 8 ┃ 7 │ 6 │ 5 │ 4 ┃ 3 │ 2 │ 1 │ 0 │
           └───┴───┴───┴───┻───┴───┴───┴───┻───┴───┴───┴───┻───┴───┴───┴───┘
*/