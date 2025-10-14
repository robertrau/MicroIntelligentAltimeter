#define STRINGIFY(s) XSTRINGIFY(s)
#define XSTRINGIFY(s) #s  // So you can do #pragma message ("Compile_Time_Var= " STRINGIFY(Compile_Time_Var)) and see it in the compilier messages

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
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

          The first requirement is that you install minicore into the Arduino IDE so you can use the PB version of the ATMEGA328 that is on the Mia
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
            The programmer board powers your Mia. If programming fails, attach a battery to your Mia, turn the ON-OFF switch on, and re-try.
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
                The EEPROM location supports 64 bits to avoid the year 2038 problem, but the software is not 64 bit ready.
                The Mia host application also updates this time to the current time.

            ADDR = 8: The last Known air pressure at sea level. This is a float.

            ADDR = 12: The next available address in the external EEPROM. uint32_t

            ADDR = 16: This field is a personalization field. The user can put any thing they want here but the intent is for name and contact information.
                This field is 63 characters long with the last byte as a end of string byte.   NOT INPLEMENTED

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
                (Should we put the BMP390/BMP581 sample rate, IIR filter coeff & oversampling stuff here? No, not yet.)

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

            ADDR - 220: Milliseconds between samples during ascent and apogee      NOT IMPLEMENTED
                Read and set with "w" command

            ADDR - 222: Milliseconds between samples after [apogee plus 'ServoApogeeDuration_ms'] through landing      NOT IMPLEMENTED
                Read and set with "w" command

            (Space left for more word parameters that the "w" command can set. w command writes eight 16 bit values from 216 through 231)

            ADDR - 1020: 4 byte debug location. Used to store in-flight data and read it back later.


            Sketch uses 32042 bytes (98%) of program storage space. Maximum is 32384 bytes.   This is a lot, see programmer tips above.
    @endverbatim

    @author Robert Rau

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

  Updated: 6/6/2025
  Rev.: 1.0.0
  By: Robert Rau
  Changes: Just started

  Updated: 9/28/2025
  Rev.: 1.0.1
  By: Robert Rau
  Changes: Added printing of MCU EEPROM debug location in float, uint32, int32, and uint16.

*/
// Version
const char VersionString[] = "1.0.1\0";        //  ToDo, put in flash  see: https://arduino.stackexchange.com/questions/54891/best-practice-to-declare-a-static-text-and-save-memory
#define BIRTH_TIME_OF_THIS_VERSION 1759066867  //  Seconds from Linux Epoch. Used as default time in MCU EEPROM.
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
bool ApogeeDetected = false;
uint8_t ServoFlightStateArray[7] = { 90, 90, 90, 90, 90, 90, 90 };  // Values must be from 0 to 180. 0:Waiting for launch 1:Ascent 2:Apogee 3:Descent 4:Landed 5:Low power 6:@ high current altitude
uint8_t ServoState;
#define ServoWaitingForLaunch_index 0
#define ServoAscent_index 1
#define ServoApogee_index 2
#define ServoDescent_index 3
#define ServoLanded_index 4
#define ServoLowPower_index 5
#define ServoApogeeDuration_ms 400  //  The servo will go to position index 2 at apogee and stay there for ServoApogeeDuration_ms milliseconds, then go to position index ServoDescent_index

// Analog channels
#define USBToTenthsVolts 60 / 1023  // This will convert an A/D reading of Vusb to a number ten times the volts. NO parenthesis, we want the multiply by the 60 first, if integer math.

// I2C Addresses
#define I2C_OLED_ADDRESS 0x3C  // 0x3C address for 128x32 OLED
//  External EEPROM is at 0x50 and 0x51
#define I2C_BMP581_ADDRESS 0x47                      // 0x47 address for BMP581
#define I2C_MC3416_ADDRESS 0x4C                      // 0x4C address for accelerometer option 1 (never tested)
#define I2C_KX134_ADDRESS 0x1E                       // 0x1E address for accelerometer option 2 (KX134ACR NOT KX134-1211, software not compatible)
#define I2C_ACCELEROMETER_ADDRESS I2C_KX134_ADDRESS  // Address for selected accelerometer


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
#define APOGEE_DESCENT_THRESHOLD 6.0                   //  We must be below maximum altitude minus this value to detect we have passed apogee.
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
float CurrentAltitude1Ago_m;  // used for pre-launch queue and landing detection
float CurrentAltitude2Ago_m;  // used for pre-launch queue and landing detection
float CurrentAltitude3Ago_m;  // used for pre-launch queue and landing detection
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

#define MCU_EEPROM_DEBUG_LOCATION 1020

//  External EEPROM stuff
#define ExternalEEPROMSizeInBytes 262144U    //    262144 Bytes or 8192 flight records
#define MCU_EEPROM_EXT_EEPROM_ADDR_START 12  //  location in the MCU EEPROM where the next free address is in the external EEPROM
uint32_t EepromAddress;                      // Address within the external EEPROM
uint16_t RecordNumber;
#define MCU_EEPROM_ADDR_LATITUDE_LONGITUDE 160  // Last known launch location. 24 byte text string <latitude>,<longitude>,0x00

//  This type, Measurement, defines the format of every time-stamped sensor measurement record (pre and post launch) for the external EEPROM
typedef union {
  struct {                     // *** This structure is for the in flight data records
    uint16_t RecordIndex;      // This is a index of each record per flight
    uint16_t Status;           // This is the bits: 0: initial record (with field altitude and sea level pressure). 1:PD7_On. 2:BuzzerOn. 3:PD3_TP7. 4:Altitude in feet. 5: Temperature in °C.
                               //   6: Last record of current flight   8: Second initial record (with lat & lon)   9-11: Servo position    12: Abnormal termination  14: Apogee detected  15: Landing detected.
    uint32_t current_time_ms;  // This is time from when the Mia is turned on, mission elapsed time. In milliseconds. This is not related to the Linux time stamp at MCU EEPROM addr 0.
    float Altitude;            //  Altitude above GROUND level.
    float Temperature;         //  Using analog connector P3.
    uint32_t LightVoltage;
    float AccelerationX_g;  //  Not on first version of Mia.
    float AccelerationY_g;  //  Not on first version of Mia.
    float AccelerationZ_g;  //  Not on first version of Mia.
  } Record;
  byte Bytes[sizeof(Record)];  // *** This structure is for writing the records to the external EEPROM
  struct {                     // *** This structure is for the initial record before each flight.
    uint16_t RecordIndex;      // This is a index of each record per flight, for this initial record format it will be 0
    uint16_t Status;           // This is the bits: 0:InitialRecord (with field altitude and sea level pressure). 1:PD7_On. 2:BuzzerOn. 3:PD3_TP7. 4:Altitude in feet. 5: Temperature in °C.
                               //   6: Last record of current flight   8: Second initial record (with lat & lon)   9-11: Servo position    12: Abnormal termination  14: Apogee detected  15: Landing detected.
    uint32_t current_time_ms;  // This is time from when the Mia is turned on, mission elapsed time. In milliseconds. This is not related to the Linux time stamp at MCU EEPROM addr 0.
    float Altitude;            //  GROUND level altitude.
    float Temperature;         //  Using analog connector P3.
    uint32_t SeaLevelPressureX4;
    int64_t LinuxDateTime;
    uint32_t spare;
  } InitRecord;
} Measurement;
uint32_t ShortEEPROMTime;

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

  Serial.println("");
  Serial.println("");

  Serial.println(F("MCU EEPROM dump:"));
  uint8_t EEPROMByte;
  for (i = 0; i < 1024; i++) {
    if (i % 16 == 0) {
      Serial.println("");
      SerialPrintDecimalAndHex(i, 2);
      Serial.print(F("  "));
    }
    EEPROM.get(i, EEPROMByte);
    SerialPrintHexShort(EEPROMByte, 1);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print(F("Debug location in MCU EEPROM (1020) as float:"));

  float debugFloat;
  EEPROM.get(MCU_EEPROM_DEBUG_LOCATION, debugFloat);
  Serial.print(debugFloat);

  uint32_t debugUint32;
  Serial.print("  as uint32_t:");
  EEPROM.get(MCU_EEPROM_DEBUG_LOCATION, debugUint32);
  Serial.print(debugUint32);

  uint32_t debugint32;
  Serial.print("  as int32_t:");
  EEPROM.get(MCU_EEPROM_DEBUG_LOCATION, debugint32);
  Serial.print(debugint32);

  uint32_t debugUint16;
  Serial.print("  as uint16_t:");
  EEPROM.get(MCU_EEPROM_DEBUG_LOCATION, debugUint16);
  Serial.print(debugUint16);


  Serial.println("");
  Serial.println("");

  Wire.begin();
  Wire.setClock(400000L);

  M24M02E_Setup();

  //  Setup I2C

  InitExtEEPROMAddress();
}








/**
   @brief Read and report about External EEPROM starting address in MCU EEPROM, then set working external EEPROM address to zero.

   @details

   @param[in]  none

   @retval    none
*/
void InitExtEEPROMAddress() {
  uint32_t ExternalEEPROMAddressSaved;
  EEPROM.get(MCU_EEPROM_EXT_EEPROM_ADDR_START, ExternalEEPROMAddressSaved);
  if (ExternalEEPROMAddressSaved >= ExternalEEPROMSizeInBytes) {  //  see if it is invalid
    Serial.print(F("External EEPROM address in MCU EEPROM was invalid. It is "));
    SerialPrintDecimalAndHex(ExternalEEPROMAddressSaved, 4);
    EepromAddress = 0;
  } else {
    Serial.print(F("External EEPROM address in MCU EEPROM was in range. It is "));
    SerialPrintDecimalAndHex(ExternalEEPROMAddressSaved, 4);
  }
  Serial.println("");
  EepromAddress = 0;
  Serial.println(F("External EEPROM address set to zero. MCU EEPROM not changed."));
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
   @brief  Prints on the serial port a 2 digit Hex integer, adding leading zero if necessary

   @param[in] uint8_t Value to print

   @retval  none
*/
void SerialPrint2DigitHex(uint8_t IntForOutput) {
  if (IntForOutput < 16) {
    Serial.print("0");
  }
  Serial.print(IntForOutput, HEX);
}



/**
   @brief  Prints on the serial port a 1 to 4 byte number in both decimal and hex.

   @param[in] uint8_t Value to print

   @retval  none
*/
void SerialPrintDecimalAndHex(uint32_t Value, int NumberOfBytes) {
  if (Value < 1000000000) {
    Serial.print(" ");
    if (Value < 100000000) {
      Serial.print(" ");
      if (Value < 10000000) {
        Serial.print(" ");
        if (Value < 1000000) {
          Serial.print(" ");
          if (Value < 100000) {
            Serial.print(" ");
            if (Value < 10000) {
              Serial.print(" ");
              if (Value < 1000) {
                Serial.print(" ");
                if (Value < 100) {
                  Serial.print(" ");
                  if (Value < 10) {
                    Serial.print(" ");
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  Serial.print(Value);
  Serial.print(F(", 0x"));
  int i;
  uint8_t OneByte;
  for (i = 0; i < NumberOfBytes; i++) {
    OneByte = (Value >> ((NumberOfBytes - 1) * 8 - (i * 8))) & 0xff;
    SerialPrint2DigitHex(OneByte);
  }
}




/**
   @brief  Prints on the serial port a 1 to 4 byte number in both decimal and hex.

   @param[in] uint8_t Value to print

   @retval  none
*/
void SerialPrintHex(uint32_t Value, int NumberOfBytes) {
  Serial.print(F("0x"));
  int i;
  uint8_t OneByte;
  for (i = 0; i < NumberOfBytes; i++) {
    OneByte = (Value >> ((NumberOfBytes - 1) * 8 - (i * 8))) & 0xff;
    SerialPrint2DigitHex(OneByte);
  }
}


/**
   @brief  Prints on the serial port a 1 to 4 byte number in both decimal and hex.

   @param[in] uint8_t Value to print

   @retval  none
*/
void SerialPrintHexShort(uint32_t Value, int NumberOfBytes) {

  int i;
  uint8_t OneByte;
  for (i = 0; i < NumberOfBytes; i++) {
    OneByte = (Value >> ((NumberOfBytes - 1) * 8 - (i * 8))) & 0xff;
    SerialPrint2DigitHex(OneByte);
  }
}


/**
   @brief  Prints float so they will line up in columns

   @param[in] uint8_t Value to print

   @retval  none
*/
void SerialPrintFloat(float Value) {
  if (abs(Value) < 99999.95) {
    Serial.print(" ");
    if (abs(Value) < 9999.95) {
      Serial.print(" ");
      if (abs(Value) < 999.95) {
        Serial.print(" ");
        if (abs(Value) < 99.95) {
          Serial.print(" ");
          if (abs(Value) < 9.95) {
            Serial.print(" ");
          }
        }
      }
    }
  }
  if (Value >= 0.0) {
    Serial.print(" ");
  }
  Serial.print(Value);
}


/**
   @brief   Prints out the serial port a single line of initial measurements & parameters formatted as a line in a csv file.

   @details

   @note

   @param[in]  measurement

   @retval  none
*/
void PrintRecord(Measurement& measurement) {  // record dump
  uint32_t ShortEEPROMTime;

  SerialPrintDecimalAndHex(measurement.Record.RecordIndex, 2);
  Serial.print(" , ");
  SerialPrintHex(measurement.Record.Status, 2);
  Serial.print(" , ");

  if ((measurement.Record.RecordIndex == 1) && (measurement.Record.Status == 0x100)) {
    int x;
    for (x = 4; (x < 28); x++) {
      SerialPrintHexShort(measurement.Bytes[x], 1);
      Serial.print(" ");
    }
    Serial.print(" , ");
    for (x = 4; (x < 28) & (measurement.Bytes[x] != 0); x++) {
      Serial.print((char)measurement.Bytes[x]);
    }
    Serial.print("         ");

  } else {

    SerialPrintDecimalAndHex(measurement.Record.current_time_ms, 4);
    Serial.print(" , ");
    SerialPrintFloat(measurement.Record.Altitude * METERS_TO_FEET);
    Serial.print(" , ");
    SerialPrintFloat(measurement.Record.Temperature);
    Serial.print(" , ");
    SerialPrintDecimalAndHex(measurement.InitRecord.SeaLevelPressureX4, 4);  //  SeaLevelPressureX4 or LightVoltage
    Serial.print(" , ");
    ShortEEPROMTime = measurement.InitRecord.LinuxDateTime & 0xffffffff;
    SerialPrintDecimalAndHex(ShortEEPROMTime, 4);
    Serial.print(" , ");
    Serial.print(measurement.Record.AccelerationX_g);
    //Serial.print(" (");
    //Serial.print((uint32_t)measurement.Record.AccelerationX_g);
    Serial.print(" , ");
    Serial.print(measurement.Record.AccelerationY_g);
   // Serial.print(" (");
    //Serial.print((uint32_t)measurement.Record.AccelerationY_g);
    Serial.print(" , ");
    Serial.print(measurement.Record.AccelerationZ_g);
  }
}


void loop() {


  readByteArray(EepromAddress, current_measurement.Bytes, 32);
  if (current_measurement.Record.RecordIndex == 0) {
    Serial.println("");
    Serial.println(F(" Addr   ,     Index,  ,        Status    ,   Mission Time,  ,     Altitude   ,   Temp     ,     Light/Sealevel,  ,    Linux Time-Date,  ,     X  ,      Y  ,      Z"));
  }
  SerialPrintHex(EepromAddress, 3);
  Serial.print(F(" , "));
  PrintRecord(current_measurement);
  Serial.println("");
  EepromAddress = EepromAddress + 32;
  //if (EepromAddress >= ExternalEEPROMSizeInBytes) {
  if (EepromAddress >= 262140) {
    while (1)
      ;
  }
}
