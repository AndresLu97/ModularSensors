[//]: # ( @page alpha_EC_logging EC Logging)
# Electrical Conductivity - saving data to an SD card  alpha version

This is an "development" version located in ModularSensors\examples\tu_ec01 as the PlatformIo setup, 
and allows editing of code in the ModularSensors\a\tu_ec01.
and references the github stable version of ModularSensors - copied to 
examples\tu_ec01\.pio\libdeps\mayfly\EnviroDIY_ModularSensors

This  configuration is defined in ModularSensors\examples\tu_ec01\platformio.ini.
To use,
a ) Build and download from PlaformIO
b) FUT   
in shell step to exampes\tu_ec01
.\doMe.ps1  <release_num>   eg .\doMe.ps1 0.30.0dvlp  .hex are put in ../../../releases

The time needs to be configured.
https://raw.githubusercontent.com/EnviroDIY/Sodaq_DS3231/master/examples/PCsync/PCsync.ino
https://www.envirodiy.org/topic/clock-sync-for-mayfly-v1-0/
_______

[//]: # ( @tableofcontents )

[//]: # ( Start GitHub Only )
- [Using ModularSensors to save data to an SD card](#using-modularsensors-to-save-data-to-an-sd-card)
- [Unique Features of the Simple Logging Example](#unique-features-of-the-simple-logging-example)
- [To Use this Example:](#to-use-this-example)
  - [Prepare and set up PlatformIO](#prepare-and-set-up-platformio)
  - [Set the logger ID](#set-the-logger-id)
  - [Upload!](#upload)

[//]: # ( End GitHub Only )

_______

[//]: # ( @section example_simple_logging_unique Unique Features of the Simple Logging Example )
# Unique Features of the Simple Logging Example
- Only logs data to an SD card.

[//]: # ( @section example_simple_logging_using To Use this Example: )
# To Build this program:

[//]: # ( @subsection example_simple_logging_pio Prepare and set up PlatformIO )
## Prepare and set up PlatformIO
- Ensure PlatformIO project is setup
- From PlatformIO, go to File Folder and open top level of project ModularSensors\a\tu_ec01


[//]: # ( @subsection example_simple_logging_logger_id Set the logger ID )
## Setup per logger configuration
- Open D:\usr\a\Documents\Arduino\env03\ModularSensors\a\tu_ec01\src\ini_opts\ms_cfg.ini, 
- change the values to represent your logger.

```cpp
[COMMON]
;This section required. Max 80 active chars, not including after any ;
LOGGER_ID =FOP_EC01 ;Auto Site Reference
LOGGING_INTERVAL_MINUTES=15
LIION_TYPE = 3 ;0=Any 1=LiIon0.5A 2=1AH 3=LiSOCL2 4=3Leclanche
TIME_ZONE=-8  ; -12 to +12
GEOGRAPHICAL_ID="SSU FOP EC DemoCreek#02 Monitor" 
```

## Configure Mayfly EEPROM
Two ms_cfg.iniX files can program the EEPROM.
if an ms_cfg.ini0 or ms_cfg.ini1 is detected they are read, and values operated on,
then the file is renamed with "run" appended so it is only acted on once.
For ms_cfg.ini0 it is read, acted on, then renamed to ms_cfg.ini0run
For ms_cfg.ini1 it is read, acted on, then renamed to ms_cfg.ini1run
 
- to permanently configure in the EEPROM, including board information
- modify ms_cfg.ini0 with [BOOT] section enabled
- modify/create ms_cfg.ini1 with no [BOOT] and the following enabled.
  [USER]
  ACTION=WRITE ; Last Operation

- Typically for ms_cfg.ini1, 
  copy the core ms_cfg.ini to ms_cfg.ini1
  then enable 
  [USER]
  ACTION=WRITE ; Last Operation


## Upload!
- Test everything at the office **before** deploying out in the wild!

_______


[//]: # ( @section example_simple_logging_pio_config PlatformIO Configuration )

[//]: # ( @include{lineno} simple_logging/platformio.ini )

[//]: # ( @section example_simple_logging_code The Complete Code )
