[![hacs_badge](https://img.shields.io/badge/HACS-Custom-orange.svg?style=for-the-badge)](https://github.com/custom-components/hacs)

# custom_components
NEW PIDThermostat

## PID controller thermostat

### Installation:
1. Go to <conf-dir> default /homeassistant/.homeassistant/ (it's where your configuration.yaml is)
2. Create <conf-dir>/custom_components/ directory if it does not already exist
3. Clone this repository content into <conf-dir>/custom_components/
4. Set up the pid_thermostat and have fun

### Usage:
pid controller will be called periodically.
If no pwm interval is definde. It will set the state of "heater" 0-"difference"
Else it will turn off and on the heater proportionally.

#### Autotune:
Removed! Will be replaced by additional platform

### Parameters:

#### Still same:

* name (Required): Name of thermostat
* heater (Required): entity_id for heater switch, must be a toggle device. Becomes air conditioning switch when ac_mode is set to True
* target_sensor (Required): entity_id for a temperature sensor, target_sensor.state must be temperature.
* min_temp (Optional): Set minimum set point available (default: 7)
* max_temp (Optional): Set maximum set point available (default: 35)
* target_temp (Optional): Set initial target temperature. Failure to set this variable will result in target temperature being set to null on startup. As of version 0.59, it will retain the target temperature set before restart if available.
* ac_mode (Optional): Set the switch specified in the heater option to be treated as a cooling device instead of a heating device.
* away_temp (Optional): Set the temperature used by “away_mode”. If this is not specified, away_mode feature will not get activated.
* precision (optional): The desired precision for this device. Can be used to match your actual thermostat’s precision. Supported values are 0.1, 0.5 and 1.0.

Default: 0.5 for Celsius and 1.0 for Fahrenheit.
#### Removed:

* cold_tolerance (Optional):
* hot_tolerance (Optional):

#### Edited:

* keep_alive (Required): Set a keep-alive interval. Interval pid controller will be updated.
* min_cycle_duration (Optional): Set a minimum time for the PWM to be on. This should ne the value given by your valve for it not to break.

#### New:

* kp (Optional): Set PID parameter, p controll value.
* ki (Optional): Set PID parameter, i controll value.
* kd (Optional): Set PID parameter, d controll value.
* pwm (Optional): Set period time for pwm signal in minutes. If it's not set pwm is disabled.
* autotune (Removed)

* difference (Removed): always 100 now!
* noiseband (Removed):

#### Services:

* set_kp: will set the kp value during runtime. (won't save it!)
* set_ki: will set the ki value during runtime. (won't save it!)
* set_ki: will set the ki value during runtime. (won't save it!)
* set_pwm: will set the pwm value during runtime. (won't save it!)

#### configuration.yaml
```
climate:
  - platform: pid_thermostat
    name: Study
    heater: switch.study_heater
    target_sensor: sensor.study_temperature
    min_temp: 15
    max_temp: 21
    ac_mode: False
    target_temp: 17
    keep_alive:
      seconds: 10
    min_cycle_duration:
      minutes: 5
    away_temp: 16
    kp : 5
    ki : 3
    kd : 2
    pwm:
      minutes: 16
```
### Help

The python PID module:
[https://github.com/hirschmann/pid-autotune](https://github.com/hirschmann/pid-autotune)

PID controller explained. Would recommoned to read some of it:
[https://controlguru.com/table-of-contents/](https://controlguru.com/table-of-contents/)

Homeassistant forum:
[https://controlguru.com/table-of-contents/](https://controlguru.com/table-of-contents/)
