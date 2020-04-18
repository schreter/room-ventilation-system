# Debugging Features

If the define \#DEBUG is set, then the controller supports additional debugging
MQTT messages to set and get various status information.

This (incomplete) guide lists some of the debugging commands.

Refer to [Status.md](Status.md) for regular MQTT status messages.


## Fan Debugging MQTT Topics

Topic                          | Value (Unit)      | Description
------------------------------ | ----------------- | --------------------------------------
`d15/debugset/kwl/fan#/pwm`    | #### (PWM value)  | Set PWM value driving the fan (1 or 2) for the current mode
`d15/debugset/kwl/fan#/getvalues` | `on` or `off`  | Turn MQTT debug reporting for fan (1 or 2) on or off (via `d15/debugstate/kwl/fan#`), any other value causes debug info to be sent once, if debugging is off
`d15/debugset/kwl/fan#`        | `simulate` or `measure` | Simulate fan (1 or 2) running at RPM 10 * PWM or to reset to real readings
`d15/debugset/kwl/fan/pwm/store_IKNOWWHATIMDOING` | (any) | Store PWM values to EEPROM for the current mode
`d15/debugstate/kwl/fan#`      | (status string)   | Status of fan (1 or 2), with timestamp, desired/actual speed difference, PWM value, desired and current speed


## Differential Pressure Sensor Debugging MQTT Topics

Topic                          | Value (Unit)            | Description
------------------------------ | ----------------------- | ---------------------------------
`d15/debugset/kwl/dp#`         | ###.# (Pa) or `measure` | Set reading of differential pressure sensor (1 or 2), `measure` to reset back to real readings


## Temperature Sensor Debugging MQTT Topics

Topic                                    | Value (Unit) | Description
---------------------------------------- | ------------ | ----------------------------------
`d15/debugset/kwl/t#`                    | ###.# (ºC)   | Set reading of temperature sensor (1-4), `measure` to reset back to real readings, `simulate` to set back to last simulated value
`d15/debugset/kwl/aussenluft/temperatur` | ###.# (ºC)   | Set reading of temperature sensor 1 (outside)
`d15/debugset/kwl/zuluft/temperatur`     | ###.# (ºC)   | Set reading of temperature sensor 2 (intake)
`d15/debugset/kwl/abluft/temperatur`     | ###.# (ºC)   | Set reading of temperature sensor 3 (outtake)
`d15/debugset/kwl/fortluft/temperatur`   | ###.# (ºC)   | Set reading of temperature sensor 4 (exhaust)


## NTP Time Debugging MQTT Topics

Topic                          | Value (Unit)            | Description
------------------------------ | ----------------------- | ---------------------------------
`d15/debugset/kwl/ntp/time`    | ########### (Unix time) | Set NTP time to specified Unix timestamp (seconds since 01.01.1970 GMT)
