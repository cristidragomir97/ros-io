# ros-io

A hardware abstraction layer for modules and chips that are connected to I2C, UART and GPIO interfaces. This reduces friction by eliminating the need to write low-level code, firmware or ROS nodes and replaces it with a config file similar to the way docker-compose works. 

--- 
In a world where ICs, modules and electronics are going trough long waiting times this provides robotics fleet owners the flexiblity to replace the hardware faster and deploy fleets with mixed hardware configurations while keeping the upper and more important layers away from all the variablity. 

## config.json
I've done a major update to the way the configuration file gets handled, mainly adding a schema-based validator instead of nested error handling.
I didn't have time to update documentation yet, but here is the schema used to validate the file: 

```json
{
   "file" :{
        "type" : "object",
        "properties" : {
            "name" : {"type" : "string"},
            "desc" : {"type" : "string"},
            "parts": {"type" : "object"}
        },
    },

    "simple_parts : {
        "type" : "object",
        "properties": {
                "role" : {"type": "string"},
                "topic" : {"type": "string"},
                "repo" : {"type": "string"},
                "library" : {"type": "string"},
                "address" : {"type": "string"},
                "args": {"type" : "object"}
        }, 
        "required": ["role", "topic", "repo", "library", "address"]
    }, 

    "complex_parts" : {
        "type" : "object",
        "properties": {
                "repo" : {"type": "string"},
                "library" : {"type": "string"},
                "address" : {"type": "string"},
                "channel_no": {"type": "number"},
                "channels": {"type" : "object"},
                "args": {"type" : "object"},
            }, 
            "required": ["repo", "library", "address",  "channel_no", "channels"]
    },

    "channel":{
        "type" : "object",
        "properties": {
            "role" : {"type" : "string"},
            "topic": {"type" : "string"},
            "channel": {"type" : "string"},
            "args": {"type" : "object"}
        },
        "required": ["role", "topic", "channel"]
    }
}
```

## Supported parts
| Name      | Type.  |Desc. 
| ----------- | ----------- | ----------- | 
| **ADS1015** | Interface | 4-channel 12-bit I2C ADC | 
| **pi-gpio** | Interface | Raspberry Pi GPIO Wrapper | 
| **VL53L1** | Range Sensor | ToF Sensor |
| **VL53L1_Array** | Range  Sensor | Configurable array of ToF Sensors |
| **ICM20948** | Motion Sensor | 9-Axis MEMS IMU  |
| **LSM9DS1** | Motion Sensor | 6-Axis MEMS IMU |
| **SparkfunTwist** | Sensor | Sparkfun Dual Encoder Reader |
| **INA219** | Power Sensor | Voltage/Current/Power Sensor |
| [**4245-PSOC**](https://github.com/cristidragomir97/robot-block-lib/tree/main/4245-PSOC) | Motor Driver | Serial/I2C Motor Driver found in Sparkfun Auto pHat  

## How to write a wrapper for a new part

