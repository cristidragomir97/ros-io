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

    "simple_parts" : {
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

### Core Library 
| Name      | Type.  |Desc. 
| ----------- | ----------- | ----------- | 
| [**ADS1015**](https://github.com/cristidragomir97/robot-block-lib/tree/main/ADS1115) | Interface | 4-channel 12-bit I2C ADC | 
| [**VL53L1_Array**](https://github.com/cristidragomir97/robot-block-lib/tree/main/VL53L1_Array) | Range  Sensor | Configurable array of ToF Sensors |
| [**ICM20948**](https://github.com/cristidragomir97/robot-block-lib/tree/main/ICM20948) | Motion Sensor | 9-Axis MEMS IMU  |
| [**LSM9DS1**](https://github.com/cristidragomir97/robot-block-lib/tree/main/LSM9DS1) | Motion Sensor | 6-Axis MEMS IMU |
| [**SparkfunTwist**](https://github.com/cristidragomir97/robot-block-lib/tree/main/SparkfunTwist) | Sensor | Sparkfun Dual Encoder Reader |
| [**INA219**](https://github.com/cristidragomir97/robot-block-lib/tree/main/INA219) | Power Sensor | Voltage/Current/Power Sensor |
| [**4245-PSOC**](https://github.com/cristidragomir97/robot-block-lib/tree/main/4245-PSOC) | Motor Driver | Serial/I2C Motor Driver found in Sparkfun Auto pHat  

### How to add packages for new parts

You can basically write any valid python code as long as some conventions are followed. 
Let's take ADS1015, a 4-channel Analog-Digital Converter as an example.

1. Package folder must contain a JSON configuration file that defines it's properties, dependencies, ros message types and callback functions. All fields in this example are mandatory. We also take care of dependencies for your as long as you add them to your config. (currently only pip packages are supported)
    
    ```json
    {
    	"name":  "ADS1115",
    	"info":  "4-channel 12-bit I2C ADC",
    	"dependencies":  [
    
    		{
    		"type":  "pip3",
    		"package":"adafruit-circuitpython-ads1x15"
    		}
    
    	],
    	"callback":  ["read0","read1","read2","read3"],
    	"ros_message":  ["std_msgs.msg",  "Int32"]
    }
    ```
    
2. For the dynamic imports to work, folder name, config file, python file and class constructor must all share the same name:
ADS1015, ADS1015.json, ADS1015.py, `ADS1015(args)`
3. devices with multiple channels must expose `read` and `update` callbacks for each channel: `["read0","read1","read2","read3"]`
4. You can make use of the utility functions in robot-block as they get imported at runtime. Take a look at [utils.py](https://github.com/cristidragomir97/ros-io/blob/master/robot/src/core/utils.py)
