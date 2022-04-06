# robot-guts 

a balena-block that creates a ROS representation of the stuff your machine is made of. Components that are coneected to SBCs via I2C, UART, and GPIO and USB.  
--- 
In a world where ICs, modules and electronics are going trough long waiting times this provides robotics fleet owners the flexiblity to replace the hardware faster and deploy fleets with mixed hardware configurations while keeping the upper and more important layers away from all the variablity. 

## config.json
I've done a major update to the way the configuration file gets handled, mainly adding a schema-based validator instead of nested error handling.
I didn't have time to update documentation yet, but here is the schema used to validate the file: 

   "file" :{
        "type" : "object",
        "properties" : {
            "name" : {"type" : "string"},
            "desc" : {"type" : "string"},
            "guts: {"type" : "object"}
        },
    },

    "simple_guts" : {
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

    "complex_guts" : {
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

## Supported modules
I also need to add the same schema based mechanism to the packags at https://github.com/cristidragomir97/robot-block-lib and the way. 
Currently tested only with motorhead. 

* **Motor Drivers**: 
    * L298N GPIO (and any other motor controllers using PWM + DIR pins, eg. VNH3SP30)
    * Sparkfun Qwiic Motor Controler
    * Motorhead (here)[https://github.com/cristidragomir97/motorhead]
* **Encoders**: 
    * Quadrature Encoders (GPIO)
* **Sensors**: 
    * **Basics:**
        * GPIO Input 
        * I2C ADC 
    * **Ranging:**
        * VL53L1
        * SR04
    * **IMUs:**
        * LSM9DS1
        * MPU6050
* **Actuators**
    * GPIO Output 
    * GPIO PWM 
    * **Servo**:
        * Jetson GPIO (hardware PWM on pin 32, 32)
        * PCA9865 I2C Servos
