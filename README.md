# robot-guts 
this is a block that together with ros-core abstracts away hardware components that can be used with ROS to the point of refering to them simply as guts. 
These can be . Mainly anything that you connect to an SBC via I2C, UART, and GPIO. 


---
## Configuration
I've done a major update to the way the configuration file gets handled, mainly adding a schema-based validator instead of nested error handling. This makes everything way more robust but also requires modifications. 
This solves many of the problems that kept out the joy of working on this project for a while. Will update this later with a proper guide, but for now, here's the schema. 

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
Again, this project went through major brain surgery. Oh and I also need to add the same schema based mechanism to the packags at https://github.com/cristidragomir97/robot-block-lib

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
