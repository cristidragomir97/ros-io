# ros-io

A hardware abstraction layer for modules and chips that are connected to I2C, UART and GPIO interfaces. This reduces friction by eliminating the need to write low-level code, firmware or ROS nodes and replaces it with a config file similar to the way docker-compose works. 

In a world where ICs, modules and electronics are going trough long waiting times this helps makers prototype much faster, and allows robotics fleet owners the flexiblity to replace the hardware quickly and deploy fleets with mixed hardware configurations.

Here's how this all works:
* You define your hardware in a configuration file and upload to a git repository
* ros-io downloads your config file, parses its contents and installs the proper packages and 3rd party dependencies
* Package code gets imported into the ros-io code, and worker threads are deployed:
	* Workers wrap the part object, create ROS messages and map the read/update functions to rospy Subscriber and Publisher objects
	


## How to use ros-io 

### 1. Add to ros-io docker-compose.yaml
```yaml
ros-io:
  privileged: true 
  image: cristidragomir97/ros-io
  environment:
    - ROS_HOSTNAME=ros-io 
    - ROS_MASTER_URI=http://ros-core:11311
    - CONFIG_REPO=https://github.com/cristidragomir97/ep1-rc-car
  volumes:
    - ros-bin:/opt/ros/noetic
  devices:
    - "/dev:/dev"
```

Of course, your configuration might vary, but here are a few pointers:
* `privileged: true` is required for access to hardware 
* `ROS_HOSTNAME ` has to match the service name more info on ROS, docker and networking [here](https://github.com/cristidragomir97/robotics-images)
* `CONFIG_REPO` is where ros-io can find your configuration file. It expects to find a file called `config.json` at the root of your repository. 
* ros-io doesn't contain any ROS binaries, these are loaded from the ros-core service using a volume share:
```yaml 
volumes:
  - ros-bin:/opt/ros/noetic
```
	
--- 

### 2. config file
The `config.json` file is conceptually very similar to `docker-compose.yaml`. It defines what parts you are using, how to access them, and how to expose them to ROS. The config file has the following structure:

```json
{
	"name": "",
	"desc": "",
	"downloads": {}, 
	"parts": {}, 
}
```

---

#### 2.1 Downloads
By default ros-io doesn't contain any packages, however you can specify multiple sources for those in the `Downloads` section of the config file. 
```json
"downloads":{
		"repos":[
			["https://github.com/cristidragomir97/motorhead", "./library/motorhead"],
			["https://github.com/cristidragomir97/robot-block-lib", "./library/core-lib"]
		],
},
```

--- 

#### 2.2 Parts
ros-io supports two types of parts:
* **Simple parts** expose one ROS topic / part
* **Multi-channel** parts like ADCs or PWM drivers, these expose a separate ROS topic for each one of the channels.

The following fields are mandatory for any type of part:
* `package` is the name of the package. 
* `folder` this is where on the filesystem your package is residing. That should usually be the path you have defined in the downloads section plus the package name. 
* `address` tells ros-io how to physically talk to your device.
* Everything in `args` will be unpacked and directly passed to the constructor of your part as arguments. This is how you can configure the package itself. 

##### 2.2.1 Simple Part Example
A good example of a simple part is the [motorhead](https://github.com/cristidragomir97/motorhead) motor-driver. What defines a simple parts, is that they can only expose a single `topic` and have a single `role`

```json
"motor_driver": {
	"role": "subscriber",
	"topic": "/cmd_vel",
	"folder": "/motorhead",
	"package": "motorhead",
	"address": "0x76",
	"args":{
		"radius": 0.0325,
		"flip": "true",
		"pins": {
			"right_a": 5,
			"right_b": 4,
			"right_pwm": 3,
			"left_a": 8,
			"left_b": 7,
			"left_pwm": 6,
			"right_enc_a": 10,
			"right_enc_b": 11,
			"left_enc_a": 12,
			"left_enc_b": 13
		}
	}
},
```
##### 2.2.2 Multi-Channel Part Example
The configuration structure for multi-channel adds another two mandatory fields, called `channel_no` and `channels`:

* `channel_no` tells ros-io how many of the devices channels will be used and 
* `channels` is where you define the `role` and `topic` for each one of the channels. That's because each channel gets it's own instance of **Subscriber** or **Publisher**.

Oh, and depending on your part, ros-io supports mixed role parts. One channel could be an input while the other is output. 

The ADS1115 ADC is a great example of a multi-channel part. 

```json
"ADC": {
	"folder": "core-lib/ADS1115",
	"package": "ADS1115",
	"address": "0x48",
	"channel_no": 4,
		"channels":{
			"front_floor_right": {
				"pin": 0,
				"role": "publisher",
				"topic": "/floor/front_right",
				"args":{}
			},
			"front_left_floor": {
				"pin": 1,
				"role": "publisher",
				"topic": "/floor/front_left",
				"args":{}
			},
			"right_floor": {
				"pin": 2,
				"role": "publisher",
				"topic": "/floor/right",
				"args":{}
			},
			"left_floor": {
				"pin": 3,
				"role": "publisher",
				"topic": "/floor/left",
				"args":{}
			}
		},
		"args":{}
	}
```

--- 

### 3. Parts, packages and the library
To understand the role of each of these, we first need to define a few terms:
* **Package** - the code and the configuration file 
* **Library** - the library doesn't really exist anywhere, it's just the collection of packages that gets downloaded for your solution
* **Part** - a part is an instance of a package. ros-io supports two types of parts:


### 3.1 Core Packages
Here are a few of the packages I have written for parts I had lying around.
| Name      | Type.  |Desc. 
| ----------- | ----------- | ----------- | 
| [**ADS1015**](https://github.com/cristidragomir97/robot-block-lib/tree/main/ADS1115) | Interface | 4-channel 12-bit I2C ADC | 
| [**VL53L1_Array**](https://github.com/cristidragomir97/robot-block-lib/tree/main/VL53L1_Array) | Range  Sensor | Configurable array of ToF Sensors |
| [**ICM20948**](https://github.com/cristidragomir97/robot-block-lib/tree/main/ICM20948) | Motion Sensor | 9-Axis MEMS IMU  |
| [**LSM9DS1**](https://github.com/cristidragomir97/robot-block-lib/tree/main/LSM9DS1) | Motion Sensor | 6-Axis MEMS IMU |
| [**SparkfunTwist**](https://github.com/cristidragomir97/robot-block-lib/tree/main/SparkfunTwist) | Sensor | Sparkfun Dual Encoder Reader |
| [**INA219**](https://github.com/cristidragomir97/robot-block-lib/tree/main/INA219) | Power Sensor | Voltage/Current/Power Sensor |
| [**4245-PSOC**](https://github.com/cristidragomir97/robot-block-lib/tree/main/4245-PSOC) | Motor Driver | Serial/I2C Motor Driver found in Sparkfun Auto pHat  


### 3.2 How to add packages for new parts
Creating a ros-io package for your part is pretty straightforward. Most breakout boards and modules from vendors like Seeed, Adafruit or Sparkfun come with libraries and examples for Python and Arduino.

Vendors take care of the low level communication between the part and your SBC, ros-io takes care of the ROS communication, a package is the glue between those. 

Let's start with a concrete example, the ADS1x15 series of Analog-Digital-Converters. 

#### 3.2.1 Get information on the part
First step is to investigate and analyse the part you are going to write a package for. A great starting point is the repository of a vendors' library for that part. [Adafruit_CircuitPython_ADS1x15](https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15) in our case. Investigating their example code we can find out the steps needed to comunicate with our module. 

1. Imports:
	 ```python
	import time, board, busio
	import adafruit_ads1x15.ads1015 as ADS
	from adafruit_ads1x15.analog_in import AnalogIn
	```
2. Initialization 
	```python
	# Create the I2C bus
	i2c = busio.I2C(board.SCL, board.SDA)
	# Create the ADC object using the I2C bus
	ads = ADS.ADS1015(i2c)
	```
4. Create channel object: 
	```python
	chan = AnalogIn(ads, ADS.P0)
	```

4. Get values from channel:
```chan.value, chan.voltage```
		

#### 3.2.2 Pick ROS Message type
In the case of our ADC, values are integers between (0-4096), so our decision here is pretty simple. Check out  [std_msgs](http://wiki.ros.org/std_msgs) for a list of base message types. 
For more specific hardware, you might need something  more complex, such as `sensor_msgs.msg.Imu` for, or `geometry_msgs.msg.Twist` for motor controllers. These are usually found in [common_msgs](http://wiki.ros.org/common_msgs).

You will have to take care of encoding/decoding these messages inside your package code.  For more information on how to do that check out [this tutorial](http://wiki.ros.org/rospy/Overview/Messages). 

```python
from std_msgs.msg import Int32

... 

def create_msg(value):
	msg = Int32()
	msg.data = value
	return  msg
```



#### 3.2.3 Write configuration file 
Package folder must contain a JSON configuration file that defines it's properties, dependencies, ROS message types and callback functions. All fields in this example are mandatory.
    
 ```json
 {
    	"name":  "ADS1115",
    	"info":  "4-channel 12-bit I2C ADC",
    	"dependencies":  [{
    		"type":  "pip3",
    		"package":"adafruit-circuitpython-ads1x15"
    	}],
    	"callback":  ["read0","read1","read2","read3"],
    	"ros_message":  ["std_msgs.msg",  "Int32"]
}
```

#### 3.2.4 Write package code
Your package code can be any valid python code, however, some conventions must be respected:

* On runtime, ros-io injects `rospy` into your scope, you can use everything you want from there to aid in writing your package.
* For the dynamic imports to work the package ` ADS1015`, config file `ADS1015.json` , python file `ADS1015.py` and constructor `ADS1015(args)` must all share the same name. 

* The interface between ros-io and your package are the object constructor and callback methods. These callback functions can be called however you want as long as you specify that in `package.json`. However, for simplicity i suggest using `update` for subscribers, and `read` for publishers. 

* devices with multiple channels must expose `read` and `update` callbacks for each channel. eg: `["read0","read1","read2","read3"]`
* Let us know about the packages you write, we'd be more than happy to add them on the list of supported parts. 


