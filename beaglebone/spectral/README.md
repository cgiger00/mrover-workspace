Code for the AS7265x Spectral Triad Sensor and for the AS7263 Single Spectral Sensors
======================================================================================
### About
This is the code for running the AS7265x spectral traid sensor and three AS7263 single spectral sensors that are muxed to a TCA9548A i2c switch multiplex sensor. Upon receiving an LCM command it will output the channel readings of the specified spectral sensor. The triad will output data for 18 channels, each single sensor will output data for 6 channels and 0s for the rest.  

#### LCM Channels Publishing/Subscribed To 
**Spectral Data [publisher]** \
Messages: [SpectralData.lcm](https://github.com/cgiger00/mrover-workspace/blob/spectral/rover_msgs/SpectralData.lcm) "/spectral_data_publish" \
Publishers: beaglebone/spectral \
Subscribers: base_station/gui

**Spectral Command [subscriber]** \
Messages: [SpectralCmd.lcm](https://github.com/cgiger00/mrover-workspace/blob/spectral/rover_msgs/SpectralCmd.lcm) "/spectral_cmd" \
Publishers: base_station/gui \
Subscribers: beaglebone/spectral 

### Usage
Required electrical components: \
1 AS7265x spectral traid sensor \
3 AS7263 spectral sensors \
1 TCA9548A multiplex \
1 beaglebone green/back board 

Wire up the sensors to the multiplex so that the triad is in SDA/SCL port 0, and the spectral sensors are in ports 1, 2, and 3. Connect the multiplex to the i2c port on the beaglebone. 

To execute: \
```$ cd ~/mrover-workspace/```  to move to the mrover-workspace directory \
```$ ./jarvis build beaglebone/spectral```  to build the spectral program \
```$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec beaglebone/spectral``` 

#### LCM Commands
To get readings from the sensor: \
In a new terminal \
```$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_echo SpectralData "/spectral_data_publish"```

In the original terminal \
```$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_send "/spectral_cmd" "{'type': 'SpectralCmd', 'sensor': $SENSOR_NUM } ``` \
where $SENSOR_NUM is either 0 to get readings from the triad, or 1 - 3 to get readings from the respective single spectral sensors connected to the multiplex ports 1, 2, and 3

### Off Nominal Behavior Handling
None

### Common Errors
#### lcm_tools_echo or lcm_tools_send does not exist
In the terminal: \
```$ ./jarvis build lcm_tools/echo``` \
```$ ./jarvis build lcm_tools/send``` 

### To Do
- [x] Validate that the spectral data being output is correct 
- [x] Run this code connected to the Jetson TX2

### Notes
This code has been tested with python 3.6 on a beaglebone black. \
If you have any issues building the code, this is likely a result of not having the
[correct packages installed](https://github.com/umrover/mrover-workspace/wiki/Jarvis-Build-System#trouble) or [not being connected to the internet](https://docs.google.com/document/d/1jvlgHowy4Wunztz6Fqz8K6ajLKeHnPeXIuHctarIlso/edit#heading=h.renhpmsh260w). 
