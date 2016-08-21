# photonMasterRemote
A photon Master Alarm system with any number of optional photon Remote nodes

Common source generates a master alarm node or a remote node.  
The master node is created by the #define PHOTON_MASTER line in parms.h.
A remote node is created by commenting out the master node #define , i.e. 
//#define PHOTON_MASTER which will result in a #define PHOTON_REMOTE
In addition, you must inclue the line #define REMOTE_SYSTEM_ID 1
The id number must be 1 or greater.  The id of the master node must be 0.

The master my have any number of it's own sensors defined in writeTestConfiguration.
The master's representation of a remote sensor is defined with device.use set to
SUB_REMOTE_THERMOMETER or SUB_REMOTE_SENSOR.  
The actual sensor is defined in the remote with master_idx set to the index number 
in the master which represents the remote sensor.

Communication of the remote sensor status is done via a particle.publish string which represents
the state of all of the remote sensors, such as:
3@1@T@13:12:16@TEST DOOR:2:0.000000:3@OUTSIDE TEMP:3:89.937500:0$
The @ character separates the first four data fields (sequence,remote id, alert type, and time).
Data for each sensor follows, also separated by an @ character.
These fields are parsed by the class Parse.

Each sensor's data fields are separated by a : character.  They are Name,device index in master,
and sensor reading.  These data fields are also parsed by class Parse.

The remote node publishes 'remoteData' to the particle cloud, and the master node is subscribed
to this cloud data field.  The remote node performs a publish immediately upon any sensor being
tripped, and also periodically as an Information update on a specified schedule, set in
parms.h.  This allows for immediate response to an alarm condition and periodic read on the
master node of other less time-sensitive data such as a temperature probe.  Upon receipt of the
subscribed remoteData, the master device is update with the new sensor data, stored into the
device record as dev_reading, and the current time is stored into dev_last_read.

In my operational test system, the hardware for both master and remote can be identical, except
for the attached sensors.  The remote node may also be of any custom design, as required.  It
only needs to publish a properly formatted data string as 'remoteData'.

I have implemented and tested the following configurations:

Master alarm in my house with sensors to read house temperature, boiler temperature, water detector,
fire alarm detector, motion detector, and magnetic door closure detectors.  All of these sensors
are 1-wire devices, such as a DS2413, DS1820, etc.  Temperature sensors can be either a 1-wire
device such as a DS18B20 or an I2C attached device such as an MCP9808 precision thermometer.
All 1-wire devices are attached to the photon by use of an I2C to 1-wire interface chip, the DS2482.
My home alarm / monitor system also includes two remote nodes.  Both of these nodes are implemented
with identical physical hardware, my custom PCB manufacture by PCBway from my Eagle design files.
The two remote nodes are 1) a remote alarm installed in my storage shed with a door sensor and an 
MCP9808 reading outside temperature, and 2) a remote node installed in my garage with a door sensor
for each garage door, motion-detector to cover the garage area, and a door sensor on the entry door
to the house.   This was implemented as a remote node in order to eliminate physical wiring back to
the master alarm.

I have also tested two unusual remote nodes.  One a battery powered photon with an ultrasonic distance
measurement device to read the hone heating fuel tank level.  See class Tank.
The second remote node is an Particle Electron based node installed in my truck which reads ambient 
temperature using an MCP9808 sensor, and transmits alarms if the temperature exceeds the device
settings.  It also sends a periodic Informational message of worry Temp.  This is to protect
my dog from dangerous heat while being left in the truch for short periods of time.  Both of these
remote nodes function like any remote node.  They simply publish 'remoteData' which is processed
by the master alarm.  Me and my truck can be thousands of miles away from home, and my home master
alarm functions to handle these high-temperature alerts from my truck to my iPhone just like any other
alarm sensor.

The master alarm publishes some data but the principal communication
to the user is done using web hooks to send alerts to my iPhone via Pushover.  

I have designed an iPhone app which communicates to the master alarm node.  The iPhone app receives a
minimal amount of data from the master directly.  These include alarm state and the current temperature
from one selected probe.  I have it configured to report the outside temperature from the shed.

A large amount of detail of the alarm configuration and current state is available by selection from
a drop-down menu in the app.  The resulting data is pushed to the iPhone as an alert message.
The commands available in the drop-down are:
HLP Help lists the available commands
ABT About tells a little about the system including the version
TMP Displays the current reading of all Temperature Sensors
SET Set the Alarm system to Enabled
DIS Disable the Alarm System
ACK Acknowledge an alarm condition
HOU Set the number of hours between worry pages
LIS List  basic information about sensors
NAM Set the Name of a sensor
SEN Show all detail of a sensor
ACT Set a sensor to active
DEA Disable a sensor
MIN Set minimum value to trigger an alert
MAX Set maximum value to tirgger an alert
