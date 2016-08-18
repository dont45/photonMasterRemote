# photonMasterRemote
photon Master Alarm with multiple photon Remote nodes

common source generates a master alarm node or a remote node.  
select master by the #define PHOTON_MASTER line in parms.h, or
select remote by commenting out this #define , i.e. //#define PHOTON_MASTER

The master my have any number of it's own sensors defined in writeTestConfiguration **
The master's representation of a remote sensor is defined with device.use set to
SUB_MASTER_ALARM.  The actual sensor is defined in the remote with master_idx
set to the index number in the master which represents the remote sensor in the master.

communication of the remote sensor is via a particle.publish string which represents
the state of all of the remote sensors, such as:
3@T@13:12:16@TEST DOOR:2:0.000000:3@OUTSIDE TEMP:3:89.937500:0$
The @ character separates the first three data fields (sequence,alert type, and time).
Data for each sensor follows, also separated by an @ character.
These fields are parsed by the class Parse.

Each sensor's data fields are separated by a : character.  They are Name,device index in master,
sensor reading, and ??.  These data fields are also parsed by class Parse.

The remote node publishes 'remoteData' to the particle cloud, and the master node is subscribed
to this cloud data field.  The remote node performs a publish immediately upon any sensor being
tripped, and also periodically as an Information update on a specified schedule, set in
parms.h.  This allows for immediate response to an alarm condition and periodic read on the
master node of other less time-sensitive data such as a temperature probe.


Work in progress.  To be added is a remote system id, to be included in the remoteData message,
and to allow for identification of multiple remote nodes.

In my operational test system, the hardware for both master and remote can be identical, except
for the attached sensors.  The remote node may also be of any custom design, as required.  It
only needs to publish a properly formatted data string as 'remoteData'.

I have implemented and tested the following configurations:

Master alarm in my house with sensors to read house temperature, boiler temperature, water detector,
fire alarm detector, motion detector, and magnetic door closure detectors.  All of these sensors 
are 1-wire devices, such as a DS2413, DS1820, etc.  They are attached to the photon by use of
an I2C to 1-wire interface chip, the DS2482.

I have also tested two unusual remote nodes.  One a battery powered photon with an ultrasonic distance
measurement device to read the hone heating fuel tank level.  See class Tank.
The second remote node is an electron based node installed in my truck which reads ambient 
temperature using an MCP9808 sensor, and transmits alarms if the temperature exceeds the device
settings.  It also sends a periodic Informational message of worry Temp.  This is to protect
my dog from dangerous heat while being left in the truch for short periods of time.

I have impl
