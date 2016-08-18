# photonMasterRemote
photon Master Alarm with multiple photon Remote nodes

common source generates a master alarm node or a remote node.  
select master by the #define PHOTON_MASTER line in parms.h
select remote by commenting our , i.e. //#define PHOTON_MASTER

The master my have any number of it's own sensors defined in writeTestConfiguration **
The master's representation of a remote sensor is defined with use set to
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
