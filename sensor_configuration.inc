/* SENSOR DEVICE CONFIGURATION
 * 03-Jul-2022
 *
 */
 #ifdef DEV_SENSOR_1 //PORT D4 = GARAGE OVERHEAD
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.master_idx[i] = i;  //Obsolete 
  configuration.sys_id[i] = 1;      //SHOULD HAVE NAME ??
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  #ifdef PHOTON_MASTER
    configuration.port[i] = 0;
    configuration.use[i] = SUB_REMOTE_SENSOR;
  #else
    configuration.port[i] = D4;
    configuration.use[i] = IO_SENSOR;
  #endif
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "GARAGE OVERHEAD", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 1: mcp_9808
#ifdef DEV_SENSOR_2
  for(int k=1;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.dev_addr[i][0] = MCP9808_I2CADDR;
  configuration.master_idx[i] = i;
  configuration.sys_id[i] = 1; // ADD BY SYSTEM NAME ??
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  #ifdef PHOTON_MASTER
  configuration.use[i] = SUB_REMOTE_THERMOMETER;
  #else
  configuration.use[i] = MCP9808_THERMOMETER;
  #endif
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "GARAGE TEMP", SENSOR_NAME_SIZE);
  i++;
#endif

#ifdef DEV_SENSOR_3
//add:5:5B:02:A:0:0:0:87:   DS2405 red star
const uint8_t testrom3[8] = { 0x05,0x5B,0x02,0x0A,0x00,0x00,0x00,0x87 };
  configuration.master_idx[i] = i;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom3[k];
  configuration.dev_flags[i] = (1 << DEVICE_PRIORITY) | (1<< DEVICE_REMOTE);
  #ifdef PHOTON_MASTER
  configuration.port[i] = 0;
  configuration.use[i] = SUB_REMOTE_SENSOR;
  #else
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  #endif
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "GARAGE MOTION", SENSOR_NAME_SIZE);
  i++;
#endif

// Weather-Sealed 1-W Thermometer: yellow
#ifdef DEV_SENSOR_4
const uint8_t testrom4[8] = { 0x28,0xaa,0xd9,0xf6,0x17,0x13,0x02,0x6d };
configuration.master_idx[i] = i;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom4[k];
  configuration.dev_flags[i] = 0;
  #ifdef PHOTON_MASTER
    configuration.port[i] = 0;
    configuration.use[i] = SUB_REMOTE_THERMOMETER;
  #else
    configuration.port[i] = PIO_B;
    configuration.use[i] = OW_THERMOMETER;
 #endif
configuration.sense[i] = 0;
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "TEMP GARAGE", SENSOR_NAME_SIZE);
i++;
#endif

// Weather-Sealed 1-W Thermometer: green
#ifdef DEV_SENSOR_5X
const uint8_t testrom5[8] = { 0x28,0xaa,0x11,0xf1,0x17,0x13,0x02,0xF7 };
configuration.master_idx[i] = i;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom5[k];
configuration.dev_flags[i] = 0;
 #ifdef PHOTON_MASTER
    configuration.port[i] = 0;
    configuration.use[i] = SUB_REMOTE_THERMOMETER;
  #else
    configuration.port[i] = PIO_B;
    configuration.use[i] = OW_THERMOMETER;
 #endif
configuration.sense[i] = 0;
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "TEMP XX", SENSOR_NAME_SIZE);
i++;
#endif

//new sensor ds2406p, smoke detector
#ifdef DEV_SENSOR_6
const uint8_t testrom6[8] = { 0x12,0x8E,0x15,0x6D,0x00,0x00,0x0,0xDB };
configuration.master_idx[i] = i;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom6[k];
configuration.dev_flags[i] = (1 << DEVICE_PRIORITY) | (1<< DEVICE_REMOTE);
configuration.port[i] = PIO_A;
configuration.use[i] = OW_SENSOR;
configuration.sense[i] = SENSE_NORMAL_OPEN;
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "GARAGE SMOKE", SENSOR_NAME_SIZE);
i++;
#endif

#ifdef DEV_SENSOR_7
//add:05:46:11:0A:00:00:00:A4  NEW DS2405 yellow star
const uint8_t testrom7[8] = { 0x05,0x46,0x11,0x0A,0x00,0x00,0x00,0xA4 };
  configuration.master_idx[i] = i;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom7[k];
  configuration.dev_flags[i] = (1 << DEVICE_PRIORITY) | (1<< DEVICE_REMOTE);
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "FRONT DOOR", SENSOR_NAME_SIZE);
  i++;
#endif

#ifdef DEV_SENSOR_8
//add:05:2D:16:0A:0:0:0:CE:   DS2405 blue star
const uint8_t testrom8[8] = { 0x05,0x2D,0x16,0x0A,0x00,0x00,0x00,0xCE };
  configuration.master_idx[i] = i;
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = testrom8[k];
  configuration.dev_flags[i] = (1 << DEVICE_PRIORITY)| (1<< DEVICE_REMOTE);
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_SENSOR;
  configuration.sense[i] = SENSE_NORMAL_CLOSED;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "GARAGE UNASSIGN", SENSOR_NAME_SIZE);
  i++;
#endif

#ifdef DEV_SENSOR_9 //ds2413 YELLOW star
const uint8_t testrom9[8] = { 0x3a,0x18,0xf1,0x5c,0x00,0x00,0x00,0xc4 };
  configuration.master_idx[i] = i;
  for(int k=0;k<8;k++)
      configuration.dev_addr[i][k] = testrom9[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  #ifdef PHOTON_MASTER
  configuration.port[i] = 0;
  configuration.use[i] = SUB_REMOTE_SENSOR;
  #else
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_INDICATOR;
  #endif
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "XGARAGE OVERHEAD", SENSOR_NAME_SIZE);
  i++;
#endif

// Weather-Sealed 1-W Thermometer: RED
#ifdef DEV_SENSOR_10
const uint8_t testrom10[8] = { 0x28,0xaa,0x0e,0x06,0x18,0x13,0x02,0x4c };
configuration.master_idx[i] = i;
configuration.sys_id[i] = 0;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom10[k];
configuration.dev_flags[i] = 0;
configuration.port[i] = PIO_B;
configuration.use[i] = OW_THERMOMETER;
configuration.sense[i] = 0;
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "PATIO TEMP", SENSOR_NAME_SIZE);
i++;
#endif

// Weather-Sealed 1-W Thermometer: BLUE
#ifdef DEV_SENSOR_11
const uint8_t testrom11[8] = { 0x28,0xaa,0x71,0x01,0x18,0x13,0x02,0xc8 };
configuration.master_idx[i] = i;
configuration.sys_id[i] = 0;
for(int k=0;k<8;k++)
  configuration.dev_addr[i][k] = testrom11[k];
configuration.dev_flags[i] = 0;
configuration.port[i] = PIO_B;
configuration.use[i] = OW_THERMOMETER;
configuration.sense[i] = 0;
configuration.alert_level[i] = ALWAYS_DEVICE;
configuration.alert_min[i] = 0;
configuration.alert_max[i] = 0;
strncpy(configuration.name[i], "LR TEMP", SENSOR_NAME_SIZE);
i++;
#endif

#ifdef DEV_SENSOR_12 //ds2413 ORANGE? star
const uint8_t testrom12[8] = { 0x3a,0x34,0xef,0x5c,0x00,0x00,0x00,0xd1 };
  configuration.master_idx[i] = i;
  configuration.sys_id[i] = 0;
  for(int k=0;k<8;k++)
      configuration.dev_addr[i][k] = testrom12[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_INDICATOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = HOME_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "LR MOTION", SENSOR_NAME_SIZE);
  i++;
#endif
#ifdef DEV_SENSOR_13 //ds2413 BLACK? star
const uint8_t testrom13[8] = { 0x3a,0x9e,0xef,0x5c,0x00,0x00,0x00,0x42 };
  configuration.master_idx[i] = i;
  configuration.sys_id[i] = 0;
  for(int k=0;k<8;k++)
      configuration.dev_addr[i][k] = testrom13[k];
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  configuration.port[i] = PIO_B;
  configuration.use[i] = OW_INDICATOR;
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "LR SLIDER", SENSOR_NAME_SIZE);
  i++;
#endif
//------------
#ifdef DEV_SENSOR_14                //PORT D4 = AIRSTREAM DOOR
  for(int k=0;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.master_idx[i] = i;  //Obsolete 
  configuration.sys_id[i] = 2;      //SHOULD HAVE NAME ??
  configuration.dev_flags[i] = 1 << DEVICE_PRIORITY;
  #ifdef PHOTON_MASTER
    configuration.port[i] = 0;
    configuration.use[i] = SUB_REMOTE_SENSOR;
  #else
    configuration.port[i] = D4;
    configuration.use[i] = IO_SENSOR;
  #endif
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "CAMPER DOOR", SENSOR_NAME_SIZE);
  i++;
#endif
  //Sensor 1: mcp_9808
#ifdef DEV_SENSOR_15
  for(int k=1;k<8;k++)
    configuration.dev_addr[i][k] = 0xff;
  configuration.dev_addr[i][0] = MCP9808_I2CADDR;
  configuration.master_idx[i] = i;
  configuration.sys_id[i] = 2; // ADD BY SYSTEM NAME ??
  configuration.dev_flags[i] = 0;
  configuration.port[i] = 0;
  #ifdef PHOTON_MASTER
  configuration.use[i] = SUB_REMOTE_THERMOMETER;
  #else
  configuration.use[i] = MCP9808_THERMOMETER;
  #endif
  configuration.sense[i] = SENSE_NORMAL_OPEN;
  configuration.alert_level[i] = AWAY_DEVICE;
  configuration.alert_min[i] = 0;
  configuration.alert_max[i] = 0;
  strncpy(configuration.name[i], "CAMPER TEMP", SENSOR_NAME_SIZE);
  i++;
#endif
