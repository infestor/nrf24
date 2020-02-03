typedef struct {
  uint8_t addr;
  uint8_t  counter;
} ackRecord;

typedef enum {
  READY = 0,
  IN_FIFO = 1,
  WAIT_FREE_AIR = 2,
  WAIT_ACK = 3
} SENDING_STATUS;

typedef enum {
  SUCCESS = 0,
  PROCESSING = 1,
  TIMEOUT = 4,
  MAX_ATTEMPTS = 5
} SEND_RESULT;

typedef enum {
  RESERVED = 0,
  ACK = 1,
  ACK_RESP = 2,
  REQUEST = 3,
  RESPONSE = 4,
  PRESENTATION_REQUEST = 5,
  AUTOSEND = 6,
  RETRANSLATION = 7,
  CONFIGURATION = 8,
  PING = 9,
  PRESENTATION_RESPONSE = 10
} PACKET_TYPE;

typedef enum {
  ON_OFF_OUTPUT = 0,
  TEMP = 1,
  DOOR_SWITCH = 2,
  INTERNAL_TEMP = 3,
  DS1820_TEMP = 4,
  BATTERY_1_LION = 5,
  BATTERY_2_LION = 6,
  PWM = 7,
  PWM_3_CHANNEL = 8
} SENSOR_TYPE;

//payload related structures

typedef enum {
  READ = 0,
  WRITE = 1,
  WRITE_FIRST = 2,
  WRITE_MIDDLE = 3,
  WRITE_LAST = 4,
  CALIBRATION_WRITE = 5,
  CALIBRATION_READ = 6
} CMD_TYPE;

//for type REQUEST
typedef struct {
  uint8_t cmd; //command to be executed by receiving node
  uint8_t len;
  uint8_t for_sensor;
  uint8_t payload[4];
} payloadRequestStruct;


//for type RESPONSE (kupr odpoved na command READ)
typedef struct {
  uint8_t cmd; //executed command by receiving node (same as in request packet)
  uint8_t len;
  uint8_t from_sensor;
  uint8_t payload[4];
} payloadResponseStruct;

//for presentation packet
typedef struct {
  uint8_t num_sensors;
  uint8_t sensor_type[6];
} payloadPresentationStruct;
