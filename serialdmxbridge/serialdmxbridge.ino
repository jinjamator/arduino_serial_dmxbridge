//#include <ArduinoRS485.h>
//#include <ArduinoDMX.h>
#include <DMXSerial.h>
#include <AltSoftSerial.h>
// #include <ArduinoJson.h>



const int universeSize = 10;
uint8_t val = 0;
AltSoftSerial SSerial;  // 8 RX, 9 TX

// packetformat: command,payload_length,payload

// set dmx channel 1 to 0x15
// 01 02 01 15

// reset
// ff 00

// universe set
// 02 02 20 50 

// universe get
//F2 00


typedef enum {
  COMMAND_NONE = 0x00,
  COMMAND_SET_CHANNEL = 0x01,
  COMMAND_SET_UNIVERSE = 0x02,
  COMMAND_SET_DEBUG = 0x03,
  COMMAND_UNSET_DEBUG = 0x04,
  COMMAND_GET_CHANNEL = 0xF1,
  COMMAND_GET_UNIVERSE = 0xF2,
  COMMAND_GET_DEBUG = 0xF3,
  COMMAND_STATUS = 0xFE,
  COMMAND_RESET = 0xFF,
} Command;


typedef enum {
  STATE_NONE = 0x00,
  STATE_WAITING_FOR_COMMAND = 0x01,
  STATE_WAITING_FOR_DATA = 0x02,
  STATE_FINISHED_READING = 0x03,
  STATE_ERROR = 0xFF,
} State;


char buffer[128];

typedef struct {
  Command command;
  uint16_t payload_len;
  byte payload[universeSize + 1];
} DataPacket;

DataPacket packet_read_buffer;
State current_state;

bool debug_enabled=false;
byte current_universe_status[universeSize + 1];

void debug(char *msg){
  if (debug_enabled){
    SSerial.println(msg);
  }
}

void setup() {
  
  SSerial.begin(9600);

  
  // initialize the DMX library with the universe size
  //if (!DMX.begin(universeSize)) {
  //  SSerial.println("Failed to initialize DMX!");
  //  while (1)
  //    ;  // wait for ever
  //}
  DMXSerial.init(DMXController);
  DMXSerial.maxChannel(universeSize);
  packet_read_buffer.payload_len = 0;
  packet_read_buffer.command = 0;
  memset(packet_read_buffer.payload, 0, sizeof(packet_read_buffer.payload));
  memset(current_universe_status, 0, sizeof(packet_read_buffer.payload));
  SSerial.println("initialized dmx");
  current_state=STATE_WAITING_FOR_COMMAND;
}

void loop() {
  
  //DMX.beginTransmission();
  if (current_state == STATE_WAITING_FOR_COMMAND){
    if (SSerial.available() >= 2) {
        packet_read_buffer.command=SSerial.read();
        packet_read_buffer.payload_len=SSerial.read();
        current_state=STATE_WAITING_FOR_DATA;
        if (packet_read_buffer.payload_len >= sizeof(packet_read_buffer.payload)){
          current_state=STATE_ERROR;
          sprintf(buffer, "ERROR cannot receive that much payload %d bytes DATA, available %d" ,packet_read_buffer.payload_len, sizeof(packet_read_buffer.payload));
          debug(buffer);

        }
        
        debug("GOT COMMAND");
        
    }
  }
  if (current_state==STATE_WAITING_FOR_DATA){
    if (SSerial.available() >= packet_read_buffer.payload_len) {
        SSerial.readBytes(packet_read_buffer.payload, packet_read_buffer.payload_len);
        current_state=STATE_FINISHED_READING;
        sprintf(buffer, "GOT %d bytes DATA" ,packet_read_buffer.payload_len);
        debug(buffer);
    }
    else{
      sprintf(buffer, "WAITING FOR %d bytes DATA, available %d" ,packet_read_buffer.payload_len, SSerial.available());
      debug(buffer);

    }
    //SSerial.readBytes(reinterpret_cast<char*>(&packet_read_buffer.payload_len), sizeof(packet_read_buffer.payload_len));
  }
  
  if (current_state == STATE_FINISHED_READING){
    switch (packet_read_buffer.command){
      case COMMAND_SET_CHANNEL:
        //DMX.write(packet_read_buffer.payload[0], packet_read_buffer.payload[1]);
        DMXSerial.write(packet_read_buffer.payload[0], packet_read_buffer.payload[1]);
        current_universe_status[packet_read_buffer.payload[0]]=packet_read_buffer.payload[1];
        sprintf(buffer, "WRITTEN DMX: %d %d" ,packet_read_buffer.payload[0]  ,packet_read_buffer.payload[1]);
        debug(buffer);
        
        packet_read_buffer.command == COMMAND_NONE;
        current_state=STATE_WAITING_FOR_COMMAND;
        SSerial.println("ok");
        break;
      case COMMAND_RESET:
        packet_read_buffer.command == COMMAND_NONE;
        memset(packet_read_buffer.payload, 0, sizeof(packet_read_buffer.payload));
        packet_read_buffer.payload_len=0;
        current_state=STATE_WAITING_FOR_COMMAND;
        sprintf(buffer, "RESET DONE");
        debug(buffer);
        SSerial.println("ok");
        break;
      case COMMAND_SET_UNIVERSE:
        for(int i=1; i<=universeSize && i <= packet_read_buffer.payload_len; i++){
          DMXSerial.write(i, packet_read_buffer.payload[i-1]);
          current_universe_status[i]=packet_read_buffer.payload[i-1];
          sprintf(buffer, "WRITTEN DMX: %d %d" ,i  ,packet_read_buffer.payload[i-1]);
          debug(buffer);
        }
        packet_read_buffer.command == COMMAND_NONE;
        current_state=STATE_WAITING_FOR_COMMAND;
        SSerial.println("ok");
        break;
      case COMMAND_GET_UNIVERSE:
        for (byte element : current_universe_status){
          sprintf(buffer, "%02X", element);
          SSerial.print(buffer);
        }
        SSerial.println("");
        packet_read_buffer.command == COMMAND_NONE;
        current_state=STATE_WAITING_FOR_COMMAND;
        break;
      case COMMAND_GET_CHANNEL:
        sprintf(buffer, "%02X", current_universe_status[packet_read_buffer.payload[0]]);
        packet_read_buffer.command == COMMAND_NONE;
        current_state=STATE_WAITING_FOR_COMMAND;
        SSerial.println(buffer);
        break;
      case COMMAND_GET_DEBUG:
        sprintf(buffer, "%02X", current_universe_status[packet_read_buffer.payload[0]]);
        packet_read_buffer.command == COMMAND_NONE;
        current_state=STATE_WAITING_FOR_COMMAND;
        SSerial.println(debug_enabled);
        break;
      case COMMAND_SET_DEBUG:
        debug_enabled=true;
        packet_read_buffer.command == COMMAND_NONE;
        current_state=STATE_WAITING_FOR_COMMAND;
        SSerial.println("ok");
        break;
      case COMMAND_UNSET_DEBUG:
        debug_enabled=false;
        packet_read_buffer.command == COMMAND_NONE;
        current_state=STATE_WAITING_FOR_COMMAND;
        SSerial.println("ok");
        break;

      default:
        current_state=STATE_ERROR;
        sprintf(buffer, "UNKNOWN COMMAND: %d" ,packet_read_buffer.command);
        SSerial.println(buffer);
        packet_read_buffer.command == COMMAND_NONE;
        current_state=STATE_WAITING_FOR_COMMAND;
    }

    }
    if (current_state == STATE_ERROR){

        sprintf(buffer, "ERROR: reseting");
        SSerial.println(buffer);
        current_state=STATE_FINISHED_READING;
        packet_read_buffer.command=COMMAND_RESET;
  }
  
  
  //DMX.endTransmission();
  
}
