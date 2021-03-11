#include <mavlink.h>
//#include <SoftwareSerial.h>
 
//#define RXpin 0
//#define TXpin 1
//SoftwareSerial SerialMAV(2, 3); // sets up serial communication on pins 3 and 2
 
void setup() {
  Serial.begin(57600); //Main serial port for console output
  Serial2.begin(57600); //RXTX from Pixhawk (Port 19,18 Arduino Mega)
  Serial.println("MAVLink starting.");

  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY;
  
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  Serial.write(buf, len);
  
  request_datastream();
}
 
void loop() {
MavLink_receive();
}
 
//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive()
  {
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(Serial2.available())
  {
    uint8_t c= Serial2.read();
 
    //Get new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
    //Handle new message from autopilot
      switch(msg.msgid)
      {
      case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_attitude_t packet;
        mavlink_msg_attitude_decode(&msg, &packet);
        
        Serial.print("\nRoll: ");Serial.println(packet.roll);
        Serial.print("Pitch: ");Serial.println(packet.pitch);
        Serial.print("Yaw: ");Serial.println(packet.yaw);
      }
      break;
 
      }
    }
  }
}
 
void request_datastream() {
//Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_RAW_CONTROLLER;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  uint8_t _start_stop = 1; //1 = start, 0 = stop

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
 
  Serial2.write(buf, len); //Write data to serial port
}
