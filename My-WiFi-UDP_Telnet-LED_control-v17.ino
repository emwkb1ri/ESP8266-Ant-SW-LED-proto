/* 
 WiFi/Telnet antenna relay controller using LinkSprint LinkNode R8 ESP8266 controller

  Copyright (c) 2018 Eric Wagner. All rights reserved.
  This file is part of the ESP8266WiFi library for Arduino environment.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* This sketch is my current working model for my remote antenna switch 
 *  controller.  It still needs some cleanup for robustness.  Things like
 *  automatic reconnect and some sort of heartbeat status sent when connected
 *  and possibly some sort of broadcast status when not connected.  
 *  
 *  For now the simplified telnet connection with straightforward 2 byte antenna
 *  set commands will suffice for this working version.  I am using the
 *  LinkSprite Linknode R8 with this custom sketch.  Any ESP8266 or Arduino with
 *  WiFi could be easily adapted for this project.
 *  
 */
 
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266WiFiMulti.h>
#include <PString.h>

ESP8266WiFiMulti wifiMulti;

/* ############################################################################### 
 * ver 11 - fixed long delay in acting on command from client application
 *          looked for '/n', the last character sent rather than '/r' that 
 *          turned out to be the second last character.
 * ver 12 - Added WiFiMulti to connect to best WiFi access point.
 *        - Added second WiFI AP constants
 *        - moved the max clients conected message inside the proper if statement
 *        - made MAX_SRV_CLIENTS 2
 *        - added additional comments throughout code
 *        - added the revision history table
 *        - simplified the command parsing if/else instructions
 *        - use for loop to initialize relay control pins
 *        - removed debugging code timers except for full path 
 * ver 13 - various code cleanup items
 * ver 14 - add PString library
 *        - fix status message response string assembly using PString
 *        - added response time stats to status message string
 * ver 15 - added client status response back into the flow
 *        - fixed status response buffer type to string conversion for status
 *          response
 *        - fixed command response buffer type to string conversion for normal 
 *          respons
 *        - minor clean up to console debug messages related to status message
 *        - increased message buffer arrays sizes
 *        - removed double printing of status message in printWiFi function
 *        - added antenna pin state to status message
 *        - antenna pin state shown A1 to A8 (left to right) match switch display
 *          1 = on
 *        - made all the antenna prompts '>>' some were only single - now serial 
 *          output should match client responses
 * ver 16 - send replys to all connected clients
 *        - move Antenna pin state into status reply message
 *        - server.flush() upon connection to eliminate trash from telnet clients
 * ver 17 - chagned status reply message to only return the pin state       
 *          a shorter message to test issues in the Python code
 *        - created a new function getCurrentPins() for this purpose
 *        - changed initialization of the PString msg_Status using str.begin()
 * ver 18 - fixed initialization of msg_Stat string - status now works
 *        - returned to long status message with printWifiData
 *        - adjusted linefeeds and minor formatting changes to status message
 *        - added last req response time to the status message - indicates
 *          the status message response time
 *        - calculate response time for all antenna change requests not status
 *
 *        
 * ###############################################################################
 */
const int ver_num = 18;


//how many clients should be able to telnet to this ESP8266

#define MAX_SRV_CLIENTS 2

// WiFi network ssid(s) and password(s) available for connections
const char* ssid = "emwhome-5G";
const char* password = "emwkb1ri";
const char* ssid1 = "ATT5gNl9By";  //alternate network
const char* password1 = "3d4du+8e69ky";  //alternate network

// network constants and variables
byte mac[6];
const boolean udp_mode = false; 
unsigned int localPort = 5555;
unsigned int telnetPort = 23;



// Define variables and functions
char packetBuffer[255];  // buffer for UDP mode reads
char ReplyBuffer[1024];
char buffer[1024];         // buffer for status message PString conversions
String s = "" ;           // usde for client reply formation
unsigned int l;           // temp variable for command buffer to string conversion
char r[128];               // buffer for command conversion to char string type
boolean wifiConnected = false;
boolean connectWifi();
String command;           // received command string
String req = "";          // received command string
PString msg_Status(buffer, sizeof(buffer));
String msg_Stat = "";

unsigned long rt_total = 0; 
int num_req = 0;
unsigned long rt_max = 0;
unsigned long rt_min = 0;
unsigned long rt_avg = 0;
unsigned long rt_last = 0;
unsigned long t0 = 0; // initial time
unsigned long t1 = 0; // delta time 1
unsigned long t2 = 0; // delta time 2
unsigned long t3 = 0; // delta time 3
unsigned long t4 = 0; // delta time 4
unsigned long t5 = 0; // delta time 5
unsigned long t6 = 0; // delta time 6
unsigned long t7 = 0; // delta time 7
  
/*LinkNode R8 GPIO Table - verified these against physical relay
 * that are unmarked on the board.
S1 - GPIO 12 - HIGH = ON
S2 - GPIO 16
S3 - GPIO 4
S4 - GPIO 10
S5 - GPIO 15
S6 - GPIO 5
S7 - GPIO 13
S8 - GPIO 14

LED/Relay ON pattern for LinkNode R8 - pattern of 8 LEDs
      Ant-6   Ant-3   Ant-8   Ant-1   = Antenna #
       D12     D11     D4      D10    = ESP8266/R8 digital output
       **      **      **      **     = LED

      Ant-5   Ant-4   Ant-7   Ant-2   = Antenna #
       D16     D18     D8      D3     = ESP8266/R8 digital output
       **      **      **      **     = LED
*/

const int antS1 = 12;
const int antS2 = 16;
const int antS3 = 4;
const int antS4 = 10;
const int antS5 = 15;
const int antS6 = 5;
const int antS7 = 13;
const int antS8 = 14;

const int antpin[] = {12, 16, 4, 10, 15, 5, 13, 14};

// GPIO set/clear functions

int ant0_On();
int ant1_On();
int ant2_On();
int ant3_On();
int ant4_On();
int ant5_On();
int ant6_On();
int ant7_On();
int ant8_On();

// Create a server port to listen on
WiFiServer server(telnetPort);
WiFiClient serverClients[MAX_SRV_CLIENTS];

// Create a UDP instance
WiFiUDP Udp;

// Arduino initialization/setup function 
void setup() {
  Serial.begin(115200);
  delay(1000);

  // prepare GPIO pins and set initial values - HIGH = Relay ON/proto LED OFF
  for (unsigned int i = 0; i < 8; i++) {
    pinMode(antpin[i], OUTPUT);
    digitalWrite(antpin[i], LOW);
  }
  // Initialize wifi connection
  wifiConnected = connectWifi();
  
  // start server
  server.begin();
  server.setNoDelay(true);

  if(udp_mode) {
    // Start UDP server
    Serial.print("\nStarting UDP connection listener on ");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(localPort);
  
    // Start listening for UDP connections, report back via serial
    Udp.begin(localPort);
  }
  else {
    localPort = telnetPort;
    Serial.print("Ready! ... Use 'telnet ");
    Serial.print(WiFi.localIP());
    Serial.print(' ');
    Serial.print(localPort);
    Serial.println("' to connect");
  }
}

// This might need to be defined as a char or byte array for better compatibility
// with network TCP/UDP transfer protocols to my Python app

void loop() {
  boolean err = false;

  if (!udp_mode) {
    uint8_t i;
    uint8_t n;
    //check if there are any new clients
    if (server.hasClient()){
      Serial.print("hasClient() = ");
      Serial.println(server.hasClient());
      for(i = 0; i < MAX_SRV_CLIENTS; i++){        
        //find free or disconnected client slot
/*        
        Serial.print("i = ");
        Serial.print(i);
        Serial.print(";  ");
        Serial.print(serverClients[i]);
        Serial.print(";  ");
        Serial.print(serverClients[i].connected());
        Serial.print(";  ");
        Serial.print(server.available());
        Serial.println(";");
*/        
        if (!serverClients[i] || !serverClients[i].connected()){
          if(serverClients[i]) serverClients[i].stop(); // stop client [i} if disconnected
          serverClients[i] = server.available();
          serverClients[i].flush();
          Serial.print("New client: "); 
          Serial.print(i);
          Serial.println("");
          Serial.println("SW_Ready>>");
          serverClients[i].write("SW_Ready>>");
          continue;
        }
      }
      //no free/disconnected spot so reject
      WiFiClient serverClient = server.available();
      Serial.println("Refused connection - Max clients already connected");
      serverClient.stop();
    }
  
    //check clients for data
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      req = "";  // initialize req for each client loop
      if (serverClients[i] && serverClients[i].connected()){

        if(serverClients[i].available()){
          t0 = micros();  // intialize time when command is received
          //get data from the telnet client and push it to the UART

          while(serverClients[i].available()) {
            command = serverClients[i].readStringUntil('\n'); 
            Serial.print(command);
            req += command;
          }

          // set the antennas based on 'req'
          err = set_ant();

          if (err) {      // invalid request - send invalid response            
            Serial.print("invalid request -->");
            Serial.println(req);
            Serial.print("\r\nSW_Ready>>");
            for (n = 0; n < MAX_SRV_CLIENTS; n++) {
              if (serverClients[i].connected()) {
                serverClients[n].write("\r\nInvalid Request -->");
                serverClients[n].write(req.c_str());
                serverClients[n].write("\r\nSW_Ready>>");
                serverClients[n].flush();
              }
            }
            
          }
          else if (!err){      // valid command - send a response to the client
            
            // print to the console for debugging
            Serial.print("+");
            Serial.print(req);
            Serial.print("\r\nSW_Ready>>");
            
            // Assemble the response string
            s = "+" ;
            l = req.length();
            req.toCharArray(r,l);
            s += r;
            s += ">>";
            
            if (s == "+AS>>" || s == "+BS>>") {
              msg_Stat = msg_Status;  // append PString msg_Status to String msg_Stat
              msg_Stat += s;
              
              // send to all clients
              for (n = 0; n < MAX_SRV_CLIENTS; n++) {
                if (serverClients[n].connected()) {
                  serverClients[n].write(msg_Stat.c_str());
                }
              }
            }
            
            else {    // if not a status request send simple response
/*              
              // print to the console for debugging
              Serial.print("+");
              Serial.print(req);
              Serial.print("\r\nSW_Ready>>");
*/              
              // Assemblle the response string
              s = "+" ;
              l = req.length();
              req.toCharArray(r,l);
              s += r;
              s += ">>";
              
              // send to all connected clients
              for (n = 0; n < MAX_SRV_CLIENTS; n++) {
                if (serverClients[n].connected()) {
                  serverClients[n].write(s.c_str());  
                }
              }
            }
          }
        }
      }
    }
  }
  
  else if(udp_mode) {
    // Check if a client has connected
    WiFiClient client = server.available();
  
  // UDP code insert starts here
    
    // if there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
  
      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      Serial.println("Contents:");
      Serial.println(packetBuffer);
  
      // send a reply, to the IP address and port that sent us the packet we received
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer);
      Udp.endPacket();
    }  
  // UDP code insert ends here  

  // insert call to set_ant() here - with received data converted appropriately

  }

// Eventually use the following for a heartbeat signal??
/*
  //check UART for data
  if(Serial.available()){
    size_t len = Serial.available();
    uint8_t sbuf[len];
    Serial.readBytes(sbuf, len);
    //push UART data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
        serverClients[i].write(sbuf, len);
        delay(1);
      }
    }
  }
*/
}

// Set antenna based on command received
boolean set_ant() {
    
    // Match the request
    int val2;
    boolean i = true;
  
    if ((req.indexOf("AS") != -1) || (req.indexOf("BS") != -1)) {
      val2 = sw_Status();
      i = false;
    }
    else if ((req.indexOf("A0") != -1) || (req.indexOf("B0") != -1 )) {
      val2 = ant0_On();
    }
    else if ((req.indexOf("A1") != -1 ) || (req.indexOf("B1") != -1 )){
      val2 = ant1_On();
    }
    else if ((req.indexOf("A2") != -1 ) || (req.indexOf("B2") != -1 )){
      val2 = ant2_On();
    }  
    else if ((req.indexOf("A3") != -1 ) || (req.indexOf("B3") != -1 )){
      val2 = ant3_On();
    }
    else if ((req.indexOf("A4") != -1 ) || (req.indexOf("B4") != -1 )){
      val2 = ant4_On();
    }  
    else if ((req.indexOf("A5") != -1 ) || (req.indexOf("B5") != -1 )){
      val2 = ant5_On();
    }
    else if ((req.indexOf("A6") != -1 ) || (req.indexOf("B6") != -1 )){
      val2 = ant6_On();
    }  
    else if ((req.indexOf("A7") != -1 ) || (req.indexOf("B7") != -1 )){
      val2 = ant7_On();
    }
    else if ((req.indexOf("A8") != -1 ) || (req.indexOf("B8") != -1 )){
      val2 = ant8_On();
    }  
    else {
      // invalid request
      return true;
    }

    // calculate response time for everything but status
    if (i == true) {
      resp_time();  
    }
    return false;
}


// switch status function

boolean sw_Status() {
  // get status info to send as a reply
  printWifiData();
  return false;
}


// Antenna set functions
// 0 = all relays off - antennas grounded

int ant0_On() {
  Serial.println("\r\nAntenna A0 - All OFF... 00000000");
  digitalWrite(antS8, LOW);
  digitalWrite(antS7, LOW);
  digitalWrite(antS6, LOW);
  digitalWrite(antS5, LOW);
  digitalWrite(antS4, LOW);
  digitalWrite(antS3, LOW);
  digitalWrite(antS2, LOW);
  digitalWrite(antS1, LOW);
  return 0;
}


int ant1_On() {
  Serial.println("\r\nAntenna A1 - ON... 10000000");
  digitalWrite(antS8, LOW);
  digitalWrite(antS7, LOW);
  digitalWrite(antS6, LOW);
  digitalWrite(antS5, LOW);
  digitalWrite(antS4, LOW);
  digitalWrite(antS3, LOW);
  digitalWrite(antS2, LOW);
  digitalWrite(antS1, HIGH);
  return 1;
}

int ant2_On() {
  Serial.println("\r\nAntenna A2 - ON... 01000000");
  digitalWrite(antS8, LOW);
  digitalWrite(antS7, LOW);
  digitalWrite(antS6, LOW);
  digitalWrite(antS5, LOW);
  digitalWrite(antS4, LOW);
  digitalWrite(antS3, LOW);
  digitalWrite(antS2, HIGH);
  digitalWrite(antS1, LOW);  
  return 2;
}

int ant3_On() {
  Serial.println("\r\nAntenna A3 - ON... 00100000");
  digitalWrite(antS8, LOW);
  digitalWrite(antS7, LOW);
  digitalWrite(antS6, LOW);
  digitalWrite(antS5, LOW);
  digitalWrite(antS4, LOW);
  digitalWrite(antS3, HIGH);
  digitalWrite(antS2, LOW);
  digitalWrite(antS1, LOW);  
  return 3;
}

int ant4_On() {
  Serial.println("\r\nAntenna A4 - ON... 00010000");
  digitalWrite(antS8, LOW);
  digitalWrite(antS7, LOW);
  digitalWrite(antS6, LOW);
  digitalWrite(antS5, LOW);
  digitalWrite(antS4, HIGH);
  digitalWrite(antS3, LOW);
  digitalWrite(antS2, LOW);
  digitalWrite(antS1, LOW);  
  return 4;
}

int ant5_On() {
  Serial.println("\r\nAntenna A5 - ON... 00001000");
  digitalWrite(antS8, LOW);
  digitalWrite(antS7, LOW);
  digitalWrite(antS6, LOW);
  digitalWrite(antS5, HIGH);
  digitalWrite(antS4, LOW);
  digitalWrite(antS3, LOW);
  digitalWrite(antS2, LOW);
  digitalWrite(antS1, LOW);  
  return 5;
}

int ant6_On() {
  Serial.println("\r\nAntenna A6 - ON... 00000100");
  digitalWrite(antS8, LOW);
  digitalWrite(antS7, LOW);
  digitalWrite(antS6, HIGH);
  digitalWrite(antS5, LOW);
  digitalWrite(antS4, LOW);
  digitalWrite(antS3, LOW);
  digitalWrite(antS2, LOW);
  digitalWrite(antS1, LOW);
  return 6;
}

int ant7_On() {
  Serial.println("\r\nAntenna A7 - ON... 00000010");
  digitalWrite(antS8, LOW);
  digitalWrite(antS7, HIGH);
  digitalWrite(antS6, LOW);
  digitalWrite(antS5, LOW);
  digitalWrite(antS4, LOW);
  digitalWrite(antS3, LOW);
  digitalWrite(antS2, LOW);
  digitalWrite(antS1, LOW);
  return 7;
}

int ant8_On() {
  Serial.println("\r\nAntenna A8 - ON... 00000001");
  digitalWrite(antS8, HIGH);
  digitalWrite(antS7, LOW);
  digitalWrite(antS6, LOW);
  digitalWrite(antS5, LOW);
  digitalWrite(antS4, LOW);
  digitalWrite(antS3, LOW);
  digitalWrite(antS2, LOW);
  digitalWrite(antS1, LOW);
  return 8;
}


// Function to connect to wifi – returns true if successful or false if not
// Prints WiFi status when connected
boolean connectWifi(){
  boolean state = true;
  int i = 0;
  
  // Pick best WiFi to connect too
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP(ssid1, password1);

  /* original WiFi connection statement - replaced by the above multi AP commands
   *  WiFi.begin(ssid, password);
  */
  
  Serial.println("");
  Serial.println("Connecting to WiFi ");
 
  // Wait for connection
  
  /* original single WiFi AP connection while loop
   * while (WiFi.status() != WL_CONNECTED) {
  */
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // if not connected in .5*i seconds break out of while loop
    if (i > 20){
      state = false;
      break;
    }
    i++;
  }
  
  if (state){
    // when connected - print WiFi connection info 
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);

    printWifiData();
    Serial.println("");
  }
}

void printWifiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  
/*  Serial.print("\r\nIP Address: ");
  Serial.println(ip);
*/

  // initialize message status string to be returned on status command
  msg_Status.begin();

  // Append the SSID to the status string
  msg_Status.print("SSID: ");
  msg_Status.println(WiFi.SSID());

  // Append IP info to status string
  msg_Status.print("IP Address: ");
  msg_Status.print(ip);
  msg_Status.print("\r\n");
 
  // print your MAC address:
  byte mac[6];  
  WiFi.macAddress(mac);

/*  Serial.print("Module MAC address: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
*/

  // Append MAC address to the status string
  msg_Status.print("Module MAC address: ");
  msg_Status.print(mac[5],HEX);
  msg_Status.print(":");
  msg_Status.print(mac[4],HEX);
  msg_Status.print(":");
  msg_Status.print(mac[3],HEX);
  msg_Status.print(":");
  msg_Status.print(mac[2],HEX);
  msg_Status.print(":");
  msg_Status.print(mac[1],HEX);
  msg_Status.print(":");
  msg_Status.println(mac[0],HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();

/*  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.print("Module Version: ");
  Serial.println(ver_num);
  Serial.println("============================");
*/

  // Append signal strength and firmware version number to status string
  msg_Status.print("Signal strength (RSSI): ");
  msg_Status.print(rssi);
  msg_Status.println(" dBm");
  msg_Status.print("Module Version: ");
  msg_Status.println(ver_num);

  // Append response time calculations
  msg_Status.print("Requests: ");
  msg_Status.print(num_req);
  msg_Status.print("  Last: ");
  rt_last = micros() - t0;
  msg_Status.print(rt_last);
  msg_Status.println("uSec");
  msg_Status.print("Avg resp time: ");
  msg_Status.print(rt_avg);
  msg_Status.print("uSec");
  msg_Status.print("  Min: ");
  msg_Status.print(rt_min);
  msg_Status.print("  Max: ");
  msg_Status.println(rt_max);

  // get pin state 
  byte pin_state = 0;
  unsigned int pin_val[8];
  
  // append pin state to status message string
  msg_Status.print("Antenna pin state: ");
//  Serial.print("Antenna pin state: ");
  for (int i = 0; i < 8; i++) {
    pin_val[i] = 0;
    pin_val[i] = digitalRead(antpin[i]);
//    Serial.print(pin_val[i]);
    msg_Status.print(pin_val[i]);
  }
  // add a cr/newline at the end of the pin state status line
  msg_Status.print("\r\n");

  // Append a separator line
  msg_Status.println("###########################");  

  // Print assembled WiFi status message on serial console for debugging
  Serial.println("");
  Serial.print(msg_Status);
}

void getCurrentPins() {
  // get pin state 
  byte pin_state = 0;
  unsigned int pin_val[8];
  msg_Status.begin(); // re-initialize
  msg_Status.print("Antenna pin state: ");
//  Serial.print("Antenna pin state: ");

  // append pin state to status message string
  for (int i = 0; i < 8; i++) {
    pin_val[i] = 0;
    pin_val[i] = digitalRead(antpin[i]);
//    Serial.print(pin_val[i]);
    msg_Status.print(pin_val[i]);
  }

  // append a cr/newline to the end of the message
  msg_Status.print("\r\n");
  
  // print assembled status message to console for debugging
  Serial.println(msg_Status);
}

/* ****************************************************************************
 *  function to calculate average command response time over last 1000 requests
 * ****************************************************************************
*/
void resp_time() {
  if (num_req > 1000) {  // if requests is greate than 1000 reset resp_time calc
    rt_total = 0; 
    num_req = 1;
    rt_max = 0;
    rt_min = 0;
  }
  // validate switch response time for debugging
  t5 = micros() - t0;
  rt_total += t5;  // accumulate the response time
  num_req++;       // increment the number of requests
  rt_avg = rt_total/num_req;
  if(num_req == 1) rt_min = t5;
  if(t5 > rt_max) rt_max = t5;
  else if (t5 < rt_min || rt_min == 0) rt_min = t5;
}

