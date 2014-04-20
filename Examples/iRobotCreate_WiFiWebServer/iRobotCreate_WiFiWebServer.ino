/*
  WiFi Web Server
 
 A simple web server that shows the value of the analog input pins.
 using a WiFi shield.
 
 This example is written for a network using WPA encryption. For 
 WEP or WPA, change the Wifi.begin() call accordingly.
 
 Circuit:
 * WiFi shield attached
 * Analog inputs attached to pins A0 through A5 (optional)
 
 created 13 July 2010
 by dlf (Metodo2 srl)
 modified 31 May 2012
 by Tom Igoe

 */

#include <SPI.h>
#include <WiFi.h>
#include <Roomba.h>

// Set up iRobot Create:

// Defines the Roomba instance and the HardwareSerial it connected to
Roomba roomba(&Serial1);

int x;                   // 16 bit signed variable
unsigned int u;          // 16 bit unsigned variable

// Setup Wi-Fi:

char ssid[] = "Hackerspace";      // your network SSID (name) 
char pass[] = "MakingIsFun!";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(80);

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  /** 
   * Setup Wi-FI
   */
  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    // don't continue:
    while(true);
  } 
  
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:    
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  } 
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
  
  /** 
   * Setup iRobot Create
   */
  
  setupRoomba();
}

boolean setupRoomba() {
  roomba.start();
//  roomba.safeMode();
  roomba.fullMode();
  roomba.drive(0, 0); // Stop
}

uint8_t buf[52];                                        // Packet 6 returns 52 bytes

void loop() {
  
  // Get sensor data (from iRobot Create)
  boolean hasRoombaSensorData = roomba.getSensors(6 , buf , 52);             // packetID, destination, length (bytes) 
                                                          // Note that getSensors() -returns true when [length] bytes have been read 
                                                          //                        -is blocking until [length] bytes have been read or timeout occurs
                                                          // Consult Open Interface Manual pg 17 for packet lengths





//  // Serial.println("driveDirect");
//  delay(1000);
//  roomba.driveDirect(300 , 300);   // Often the first call after restart is not registered. This is an ongoing problem with no known fix.
//  roomba.driveDirect(300 , 300);   // Left/Right Wheel velocity (mm/s)
//  delay(1000);
//  roomba.driveDirect(-300 , 300);  // Left/Right Wheel velocity (mm/s)
//  delay(1000);
//  roomba.driveDirect(300, -300);   // Left/Right Wheel velocity (mm/s)
//  delay(1000);  
//  roomba.driveDirect(0, 0);        // Left/Right Wheel velocity (mm/s)
//  delay(2000);   
//
//  // Serial.println("drive");  
//  roomba.drive(300, 300);                   // Velocity (mm/s), Radius (mm)
//  delay(1000);
//  roomba.drive(300, -300);
//  delay(1000);
//  roomba.drive(300, roomba.DriveStraight);                  //DriveStraight is a special case. See Public Types in Roomba Class Reference
//  delay(1000);
//  roomba.drive(300, roomba.DriveInPlaceClockwise);
//  delay(1000);
//  roomba.drive(300, roomba.DriveInPlaceCounterClockwise);
//  delay(1000);  
//  roomba.drive(0, 0);
//  delay(1000);
  
  
  
  
  
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client");
    char httpRequestBuffer[512];
    int httpRequestLineCount = 0;
    boolean hasReceivedRequestFirstLine = false;
    boolean hasReceivedRequest = false;
    int bi = 0;
//    boolean hasHttpRequest = false;
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    
    // Parse HTTP request header
    char* spaceChar = NULL;
    int spaceCharIndex = 0;
    int httpRequestMethodIndex = 0;
    char* httpRequestMethod = &httpRequestBuffer[0]; // Poit the string to the beginning of the HTTP request
    char* httpRequestAddress = NULL;
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // Serial.write(c);
        
        // Buffer the first line of the received HTTP request
        if (httpRequestLineCount == 0) {
          httpRequestBuffer[bi] = c;
          bi++;
        }
        
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          
          Serial.println("Status: Received complete HTTP request.");
          hasReceivedRequest = true;
          
        }
        
        if (hasReceivedRequest) {
          if (strcmp (httpRequestMethod, "POST") == 0) {
            
            if (strcmp (httpRequestAddress, "/rotate") == 0) {
              
              Serial.println("Action: Rotate");
              
              // rotate
              
              roomba.driveDirect(-300 , 300);  // Left/Right Wheel velocity (mm/s)
              delay(500);
            
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              
              // TODO:
              
              break;
              
            } else if (strcmp (httpRequestAddress, "/dock") == 0) {
              
              roomba.dock(); // "Causes roomba to immediately seek the docking station. No equivalent for Create."
              
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              
              break;
              
            } else if (strcmp (httpRequestAddress, "/directDrive") == 0) {
              
              // TODO: Parse parameters from ?leftWheelVelocity=300&righWheelVelocity=-300&duration=1000&canInterrupt=true
              
              roomba.driveDirect(300, 300);   // Left/Right Wheel velocity (mm/s)
              delay(500);
              
              //roomba.driveDirect(leftWheelVelocity, righWheelVelocity);   // Left/Right Wheel velocity (mm/s)
              //delay(duration);
              
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              
              break;
              
            } else if (strcmp (httpRequestAddress, "/playSong") == 0) {
              
              Serial.println("Action: playSong");
              
              // TODO: Parse parameters from ?songNumber=0&canInterrupt=true
              
              int songNumber = 0;
              roomba.playSong(songNumber);
              
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              
              break;
              
            } else {
              
              // TODO: Default, catch-all GET handler
              
              // send a standard http response header
              client.println("HTTP/1.1 404 Not Found");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              
              break;
              
            } 
            
          } else if (strcmp (httpRequestMethod, "GET") == 0) {
            
            if (strcmp (httpRequestAddress, "/") == 0) {
            
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              client.println("<!DOCTYPE HTML>");
              client.println("<html>");
              // output the value of each analog input pin
              for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
                int sensorReading = analogRead(analogChannel);
                client.print("analog input ");
                client.print(analogChannel);
                client.print(" is ");
                client.print(sensorReading);
                client.println("<br />");       
              }
    
              // Print the state of the iRobot Create sensors
              if (hasRoombaSensorData) {
                client.println("<h1>iRobot Create Web Server</h1>");
                client.println("<h2>Packet 6 (Full) Sensor Data</h2>");
                  
                client.print("<h3>Bumps and Wheel Drops: ");
                client.println(buf[0], BIN);
                client.print("</h3>");
                client.print("Wheeldrop Caster: ");
                client.println(bitRead(buf[0], 4));
                client.print("<br />");
                client.print("Wheeldrop Left: ");
                client.println(bitRead(buf[0], 3));   
                client.print("<br />");
                client.print("Wheeldrop Right: ");
                client.println(bitRead(buf[0], 2));  
                client.print("<br />");   
                client.print("Bump Left: ");
                client.println(bitRead(buf[0], 1));
                client.print("<br />");
                client.print("Bump Right: ");
                client.println(bitRead(buf[0], 0));
                client.print("<br />");
                 
                client.print("<h3>Wall Sensor:");
                client.println(buf[1], BIN);
                client.print("</h3>");
            
                client.print("Cliff Left: ");
                client.println(buf[2], BIN);  
                client.print("<br />");
                client.print("Cliff Front Left: ");  
                client.println(buf[3], BIN);   
                client.print("<br />");
                client.print("Cliff Front Right: ");  
                client.println(buf[4], BIN);  
                client.print("<br />");
                client.print("Cliff Right: ");    
                client.println(buf[5], BIN);
                client.print("<br />");
            
                client.print("<h3>Virtual Wall: ");
                client.println(buf[6], BIN);
                client.print("</h3>");
            
                client.print("<h3>Low Side Drivers and Wheel Overcurrents: ");
                client.println(buf[7], BIN);
                client.print("</h3>");
            
                // Note the 2 unused bytes pg 18 of Open Interface Manual
                client.print("<h3>Infrared Byte: ");
                client.println(buf[10], BIN);
                client.print("</h3>");
            
                client.print("<h3>Buttons: ");
                client.println(buf[11], BIN);
                client.print("</h3>");
                client.print("Play Button: ");
                client.println(bitRead(buf[11], 0));
                client.print("<br />");
                client.print("Advance Button: ");
                client.println(bitRead(buf[11], 2));
                client.print("<br />");
                client.print("<br />");
                 
                client.print("Distance (mm): ");              // Sum of distance traveled by both wheels divided by two
                x = BitShiftCombine(buf[12], buf[13]);        // Value sent as 16 bit signed value high byte first.
                client.println(x);                            // Note that if note called frequently, value is capped at min or max
                client.print("<br />");
                client.print("<br />");
            
                client.print("Angle (Degrees- CCW+): ");          
                x = BitShiftCombine(buf[14], buf[15]);        // Value sent as 16 bit signed value high byte first.
                client.println(x);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Charging State: ");
                u = buf[16];
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Battery Voltage (mV): ");
                u = BitShiftCombine(buf[17], buf[18]);      // Value sent as 16 bit unsigned value high byte first.
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Battery Current (mA): ");
                x = BitShiftCombine(buf[19], buf[20]);      // Value sent as 16 bit signed value high byte first.
                client.println(x);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Battery Temperature (C): ");
                x = buf[21];
                client.println(x);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Current Battery Charge (mAh): ");
                u = BitShiftCombine(buf[22], buf[23]);      // Value sent as 16 bit unsigned value high byte first.
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Battery Capacity (mAh): ");
                u = BitShiftCombine(buf[24], buf[25]);      // Value sent as 16 bit signed value high byte first.
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Wall Signal Strength (0-4095): ");
                u = BitShiftCombine(buf[26], buf[27]);      // Value sent as 16 bit unsigned value high byte first.
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Cliff Left Signal Strength (0-4095): ");
                u = BitShiftCombine(buf[28], buf[29]);      // Value sent as 16 bit unsigned value high byte first.
                client.println(u); 
                client.print("<br />");
                client.print("<br />");
            
                client.print("Cliff Front Left Signal Strength (0-4095): ");
                u = BitShiftCombine(buf[30], buf[31]);      // Value sent as 16 bit unsigned value high byte first.
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Cliff Front Right Signal Strength (0-4095): ");
                u = BitShiftCombine(buf[32], buf[33]);      // Value sent as 16 bit unsigned value high byte first.
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Cliff Right Signal Strength (0-4095): ");
                u = BitShiftCombine(buf[34], buf[35]);      // Value sent as 16 bit unsigned value high byte first.
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Cargo Bay Digital Inputs: ");
                client.println(buf[36], BIN);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Cargo Bay Analog Signal (0-1023):");
                u = BitShiftCombine(buf[37], buf[38]);       // Value sent as 16 bit unsigned value high byte first.
                client.println(u);
                client.print("<br />");
                client.print("<br />");
      
                client.print("Charging Sources Available:");
                client.println(buf[39], BIN);
                client.print("<br />");
                client.print("<br />");
            
                client.print("OI Mode: ");
                u = buf[40];
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Song Number: ");
                x = buf[41];
                client.println(x);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Song Playing: ");
                client.println(buf[42], BIN);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Number of Stream Packets: ");
                u = buf[43];
                client.println(u);
                client.print("<br />");
                client.print("<br />");
            
                client.print("Requested Velocity (mm/s): ");
                x = BitShiftCombine(buf[44], buf[45]);      // Value sent as 16 bit signed value high byte first.
                client.println(x);  
                client.print("<br />");   
                client.print("<br />");     
            
            
                client.print("Requested Radius (mm):");
                x = BitShiftCombine(buf[46], buf[47]);      // Value sent as 16 bit signed value high byte first.
                client.println(x);     
                client.print("<br />");  
                client.print("<br />");   
            
            
                client.print("Requested Right Velocity (mm/s): ");
                x = BitShiftCombine(buf[48], buf[49]);      // Value sent as 16 bit signed value high byte first.
                client.println(x);       
                client.print("<br />");  
                client.print("<br />"); 
            
            
                client.print("Requested Left Velocity (mm/s): ");
                x = BitShiftCombine(buf[50], buf[51]);      // Value sent as 16 bit signed value high byte first.
                client.println(x);       
                client.print("<br />");  
                client.print("<br />"); 
              }
            
              client.println("</html>");
              break;
              
            } else {
              
              // TODO: Default, catch-all GET handler
              
              // send a standard http response header
              client.println("HTTP/1.1 404 Not Found");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              
              break;
              
            }
          }
        }
        
        
        if (c == '\n') {
          
          // Increase the HTTP request line count
          httpRequestLineCount++;
          
          // Parse HTTP header
          if (httpRequestLineCount == 1) {
            httpRequestBuffer[bi] = NULL; // Terminate string
            
            Serial.print("REQUEST: ");
            Serial.println(httpRequestBuffer);
            Serial.println();
            
            // Parse HTTP request header
//            char* spaceChar = NULL;
//            int spaceCharIndex = 0;
//            int httpRequestMethodIndex = 0;
//            char* httpRequestMethod = &httpRequestBuffer[0]; // Poit the string to the beginning of the HTTP request
//            char* httpRequestAddress = NULL;
            
            // Get HTTP request method (i.e., GET or POST)
            httpRequestMethod = &httpRequestBuffer[0];
            spaceChar = strchr(httpRequestMethod, ' ');
            spaceCharIndex = (int) (spaceChar - httpRequestBuffer);
            httpRequestBuffer[spaceCharIndex] = NULL; // Terminate the request method string
            Serial.println(httpRequestMethod);
            
            // Get HTTP request address
            httpRequestAddress = &httpRequestBuffer[spaceCharIndex + 1];
            spaceChar = strchr(httpRequestAddress, ' ');
            spaceCharIndex = (int) (spaceChar - httpRequestBuffer);
            httpRequestBuffer[spaceCharIndex] = NULL; // Terminate the request method string
            Serial.println(httpRequestAddress);
            
            hasReceivedRequestFirstLine = true;
            
            // TODO: Get parameters (after ?, between &)
          }
          
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
        
        
        
//        if (hasReceivedRequestFirstLine) {
//          hasReceivedRequest = true;
//        }
        
        
        
        
      }
    }
    // give the web browser time to receive the data
    delay(1);
    
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
  
  
  
//  if (hasRoombaSensorData)
//  {
//    Serial.println("\n\n\n\n\n\n");
//    Serial.println("iRobot Create Packet 6 (Full) Sensor Data \n");
//    
//
//    Serial.print("Bumps and Wheel Drops: ");
//    Serial.println(buf[0], BIN);
//     Serial.print("* Wheeldrop- Caster: ");
//     Serial.println(bitRead(buf[0], 4));
//     Serial.print("* Wheeldrop- Left: ");
//     Serial.println(bitRead(buf[0], 3));   
//     Serial.print("* Wheeldrop- Right: ");
//     Serial.println(bitRead(buf[0], 2));     
//     Serial.print("* Bump Left: ");
//     Serial.println(bitRead(buf[0], 1));
//     Serial.print("* Bump Right: ");
//     Serial.println(bitRead(buf[0], 0));
//     
//    Serial.print("Wall Sensor: ");
//    Serial.println(buf[1], BIN);
//
//    Serial.print("Cliff Left: ");
//    Serial.println(buf[2], BIN);  
//    Serial.print("Cliff Front Left: ");  
//    Serial.println(buf[3], BIN);   
//    Serial.print("Cliff Front Right: ");  
//    Serial.println(buf[4], BIN);  
//    Serial.print("Cliff Right: ");    
//    Serial.println(buf[5], BIN);
//
//    Serial.print("Virtual Wall: ");
//    Serial.println(buf[6], BIN);
//
//    Serial.print("Low Side Drivers and Wheel Overcurrents: ");
//    Serial.println(buf[7], BIN);
//
//    //Note the 2 unused bytes pg 18 of Open Interface Manual
//    Serial.print("Infrared Byte: ");
//    Serial.println(buf[10], BIN);
//
//    Serial.print("Buttons: ");
//    Serial.println(buf[11], BIN);
//     Serial.print("* Play Button: ");
//     Serial.println(bitRead(buf[11], 0));
//     Serial.print("* Advance Button: ");
//     Serial.println(bitRead(buf[11], 2));
//     
//    Serial.print("Distance (mm): ");              // Sum of distance traveled by both wheels divided by two
//    x = BitShiftCombine(buf[12], buf[13]);        // Value sent as 16 bit signed value high byte first.
//    Serial.println(x);                            // Note that if note called frequently, value is capped at min or max
//
//    Serial.print("Angle (Degrees- CCW+): ");          
//    x = BitShiftCombine(buf[14], buf[15]);        // Value sent as 16 bit signed value high byte first.
//    Serial.println(x);                            // Note that if note called frequently, value is capped at min or max 
//
//    Serial.print("Charging State: ");
//    u = buf[16];
//    Serial.println(u);
//
//    Serial.print("Battery Voltage (mV): ");
//    u = BitShiftCombine(buf[17], buf[18]);      // Value sent as 16 bit unsigned value high byte first.
//    Serial.println(u);      
//
//    Serial.print("Battery Current (mA): ");
//    x = BitShiftCombine(buf[19], buf[20]);      // Value sent as 16 bit signed value high byte first.
//    Serial.println(x);       
//
//    Serial.print("Battery Temperature (C): ");
//    x = buf[21];
//    Serial.println(x);
//
//    Serial.print("Current Battery Charge (mAh): ");
//    u = BitShiftCombine(buf[22], buf[23]);      // Value sent as 16 bit unsigned value high byte first.
//    Serial.println(u);
//
//    Serial.print("Battery Capacity (mAh): ");
//    u = BitShiftCombine(buf[24], buf[25]);      // Value sent as 16 bit signed value high byte first.
//    Serial.println(u);  
//
//    Serial.print("Wall Signal Strength (0-4095): ");
//    u = BitShiftCombine(buf[26], buf[27]);      // Value sent as 16 bit unsigned value high byte first.
//    Serial.println(u);    
//
//    Serial.print("Cliff Left Signal Strength (0-4095): ");
//    u = BitShiftCombine(buf[28], buf[29]);      // Value sent as 16 bit unsigned value high byte first.
//    Serial.println(u);  
//
//    Serial.print("Cliff Front Left Signal Strength (0-4095): ");
//    u = BitShiftCombine(buf[30], buf[31]);      // Value sent as 16 bit unsigned value high byte first.
//    Serial.println(u);  
//
//    Serial.print("Cliff Front Right Signal Strength (0-4095): ");
//    u = BitShiftCombine(buf[32], buf[33]);      // Value sent as 16 bit unsigned value high byte first.
//    Serial.println(u);  
//
//    Serial.print("Cliff Right Signal Strength (0-4095): ");
//    u = BitShiftCombine(buf[34], buf[35]);      // Value sent as 16 bit unsigned value high byte first.
//    Serial.println(u);  
//
//    Serial.print("Cargo Bay Digital Inputs: ");
//    Serial.println(buf[36], BIN);
//
//    Serial.print("Cargo Bay Analog Signal (0-1023):");
//    u = BitShiftCombine(buf[37], buf[38]);       // Value sent as 16 bit unsigned value high byte first.
//    Serial.println(u);  
//
//    Serial.print("Charging Sources Available:");
//    Serial.println(buf[39], BIN);
//
//    Serial.print("OI Mode: ");
//    u = buf[40];
//    Serial.println(u);
//
//    Serial.print("Song Number: ");
//    x = buf[41];
//    Serial.println(x);
//
//    Serial.print("Song Playing: ");
//    Serial.println(buf[42], BIN);
//
//    Serial.print("Number of Stream Packets: ");
//    u = buf[43];
//    Serial.println(u); 
//
//    Serial.print("Requested Velocity (mm/s): ");
//    x = BitShiftCombine(buf[44], buf[45]);      // Value sent as 16 bit signed value high byte first.
//    Serial.println(x);       
//
//
//    Serial.print("Requested Radius (mm):");
//    x = BitShiftCombine(buf[46], buf[47]);      // Value sent as 16 bit signed value high byte first.
//    Serial.println(x);       
//
//
//    Serial.print("Requested Right Velocity (mm/s): ");
//    x = BitShiftCombine(buf[48], buf[49]);      // Value sent as 16 bit signed value high byte first.
//    Serial.println(x);       
//
//
//    Serial.print("Requested Left Velocity (mm/s): ");
//    x = BitShiftCombine(buf[50], buf[51]);      // Value sent as 16 bit signed value high byte first.
//    Serial.println(x);       
//   
//
////    delay(5000);                      // Update every 5 seconds
//    //while(1);                         // Run only once
// 
//  }  // if(ret)
//  else Serial.println("GetSensors() Error");
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


int BitShiftCombine( unsigned char x_high, unsigned char x_low)
{


  int combined;  
  combined = x_high;              //send x_high to rightmost 8 bits
  combined = combined<<8;         //shift x_high over to leftmost 8 bits
  combined |= x_low;                 //logical OR keeps x_high intact in combined and fills in rightmost 8 bits 
  return combined;

}

