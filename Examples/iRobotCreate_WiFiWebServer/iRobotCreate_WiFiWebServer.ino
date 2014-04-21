/*
"Telepresence Robot", built at Collider, starting 4/20 of 2014.

TODO: Write a brief description. Describe hardware and software prerequisites.

Hardware/Circuit Notes:
- Connected iRobot pin 1 (RXD) to pin 18 (TX) on an Arduino Mega 2560,
                   pin 2 (TXD) to pin 12 (RX),
                   pin 14 (GND) to GND on the Arduino Mega 2560,
                   pin 3 (Power control toggle) to pin 8,
               and pin 13 (isCharging) to pin 9 (set the pin mode of pin 9 to "INPUT" on Arduino).

Authors:
- Michael Gubbels (Creator)

Attribution:
This project was built using the Roomba library (http://www.airspayce.com/mikem/arduino/Roomba/index.html).
Some of this code is based on RCKit (http://www.airspayce.com/mikem/arduino/RCKit/).
*/

#include <SPI.h>
#include <WiFi.h>
#include <Roomba.h>

// Set up iRobot Create:

// Defines the Roomba instance and the HardwareSerial it connected to
Roomba roomba(&Serial1);

int x;                   // 16 bit signed variable
unsigned int u;          // 16 bit unsigned variable

const int iRobotPowerControlTogglePin = 8;
const int iRobotIsChargingPin = 9;

// TODO: Set up "time since last charge" variable (so can return to base station in time for a recharge)
// TODO: Set up "time since last powered"
// TODO: Set up "time since last controlled remotely"
// TODO: Set up a "amount of time taken to charge in the past" so can estimate how long charging will take, and use for planning

// Setup Wi-Fi:

char ssid[] = "Hackerspace";      // your network SSID (name) 
char pass[] = "MakingIsFun!";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(80);

void setup() {
  
  /** Setup serial */
  setupSerial();
  
  Serial.println("Hey there. I'm waking up. Give me a minute.\n");
  
  /** Setup Wi-Fi */
  setupWiFi();
  
  /** Setup iRobot Create */
  setupRoomba();
}

boolean setupSerial() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  // Return success (true) or failure (false)
  return true;
}

boolean setupWiFi() {
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    // don't continue:
    while(true);
  } 
  
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("I'm attempting to connect to a Wi-Fi network with SSID \"");
    Serial.print(ssid);
    Serial.print("\".\n");
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:    
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  } 
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
  
  // Return success (true) or failure (false)
  return true;
}

boolean setupRoomba() {
  roomba.start();
//  roomba.safeMode();
  roomba.fullMode(); //  ModeOff = 0, ModePassive = 1, ModeSafe = 2, ModeFull = 3 }
  roomba.drive(0, 0); // Stop
  
  // Return success (true) or failure (false)
  return true;
}

boolean hasRoombaSensorData = false;
uint8_t buf[52];                                        // iRobot sensor data buffer. Packet 6 returns 52 bytes.
char httpRequestUriBuffer[512]; // HTTP request buffer
int bi = 0; // HTTP request buffer index
char* httpRequestParameters[10]; // HTTP request parameters (i.e., key/value pairs encoded in the URI like "?key1=value1&key2=value2")
int httpRequestParameterCount = 0;
char* httpRequestParameterDictionary[10][2]; // HTTP request parameters (i.e., key/value pairs encoded in the URI like "?key1=value1&key2=value2")
// httpRequestParameterDictionary[paramIndex][key] // key = 0 (returns char*)
// httpRequestParameterDictionary[paramIndex][value] // value = 1 (returns char*)

void loop() {
  
  // Get sensor data (from iRobot Create)
  hasRoombaSensorData = roomba.getSensors(6 , buf , 52);  // packetID, destination, length (bytes) 
                                                          // Note that getSensors() -returns true when [length] bytes have been read 
                                                          //                        -is blocking until [length] bytes have been read or timeout occurs
                                                          // Consult Open Interface Manual pg 17 for packet lengths

  /**
   * Web server: Process HTTP requests
   */  
  
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client connected");
    
    bi = 0; // Reset the HTTP request buffer
    int httpRequestLineCount = 0;
    boolean hasReceivedRequestFirstLine = false;
    boolean hasReceivedRequest = false;
//    boolean hasHttpRequest = false;

    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    
    // Parse HTTP request header
    char* spaceChar = NULL;
    int spaceCharIndex = 0;
    int httpRequestMethodIndex = 0;
    char* httpRequestMethod = &httpRequestUriBuffer[0]; // Poit the string to the beginning of the HTTP request
    char* httpRequestAddress = NULL;
    char* httpRequestParameterString = NULL;
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // Serial.write(c);
        
        // Buffer the first line of the received HTTP request
        if (httpRequestLineCount == 0) {
          httpRequestUriBuffer[bi] = c;
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
            
            if (strcmp (httpRequestAddress, "/rotate") == 0) { // TODO: Remove this?
              
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
              
            } else if (strcmp (httpRequestAddress, "/utility/togglePower") == 0) {
              
              // Momentarily writes high to the iRobot's "Power Control Toggle" pin. This 
              // pin "Turns iRobot Create on or off on a low-to-high transition".
              digitalWrite(iRobotPowerControlTogglePin, LOW);
              digitalWrite(iRobotPowerControlTogglePin, HIGH);
              delay(50); // TODO: Determine if this needed!
              digitalWrite(iRobotPowerControlTogglePin, LOW);
              
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              break;
              
            } else if (strcmp (httpRequestAddress, "/utility/isCharging") == 0) {
              
              // Momentarily writes high to the iRobot's "Power Control Toggle" pin. This 
              // pin "Turns iRobot Create on or off on a low-to-high transition".
              int robotChargingPinState = digitalRead(iRobotIsChargingPin);
              
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              // Return state in JSON format
              client.print("{ \"robotChargingPinState\": ");
              client.print(robotChargingPinState);
              client.print(" }");
              break;
              
            } else if (strcmp (httpRequestAddress, "/reset") == 0) {
              
              roomba.reset(); // "Resets the Roomba. It will emit its startup message Caution, this may take several seconds to complete."
              
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              break;
              
            } else if (strcmp (httpRequestAddress, "/start") == 0) {
              
              roomba.start(); // "Starts the Open Interface and sets the mode to Passive. You must send this before sending any other commands. Initialises the serial port to the baud rate given in the constructor."
              
              // send a standard http response header
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");  // the connection will be closed after completion of the response
              // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
              client.println();
              break;
              
            } else if (strcmp (httpRequestAddress, "/drive") == 0) {
              
              // TODO: Parse parameters from ?velocity=300&radius=DriveStraight&duration=500&canInterrupt=true
              
              roomba.drive(300, roomba.DriveStraight); // DriveStraight is a special case. See Public Types in Roomba Class Reference
              // "radius" enum values defined in Roomba library:
              // roomba.DriveStraight                = 0x8000
              // roomba.DriveInPlaceClockwise        = 0xFFFF
              // roomba.DriveInPlaceCounterClockwise = 0x0001
              delay(500);
              roomba.drive(0, 0);
              
              // roomba.drive(velocity, radius); // DriveStraight is a special case. See Public Types in Roomba Class Reference
              // delay(duration);
              
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
              delay(500); // TODO: Make this "non-blocking" with a timer/operation status data structure
              roomba.drive(0, 0);
              
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
            
              handleDefaultHttpRequest(client);
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
            
          } else { // Unrecognized HTTP request method (possibly an error, such as a malformed request)
            
            // Send a standard http response header
            client.println("HTTP/1.1 404 Not Found");
            client.println("Content-Type: text/html");
            client.println("Connection: close");  // the connection will be closed after completion of the response
            // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
            client.println();
            break;
          }
        }
        
        
        if (c == '\n') {
          
          // Increase the HTTP request line count
          httpRequestLineCount++;
          
          // Parse HTTP header
          if (httpRequestLineCount == 1) {
            httpRequestUriBuffer[bi] = NULL; // Terminate string
            
            Serial.print("REQUEST: ");
            Serial.println(httpRequestUriBuffer);
            Serial.println();
            
            /**
             * Parse HTTP request header
             */
            
            // Get HTTP request method (i.e., GET or POST)
            httpRequestMethod = &httpRequestUriBuffer[0];
            spaceChar = strchr(httpRequestMethod, ' ');
            spaceCharIndex = (int) (spaceChar - httpRequestUriBuffer);
            httpRequestUriBuffer[spaceCharIndex] = NULL; // Terminate the request method string
            Serial.println(httpRequestMethod);
            
            // Get HTTP request address
            httpRequestAddress = &httpRequestUriBuffer[spaceCharIndex + 1];
            spaceChar = strchr(httpRequestAddress, ' ');
            spaceCharIndex = (int) (spaceChar - httpRequestUriBuffer);
            httpRequestUriBuffer[spaceCharIndex] = NULL; // Terminate the request method string
            Serial.println(httpRequestAddress);
            
            hasReceivedRequestFirstLine = true; // Flag the first line of the HTTP request as received. This line contains the URI, used to decide how to handle the request.

            /**            
             * Extract and decode URI parameters (i.e., after ?, between &)
             */
            
            char* questionChar = NULL; // Pointer to '?' character in URI
            int questionCharIndex = 0;
            
            // httpRequestParameterString = &httpRequestUriBuffer[spaceCharIndex + 1];
            httpRequestParameterString = httpRequestAddress; // Start searching for '?' at the beginning of the URI.
            questionChar = strchr(httpRequestParameterString, '?'); // Search for the '?' character
            httpRequestParameterCount = 0; // Reset parameter count
            
            Serial.print("Looking for '?'... ");
            if (questionChar != NULL) {
            // if (httpRequestParameterString != NULL) {
              Serial.print("Found!\n");
              
              // Find the '?' character, denoting the beginning of the parameter list
              questionCharIndex = (int) (questionChar - httpRequestUriBuffer); // Get index of '?' character in HTTP request string (of form "POST /drive?velocity=300&radius=15")
              httpRequestParameterString = &httpRequestUriBuffer[questionCharIndex + 1]; // Search for the beginning of the paramter list encoded in the HTTP request
              
              char* ampersandChar = NULL;
              int ampersandCharIndex = -1; // This is used to track the indices of '&' characters during parameter parsing
              ampersandCharIndex = questionCharIndex; // Initialize to position of the last found space character
              
              // Iterate over parameter list and extract parameters
              boolean haveExtractedParameters = false; // Flag indicating whether all parameters have been extracted
              while (!haveExtractedParameters) {
              
                // Search for the next parameter enoded in the URI (if any), by searching for the next occurrence of '&' or ' ' (denoting the end of the parameter list)
                httpRequestParameters[httpRequestParameterCount] = &httpRequestUriBuffer[ampersandCharIndex + 1];
                ampersandChar = strchr(httpRequestParameters[httpRequestParameterCount], '&');
                
                if (ampersandChar != NULL) { // Check if a '&' character was found
                  ampersandCharIndex = (int) (ampersandChar - httpRequestUriBuffer);
                  httpRequestUriBuffer[ampersandCharIndex] = NULL; // Terminate the parameter key/value pair string
                  
                  httpRequestParameterCount++; // Count the parameter and continue to next parameter (if any)
                  
                } else {
                  
                  // If no '&' character found, then find the end of the parameter list
                  // spaceChar = strchr(httpRequestParameterString, ' '); // Find end of parameter list
                  spaceChar = strchr(httpRequestParameterString, '\0'); // Find end of parameter list
                  // if (endChar != NULL) { // Check if a ' ' character was found
                  spaceCharIndex = (int) (spaceChar - httpRequestUriBuffer);
                  httpRequestUriBuffer[spaceCharIndex] = NULL; // Terminate the parameter key/value pair string
                  
                  httpRequestParameterCount++;
                  
                  haveExtractedParameters = true;
                  // }
                  
                  // TODO: Remove the following line. I don't think it's needed since the flag will break the loop on the next iteration.
                  // break; // Break out of the while loop
                }
              }
              
              /**
               * We have extracted parameter key/value pairs. Now, extract them, separating the key and value in each pair.
               */

              // Iterate over paramters' key/value pairs and parse each of them.
              if (haveExtractedParameters) {
                for (int i = 0; i < httpRequestParameterCount; i++) {
                  Serial.println(httpRequestParameters[i]);
                }
              }
              
            } else {
              // The '?' character was not found in the URI, so assume that no parameters exist.
              Serial.print("NOT found!\n");
            }
            
          }
          
          // you're starting a new line
          currentLineIsBlank = true;
          
        } else if (c != '\r') {
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

boolean handleDefaultHttpRequest(WiFiClient& client) {
  
  // Send a standard HTTP response header
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");  // the connection will be closed after completion of the response
  // client.println("Refresh: 5");  // refresh the page automatically every 5 sec
  client.println();
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  
  client.println("<h1>iRobot Create Web Server</h1>");
  
  // Output the value of each analog input pin
  // TODO: Ensure that this is compatible with the most common Arduinos, especially the Arduino Uno.
  client.println("<h2>Arduino Status</h2>");
  client.println("<h4>Analog Pins (Input)</h4>");
  for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
    int sensorReading = analogRead(analogChannel);
    client.print("A");
    client.print(analogChannel);
    client.print(" is ");
    client.print(sensorReading);
    client.println("<br />");
  }
  
  // TODO: Get state of Arduino's digital pins

  // Print the state of the iRobot Create sensors
  if (hasRoombaSensorData) {
    client.println("<h2>iRobot Create Status</h2>");
    client.println("<h3>Packet 6 (Full) Sensor Data</h3>");
      
    client.print("<h4>Bumps and Wheel Drops: ");
    client.println(buf[0], BIN);
    client.print("</h4>");
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
     
    client.print("<h4>Wall Sensor:");
    client.println(buf[1], BIN);
    client.print("</h4>");

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

    client.print("<h4>Virtual Wall: ");
    client.println(buf[6], BIN);
    client.print("</h4>");

    client.print("<h4>Low Side Drivers and Wheel Overcurrents: ");
    client.println(buf[7], BIN);
    client.print("</h4>");

    // Note the 2 unused bytes pg 18 of Open Interface Manual
    client.print("<h4>Infrared Byte: ");
    client.println(buf[10], BIN);
    client.print("</h4>");

    client.print("<h4>Buttons: ");
    client.println(buf[11], BIN);
    client.print("</h4>");
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
  client.flush();
}


void printWifiStatus() {
  
  Serial.println("Success! I connected to the following Wi-Fi network:");
  
  // print the SSID of the network you're attached to:
  Serial.print("\tSSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("\tIP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("\tSignal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  
  Serial.println();
}


int BitShiftCombine( unsigned char x_high, unsigned char x_low)
{


  int combined;  
  combined = x_high;              //send x_high to rightmost 8 bits
  combined = combined<<8;         //shift x_high over to leftmost 8 bits
  combined |= x_low;                 //logical OR keeps x_high intact in combined and fills in rightmost 8 bits 
  return combined;

}

