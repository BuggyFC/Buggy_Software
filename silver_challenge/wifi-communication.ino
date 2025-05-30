/*
THIS WORKS WITH PROCESSING, HOWEVER DOES NOT INCLUDE ANY OF THE CODE FOR THE BUGGY
GUI SENDS 'S' FOR STOP, 'W' FOR START.
CHANGING SSID AND PASSWORD FOR SOME REASON MAKES IT STOP WORKING
*/

#include <WiFiS3.h>
char ssid[] = "my_wifi_network";
char pass[] = "2E10Project";
WiFiServer server(5200);
void setup() {
 Serial.begin(9600);
 WiFi.beginAP(ssid, pass);
 IPAddress ip = WiFi.localIP();
 Serial.print("IP Address:");
 Serial.println(ip);
 server.begin();
}
void loop() {
 WiFiClient client = server.available();
 if (client.connected()) 
  client.write("Hello Client");
  char c = client.read();
  if (c != -1)
  Serial.println(c);
  delay(1000);
 }
/*
  WiFi UDP Send and Receive String
 This sketch wait an UDP packet on localPort using a WiFi shield.
 When a packet is received an Acknowledge packet is sent to the client on port remotePort
*/

#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

int status = WL_IDLE_STATUS;
char ssid[] = "ssid"; //  your network SSID (name)
char pass[] = "pass";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 123;      // local port to listen on, make sure its the same on processing

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

void setup() {
  Serial.begin(9600); //Initialize serial and wait for port to open:
  if (WiFi.status() == WL_NO_MODULE) { // check for the presence of the module:
    Serial.println("WiFi module not connected");
    while (true); // don't continue:
  }
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  while (status != WL_CONNECTED) {   // attempt to connect to Wifi network:
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network.
    status = WiFi.begin(ssid,pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();
  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);  // if you get a connection, report back via serial.
}

void loop() {
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
