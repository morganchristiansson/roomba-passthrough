#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#define packTimeout 5 // ms (if nothing more on UART, then send packet)
#define bufferSize 8192

#include <SoftwareSerial.h>
#define Wake_Pin      D2
#define SERIAL_RX     D3  // pin for SoftwareSerial RX
#define SERIAL_TX     D4 // pin for SoftwareSerial TX
SoftwareSerial mySerial(SERIAL_RX, SERIAL_TX); // (RX, TX. inverted, buffer)

String ssid    = "";
String password = "";
String espName    = "roomba1";
const int port = 9001;

const char *APssid = "roombawifi";
const char *APpassword = "roombawifipass";

String WMode = "1";


WiFiServer server(port);
WiFiClient client;

uint8_t buf1[bufferSize];
uint8_t i1=0;

uint8_t buf2[bufferSize];
uint8_t i2=0;

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  pinMode(SERIAL_RX, INPUT);
  pinMode(SERIAL_TX, OUTPUT);
  pinMode(Wake_Pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(Wake_Pin,LOW);
  mySerial.begin(115200);  // uncomment this line to use SoftSerial
  WiFi.begin(ssid.c_str(), password.c_str());
  int i = 0;
  while (WiFi.status() != WL_CONNECTED && i < 31)
  {
    delay(1000);
    Serial.print(".");
    ++i;
  }
  if (WiFi.status() != WL_CONNECTED && i >= 30)
  {
    WiFi.disconnect();
    delay(1000);
    Serial.println("");
    Serial.println("Couldn't connect to network :( ");
    Serial.println("Setting up access point");
    Serial.println("SSID: ");
    Serial.println(APssid);
    Serial.println("password: ");
    Serial.println(APpassword);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(APssid, APpassword);
    WMode = "AP";
    Serial.print("Connected to ");
    Serial.println(APssid);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("IP address: ");
    Serial.println(myIP);

  }
  else
  {
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Hostname: ");
    Serial.println(espName);

  }

  Serial.println("Starting TCP Server");
  server.begin(); // start TCP server 
}

void handle_roomba_wake(){
  digitalWrite(Wake_Pin, HIGH);
  delay(100);
  digitalWrite(Wake_Pin, LOW);
  delay(500);
  digitalWrite(Wake_Pin, HIGH);
  delay(100);
}

void loop() {
  if(!client.connected()) { // if client not connected
    client = server.available(); // wait for it to connect
    if(client) {
        handle_roomba_wake();
        Serial.println("connect!");
    }
    return;
  }

  // here we have a connected client

  if(client.available()) {
    while(client.available()) {
      buf1[i1] = (uint8_t)client.read(); // read char from client (RoboRemo app)
      if(i1<bufferSize-1) i1++;
    }
    // now send to UART:
    mySerial.write(buf1, i1);
    Serial.write((char*)buf2, i2);
    i1 = 0;
  }

  if(mySerial.available()) {

    // read the data until pause:
    
    while(1) {
      if(mySerial.available()) {
        buf2[i2] = (char)mySerial.read(); // read char from UART
        if(i2<bufferSize-1) i2++;
      } else {
        //delayMicroseconds(packTimeoutMicros);
        delay(packTimeout);
        if(!mySerial.available()) {
          break;
        }
      }
    }
    
    // now send to WiFi:
    client.write((char*)buf2, i2);
    Serial.write((char*)buf2, i2);
    i2 = 0;
  }
}
