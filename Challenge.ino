#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFi.h>

WiFiClient clientPython;
WiFiClient clientJetson;
WiFiClient clientJetson2;
WiFiServer serverPython(80);

const char* ssid = "elfuhrer";
const char* password = "ciber123";

//const char* ssid = "INFINITUM99A4_2.4";
//const char* password = "Rt2VxKWFyu";

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting..");
  while(WiFi.status() != WL_CONNECTED) { 
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  serverPython.begin();
  Serial.print("Client IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  clientPython = serverPython.available();
  if (clientPython){
    Serial.println("Python is connected!");
    while (clientPython.connected()){
      while(!clientPython.available()){
        if (!clientPython.connected())
          break;
        delay(10);
      }
      sendData(clientPython.readStringUntil('\n'));
      clientPython.flush();
    }
  }
}

void sendData(String a){
  String values[2];
  splitString(a, '|', values, 2);
  Serial.print("Data vehicle 1: ");
  Serial.println(values[0]);
  Serial.print("Data vehicle 2: ");
  Serial.println(values[1]);
  if (clientJetson.connect("192.168.149.1", 2030))
    clientJetson.println(values[0]);
  if (clientJetson2.connect("192.168.149.242", 2030))
    clientJetson2.println(values[1]);
}

void splitString(String input, char delimiter, String* outputArray, int numValues) {
  int delimiterIndex;
  for (int i = 0; i < numValues; i++) {
    delimiterIndex = input.indexOf(delimiter);
    if (delimiterIndex != -1) {
      outputArray[i] = input.substring(0, delimiterIndex);
      input = input.substring(delimiterIndex + 1);
    } else {
      outputArray[i] = input; // If there are fewer values than expected
      break;
    }
  }
}
