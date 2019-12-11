#include <StaticThreadController.h>
#include <ThreadController.h>
#include <Thread.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include "web_page.h"
#include "DHT.h"


#define DHTPIN 4
 
MDNSResponder mdns;
 
const char* ssid = "Redmi";
const char* password = "12345678";
 
ESP8266WebServer server(80);
DHT dht(DHTPIN, DHT11);

Thread serverThread = Thread();
Thread dhtThread = Thread();

String webPage = "";

int Temperature = 0;
int Humidity = 0;

int char_to_int(char s) {
  if (s < '0') return -1;
  if (s > '9') return -1;
  return s - '0';
}

String getWebPage() {
  webPage = WEB_PAGE;
  return webPage;
}

void handleClient() {
  server.handleClient();
}  

void dht_read() {
  Humidity = dht.readHumidity(); //Измеряем влажность
  Temperature = dht.readTemperature(); //Измеряем температуру
  if (isnan(Humidity) || isnan(Temperature)) return;
  Serial.write('T');
  Serial.write(char(Temperature / 10) + '0');
  Serial.write(char(Temperature % 10) + '0');
  delay(10);
  Serial.write('H');
  Serial.write(char(Humidity / 10) + '0');
  Serial.write(char(Humidity % 10) + '0');
}

void setup(void){
  Serial.begin(115200);
  delay(10);
  dht.begin();
  servo.attach(SERVO_PIN);
  servo.write(10);
  WiFi.begin(ssid, password);
  //Serial.println("");
 
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
  
  //Serial.println("");
  //Serial.print("Connected to ");
  //Serial.println(ssid);
  //Serial.print("IP address: ");
  //Serial.println(WiFi.localIP());
  
  serverThread.setInterval(10);
  dhtThread.setInterval(2000);
  
  serverThread.onRun(handleClient);
  dhtThread.onRun(dht_read);
 
  if (mdns.begin("esp8266", WiFi.localIP())) {
    //Serial.println("MDNS responder started");
  }
  server.on("/", [](){
    webPage = getWebPage();
    server.send(200, "text/html", webPage);
  });
  
  // Light
  // Outside
  server.on("/LightOutsideOn", [](){
    webPage = getWebPage();
    Serial.print("O1");
    server.send(200, "text/html", webPage);
  });
  server.on("/LightOutsideOff", [](){
    webPage = getWebPage();
    Serial.print("O0");
    server.send(200, "text/html", webPage);
  });
  server.on("/LightOutsideAuto", [](){
    webPage = getWebPage();
    //Serial.print("O2");
    server.send(200, "text/html", webPage);
  });
  // Inside
  server.on("/LightInsideOn", [](){
    webPage = getWebPage();
    Serial.print("L1");
    server.send(200, "text/html", webPage);
  });
  server.on("/LightInsideOff", [](){
    webPage = getWebPage();
    Serial.print("L0");
    server.send(200, "text/html", webPage);
  });

  // Ventilation
  server.on("/VentilationOn", [](){
    webPage = getWebPage();
    Serial.print("V1");
    server.send(200, "text/html", webPage);
  });
  server.on("/VentilationOff", [](){
    webPage = getWebPage();
    Serial.print("V0");
    server.send(200, "text/html", webPage);
  });
  server.on("/VentilationAuto", [](){
    webPage = getWebPage();
    Serial.print("V2");
    server.send(200, "text/html", webPage);
  });

  // Signalization
  server.on("/SignOn", [](){
    webPage = getWebPage();
    Serial.print("S1");
    server.send(200, "text/html", webPage);
  });
  server.on("/SignOff", [](){
    webPage = getWebPage();
    Serial.print("S0");
    server.send(200, "text/html", webPage);
  });

  // Door 
  server.on("/DoorOpen", [](){
    webPage = getWebPage();
    Serial.print("D1");
    //servo.write(170);
    server.send(200, "text/html", webPage);
  });
  server.on("/DoorClose", [](){
    webPage = getWebPage();
    Serial.print("D0");
    //servo.write(10);
    server.send(200, "text/html", webPage);
  });
  
  server.begin();
  //Serial.print("HTTP server started");
}
 
void loop(void){
  if (serverThread.shouldRun()) {
    serverThread.run();
  }
  if (dhtThread.shouldRun()) {
    dhtThread.run();
  }
}
