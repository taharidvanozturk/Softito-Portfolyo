#include <NTPClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <WiFiClient.h>
//#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>


#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ARDUINO_NANO_RP2040_CONNECT)
#include <WiFiNINA_Generic.h>
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#include <WiFiClientSecure.h>
#endif

#define THINGSBOARD_ENABLE_PSRAM 0
#define THINGSBOARD_ENABLE_DYNAMIC 1
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

//-----yazdıgımız kutuphaneyi tanimliyoruz--------
#include "titan.h"
//------------------------------------------------
//---------thingsboard led kontrol----------------
//#define GPIO0_PIN 0

//------------------------------------------------
#define kirmiziLed 0
#define kirmiziled_pin
#include <ThingsBoard.h>
//-------------pin tanimlamalari-----------------
#define gasSensor D4
#define buzzer D7
titan titan;
DHT dht(DHTPIN, DHTTYPE);
int gasError = 0;
//-----------------------------------------------
int adIp = 0;//ip adresi
int adMac =1;//mac adresi
int adtarih =2;
int adSaat=3;

int a;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
//-------------------------gaz sensoru interrupt---------------
void ICACHE_RAM_ATTR gasDetected() {
  gasError++;
  Serial.print(gasError);
  if (gasError == 3) {
    while(1) {
      Serial.print("HATA!! GAZ TESPİT EDİLDİ");
      digitalWrite(buzzer, HIGH);

      delay(500);


    } 
  }
  delay(333);
}
//--------------------------------------------------------------
constexpr char WIFI_SSID[] = "Taha";
constexpr char WIFI_PASSWORD[] = "tahataha";

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
constexpr char TOKEN[] = "3vuPck0ek9ceeeZZskBI";

// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "thingsboard.cloud";
// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port.
constexpr uint16_t THINGSBOARD_PORT = 1883U;

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint32_t MAX_MESSAGE_SIZE = 256U;

// Baud rate for the debugging serial connection.
// If the Serial output is mangled, ensure to change the monitor speed accordingly to this variable
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;


// Initialize underlying client, used to establish a connection
WiFiClient wifiClient;
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(wifiClient, MAX_MESSAGE_SIZE);

// Attribute names for attribute request and attribute updates functionality

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

// Statuses for subscribing to rpc
bool subscribed = false;

// handle led state and mode changes
volatile bool attributesChanged = false;

// LED modes: 0 - continious state, 1 - blinking
volatile int ledMode = 0;

// Current led state
volatile bool ledState = false;

// Settings for interval in blinking mode
constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

// For telemetry
constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;

// List of shared attributes for subscribing to their updates
constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};

// List of client attributes for requesting them (Using to initialize device states)
constexpr std::array<const char *, 1U> CLIENT_ATTRIBUTES_LIST = {
  LED_MODE_ATTR
};



AsyncWebServer server(80);
const char *ssid = "Taha";
const char *password = "tahataha";

const char *PARAM_INPUT_1 = "istenilen_sicaklik";
//const char* PARAM_INPUT_2 = "input2";
//const char* PARAM_INPUT_3 = "input3";

// HTML web page to handle 3 input fields (input1, input2, input3)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <h1> SICAKLIK GIRINIZ</h1>
  <form action="/get">
    input1: <input type="text" name="istenilen_sicaklik">
    <input type="submit" value="Submit">
  </form><br>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}




/// @brief Initalizes WiFi connection,
// will endlessly delay until a connection has been successfully established
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been succesfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

/// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
/// @return Returns true as soon as a connection has been established again
const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}


/// @brief Processes function for RPC call "setLedMode"
/// RPC_Data is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @return Response that should be sent to the cloud. Useful for getMethods
RPC_Response processSetLedMode(const RPC_Data &data) {
  Serial.println("Received the set led state RPC method");

  // Process data
  int new_mode = data;

  Serial.print("Mode to change: ");
  Serial.println(new_mode);

  if (new_mode != 0 && new_mode != 1) {
    return RPC_Response("error", "Unknown mode!");
  }

  ledMode = new_mode;

  attributesChanged = true;

  // Returning current mode
  return RPC_Response("newMode", (int)ledMode);
}


// Optional, keep subscribed shared attributes empty instead,
// and the callback will be called for every shared attribute changed on the device,
// instead of only the one that were entered instead
const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "setLedMode", processSetLedMode }
};


/// @brief Update callback that will be called as soon as one of the provided shared attributes changes value,
/// if none are provided we subscribe to any shared attribute change instead
/// @param data Data containing the shared attributes that were changed and their current value
void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
        blinkingInterval = new_interval;
        Serial.print("Updated blinking interval to: ");
        Serial.println(new_interval);
      }
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(kirmiziLed, ledState ? HIGH : LOW);
      Serial.print("Updated state to: ");
      Serial.println(ledState);
    }
  }
  attributesChanged = true;
}

void processClientAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), LED_MODE_ATTR) == 0) {
      const uint16_t new_mode = it->value().as<uint16_t>();
      ledMode = new_mode;
    }
  }
}

const Shared_Attribute_Callback attributes_callback(SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend(), &processSharedAttributes);
const Attribute_Request_Callback attribute_shared_request_callback(SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend(), &processSharedAttributes);
const Attribute_Request_Callback attribute_client_request_callback(CLIENT_ATTRIBUTES_LIST.cbegin(), CLIENT_ATTRIBUTES_LIST.cend(), &processClientAttributes);

unsigned long epochTime;

// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}
//-------------------------------------------thingsboard led kontorl-----------------------
int status = WL_IDLE_STATUS;
boolean gpioState[] = { false, false };
//-----------------------------------------------------------------------------------------

void setup() {
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;

    }

    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
     a = inputMessage.toInt();
    Serial.println(inputMessage);
    int t = inputMessage.toInt();
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" + inputParam + ") with value: " + inputMessage + "<br><a href=\"/\">Return to Home Page</a>");
    tb.sendTelemetryString("istenilen_sicaklik", inputMessage.c_str());
  });
  server.onNotFound(notFound);
  server.begin();


  // Initalize serial connection for debugging
  Serial.begin(115200);
  pinMode(kirmiziLed, OUTPUT);
  delay(1000);
  InitWiFi();
  pinMode(buzzer, OUTPUT);
  timeClient.begin();
  Serial.println(WiFi.localIP());

  //timeClient.setTimeOffset();
  //---------------------------main init--------------------------------------
  titan.init();
  dht.begin();
  attachInterrupt(digitalPinToInterrupt(gasSensor), gasDetected, FALLING);
  //--------------------------------------------------------------------------
  int ip =WiFi.localIP();
  String mac = WiFi.macAddress();
  String tarih = tarihyaz();
  int saat = getTime();
  EEPROM.put(0,ip);
  EEPROM.put(1, mac);
  EEPROM.put(2,tarih);
  EEPROM.put(3,saat);

  
}

String tarihyaz() {
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  //Get a time structure
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon + 1;
  int currentYear = ptm->tm_year + 1900;
  //Print complete date:
  String currentDate = String(monthDay) + "-" + String(currentMonth) + "-" + String(currentYear);
  return currentDate;
}

void loop() {
  
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  int ledS = titan.butonOkuma();
  int fanS = titan.fanControl(t,a);
  int suBaskin = titan.suKontrol();
  int isiS=~fanS;
  timeClient.update();


  if (!reconnect()) {
    subscribed = false;
    return;
  }

  if (!tb.connected()) {
    subscribed = false;
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
    // Sending a MAC address as an attribute
    tb.sendAttributeString("macAddress", WiFi.macAddress().c_str());
  }

  if (!subscribed) {
    Serial.println("Subscribing for RPC...");
    // Perform a subscription. All consequent data processing will happen in
    // processSetLedState() and processSetLedMode() functions,
    // as denoted by callbacks array.
    if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
      Serial.println("Failed to subscribe for shared attribute updates");
      return;
    }

    Serial.println("Subscribe done");
    subscribed = true;

    // Request current states of shared attributes
    if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
      Serial.println("Failed to request for shared attributes");
      return;
    }

    // Request current states of client attributes
    if (!tb.Client_Attributes_Request(attribute_client_request_callback)) {
      Serial.println("Failed to request for client attributes");
      return;
    }
  }

  if (attributesChanged) {
    attributesChanged = false;
    if (ledMode == 0) {
      previousStateChange = millis();
    }
    tb.sendTelemetryInt(LED_MODE_ATTR, ledMode);
    tb.sendTelemetryBool(LED_STATE_ATTR, ledState);
    tb.sendAttributeInt(LED_MODE_ATTR, ledMode);
    tb.sendAttributeBool(LED_STATE_ATTR, ledState);
  }

  if (ledMode == 1 && millis() - previousStateChange > blinkingInterval) {
    previousStateChange = millis();
    ledState = !ledState;
    digitalWrite(kirmiziLed, ledState);
    tb.sendTelemetryBool(LED_STATE_ATTR, ledState);
    tb.sendAttributeBool(LED_STATE_ATTR, ledState);
    if (kirmiziLed == 0) {
      Serial.print("LED state changed to: ");
      Serial.println(ledState);
    }
  }

  // Sending telemetry every telemetrySendInterval time
  if (millis() - previousDataSend > telemetrySendInterval) {
    previousDataSend = millis();
    tb.sendTelemetryInt("temperature", t);
    tb.sendTelemetryInt("nem", h);
    tb.sendTelemetryInt("suBaskin", suBaskin);
    tb.sendTelemetryInt("ledS", ledS);
    tb.sendTelemetryInt("fanS", fanS);
    tb.sendTelemetryInt("isiticiS",isiS);
    if (gasError == 3) {
      tb.sendTelemetryInt("errorGas", 1);

    }
    
  
    
     
    tb.sendAttributeInt("rssi", WiFi.RSSI());
    tb.sendAttributeInt("channel", WiFi.channel());
    tb.sendAttributeString("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeString("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeString("ssid", WiFi.SSID().c_str());
    
    tb.sendAttributeString("macAddress", WiFi.macAddress().c_str());
    tb.sendTelemetryString("macAdress", WiFi.macAddress().c_str());
    tb.sendTelemetryString("macAdress", WiFi.macAddress().c_str());
    tb.sendTelemetryString("Ip", WiFi.localIP().toString().c_str());
    tb.sendTelemetryString("dateTime", tarihyaz().c_str());
    tb.sendTelemetryInt("timeStamp", getTime());
     
     /*
     EEPROM.write(0,42);

     char[] value = EEPROM.read(0);
      Serial.println(value);*/






    //datetime da eklenecek
  }

  tb.loop();
}
