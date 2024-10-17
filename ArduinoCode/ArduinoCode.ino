/**********************************************************************************
    TITLE: Arduino IoT Cloud + IR + Manual (Switch) control 4 Relays with DHT11 & Real-Time feedback using ESP32
    Click on the following links to learn more.
    YouTube Video: https://youtu.be/UlT72MMbPu8
    Related Blog :
    by Tech StudyCell
    Preferences--> Aditional boards Manager URLs :
    https://dl.espressif.com/dl/package_esp32_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json

    Download Board ESP32 : https://github.com/espressif/arduino-esp32
    Download the libraries
    ArduinoIoTCloud Library (Version 2.0.1) with all the dependencies: https://github.com/arduino-libraries/ArduinoIoTCloud
    IRremote Library (Version 3.3.0): https://github.com/Arduino-IRremote/Arduino-IRremote
    DHT Library (Version 1.4.6): https://github.com/adafruit/DHT-sensor-library
 **********************************************************************************/

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include <IRremote.h>
#include <DHT.h>

const char DEVICE_LOGIN_NAME[]  = "dbfa5e5d-9dde-4be9-bfea-b23afb2e44d5"; //Enter DEVICE ID

const char SSID[]               = "AMD";    //Enter WiFi SSID (name)
const char PASS[]               = "   hoda   ";    //Enter WiFi password
const char DEVICE_KEY[]         = "VR5ZvHa!cu0un@GWHTLJ6pfFN";    //Enter Secret device password (Secret Key)

#define DHTPIN              4 //D4  pin connected with DHT
#define IR_RECV_PIN         35 //D35 pin connected with IR Receiver IC

// define the GPIO connected with Relays and switches
#define RelayPin1 23  //D23
#define RelayPin2 19  //D19
#define RelayPin3 18  //D18
#define RelayPin4 5  //D5

#define SwitchPin1 13  //D13
#define SwitchPin2 12  //D12
#define SwitchPin3 14  //D14
#define SwitchPin4 27  //D27

#define wifiLed    2   //D2

// Uncomment whatever type you're using!
#define DHTTYPE DHT11     // DHT 11
//#define DHTTYPE DHT22   // DHT 22, AM2302, AM2321
//#define DHTTYPE DHT21   // DHT 21, AM2301


DHT dht(DHTPIN, DHTTYPE);
IRrecv irrecv(IR_RECV_PIN);
decode_results results;

int toggleState_1 = 0; //Define integer to remember the toggle state for relay 1
int toggleState_2 = 0; //Define integer to remember the toggle state for relay 2
int toggleState_3 = 0; //Define integer to remember the toggle state for relay 3
int toggleState_4 = 0; //Define integer to remember the toggle state for relay 4

// Switch State
bool SwitchState_1 = LOW;
bool SwitchState_2 = LOW;
bool SwitchState_3 = LOW;
bool SwitchState_4 = LOW;

float temperature1  = 0;
float humidity1     = 0;
int   reconnectFlag = 0;

void onLight1Change();
void onLight2Change();
void onLight3Change();
void onLight4Change();

CloudSwitch LED1;
CloudSwitch LED2;
CloudSwitch LED3;
CloudSwitch LED4;
CloudTemperatureSensor temperature;

//for Initialzing the Arduino IoT cloud services
void initProperties()
{

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(LED1, READWRITE, ON_CHANGE, onLight1Change);
  ArduinoCloud.addProperty(LED2, READWRITE, ON_CHANGE, onLight2Change);
  ArduinoCloud.addProperty(LED3, READWRITE, ON_CHANGE, onLight3Change);
  ArduinoCloud.addProperty(LED4, READWRITE, ON_CHANGE, onLight4Change);
  ArduinoCloud.addProperty(temperature, READ, 10 * SECONDS, NULL); //Update temperature value after every 8 seconds
}
//Handler for the Wifi SSID and Password.
WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);

//Function used to read the Temperature sensor.
void readSensor()
{

  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  else
  {
    humidity1 = h;
    temperature = t;
    // Serial.println(tempareture);
  }
}

void ir_remote_control()
{
  if (irrecv.decode(&results))
  {
    switch (results.value)
    {
      case 0x80BF49B6:  relayOnOff(1); LED1 = toggleState_1; break; //update the HEX-code
      case 0x80BFC936:  relayOnOff(2); LED2 = toggleState_2; break; //update the HEX-code
      case 0x80BF33CC:  relayOnOff(3); LED3 = toggleState_3; break; //update the HEX-code
      case 0x80BF718E:  relayOnOff(4); LED4 = toggleState_4; break; //update the HEX-code
      default : break;
    }
    //Serial.println(results.value, HEX);
    irrecv.resume();
  }
}

void relayOnOff(int relay)
{
  switch (relay)
  {
    case 1:
      if (toggleState_1 == 0)
      {
        digitalWrite(RelayPin1, LOW); // turn on relay 1
        toggleState_1 = 1;
        Serial.println("Device1 ON");
      }
      else
      {
        digitalWrite(RelayPin1, HIGH); // turn off relay 1
        toggleState_1 = 0;
        Serial.println("Device1 OFF");
      }
      delay(100);
      break;
    case 2:
      if (toggleState_2 == 0)
      {
        digitalWrite(RelayPin2, LOW); // turn on relay 2
        toggleState_2 = 1;
        Serial.println("Device2 ON");
      }
      else
      {
        digitalWrite(RelayPin2, HIGH); // turn off relay 2
        toggleState_2 = 0;
        Serial.println("Device2 OFF");
      }
      delay(100);
      break;
    case 3:
      if (toggleState_3 == 0)
      {
        digitalWrite(RelayPin3, LOW); // turn on relay 3
        toggleState_3 = 1;
        Serial.println("Device3 ON");
      }
      else
      {
        digitalWrite(RelayPin3, HIGH); // turn off relay 3
        toggleState_3 = 0;
        Serial.println("Device3 OFF");
      }
      delay(100);
      break;
    case 4:
      if (toggleState_4 == 0)
      {
        digitalWrite(RelayPin4, LOW); // turn on relay 4
        toggleState_4 = 1;
        Serial.println("Device4 ON");
      }
      else
      {
        digitalWrite(RelayPin4, HIGH); // turn off relay 4
        toggleState_4 = 0;
        Serial.println("Device4 OFF");
      }
      delay(100);
      break;
    default : break;
  }
}

void manual_control()
{
  if (digitalRead(SwitchPin1) == LOW && SwitchState_1 == LOW)
  {
    digitalWrite(RelayPin1, LOW);
    toggleState_1 = 1;
    SwitchState_1 = HIGH;
    LED1 = toggleState_1;
    Serial.println("Switch-1 on");
  }
  if (digitalRead(SwitchPin1) == HIGH && SwitchState_1 == HIGH)
  {
    digitalWrite(RelayPin1, HIGH);
    toggleState_1 = 0;
    SwitchState_1 = LOW;
    LED1 = toggleState_1;
    Serial.println("Switch-1 off");
  }
  if (digitalRead(SwitchPin2) == LOW && SwitchState_2 == LOW)
  {
    digitalWrite(RelayPin2, LOW);
    toggleState_2 = 1;
    SwitchState_2 = HIGH;
    LED2 = toggleState_2;
    Serial.println("Switch-2 on");
  }
  if (digitalRead(SwitchPin2) == HIGH && SwitchState_2 == HIGH)
  {
    digitalWrite(RelayPin2, HIGH);
    toggleState_2 = 0;
    SwitchState_2 = LOW;
    LED2 = toggleState_2;
    Serial.println("Switch-2 off");
  }
  if (digitalRead(SwitchPin3) == LOW && SwitchState_3 == LOW)
  {
    digitalWrite(RelayPin3, LOW);
    toggleState_3 = 1;
    SwitchState_3 = HIGH;
    LED3 = toggleState_3;
    Serial.println("Switch-3 on");
  }
  if (digitalRead(SwitchPin3) == HIGH && SwitchState_3 == HIGH)
  {
    digitalWrite(RelayPin3, HIGH);
    toggleState_3 = 0;
    SwitchState_3 = LOW;
    LED3 = toggleState_3;
    Serial.println("Switch-3 off");
  }
  if (digitalRead(SwitchPin4) == LOW && SwitchState_4 == LOW)
  {
    digitalWrite(RelayPin4, LOW);
    toggleState_4 = 1;
    SwitchState_4 = HIGH;
    LED4 = toggleState_4;
    Serial.println("Switch-4 on");
  }
  if (digitalRead(SwitchPin4) == HIGH && SwitchState_4 == HIGH)
  {
    digitalWrite(RelayPin4, HIGH);
    toggleState_4 = 0;
    SwitchState_4 = LOW;
    LED4 = toggleState_4;
    Serial.println("Switch-4 off");
  }
}

//CallBack functions that is gonna be used for Feedback on Connect, sync, or Disconnection.
void doThisOnConnect()
{
  /* add your custom code here */
  Serial.println("Board successfully connected to Arduino IoT Cloud");
  digitalWrite(wifiLed, HIGH); //Turn on WiFi LED
}
void doThisOnSync()
{
  /* add your custom code here */
  Serial.println("Thing Properties synchronised");
}

void doThisOnDisconnect()
{
  /* add your custom code here */
  Serial.println("Board disconnected from Arduino IoT Cloud");
  digitalWrite(wifiLed, LOW); //Turn off WiFi LED
}

void setup()
{
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500);

  // Defined in thingProperties.h
  initProperties();                                   // Start Arduino IoT Cloud services
  dht.begin();                                        // Start Temperature sensor
  irrecv.enableIRIn();                                // Start IR receiver
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);  // Connect to Wifi and Arduino IoT Cloud

  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::CONNECT,     doThisOnConnect);       //CallBack Function that Calls some Function to do some Action like on Connect here
  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::SYNC,        doThisOnSync);          //CallBack Function that Calls some Function to do some Action like on Sync here
  ArduinoCloud.addCallback(ArduinoIoTCloudEvent::DISCONNECT,  doThisOnDisconnect);    //CallBack Function that Calls some Function to do some Action like on Disconnect here

  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();    // Start Debug feature.

  /* Configure GPIO pins */
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin4, OUTPUT);
  
  pinMode(wifiLed, OUTPUT);

  pinMode(SwitchPin1, INPUT_PULLUP);
  pinMode(SwitchPin2, INPUT_PULLUP);
  pinMode(SwitchPin3, INPUT_PULLUP);
  pinMode(SwitchPin4, INPUT_PULLUP);

  //During Starting all Relays should TURN OFF
  digitalWrite(RelayPin1, HIGH);
  digitalWrite(RelayPin2, HIGH);
  digitalWrite(RelayPin3, HIGH);
  digitalWrite(RelayPin4, HIGH);
}

void loop()
{
  ArduinoCloud.update();    // Start Loop that updates the Variables here with one on the cloud

  manual_control();       //Manual Control
  ir_remote_control();    //IR Remote Control
  readSensor();           //Get Sensor Data
}

void onLight1Change()
{
  //Control the device
  if (LED1 == 1)
  {
    digitalWrite(RelayPin1, LOW);
    Serial.println("Device1 ON");
    toggleState_1 = 1;
  }
  else
  {
    digitalWrite(RelayPin1, HIGH);
    Serial.println("Device1 OFF");
    toggleState_1 = 0;
  }
}

void onLight2Change()
{
  if (LED2 == 1)
  {
    digitalWrite(RelayPin2, LOW);
    Serial.println("Device2 ON");
    toggleState_2 = 1;
  }
  else
  {
    digitalWrite(RelayPin2, HIGH);
    Serial.println("Device2 OFF");
    toggleState_2 = 0;
  }
}

void onLight3Change()
{
  if (LED3 == 1)
  {
    digitalWrite(RelayPin3, LOW);
    Serial.println("Device3 ON");
    toggleState_3 = 1;
  }
  else
  {
    digitalWrite(RelayPin3, HIGH);
    Serial.println("Device3 OFF");
    toggleState_3 = 0;
  }
}

void onLight4Change()
{
  if (LED4 == 1)
  {
    digitalWrite(RelayPin4, LOW);
    Serial.println("Device4 ON");
    toggleState_4 = 1;
  }
  else
  {
    digitalWrite(RelayPin4, HIGH);
    Serial.println("Device4 OFF");
    toggleState_4 = 0;
  }
}
