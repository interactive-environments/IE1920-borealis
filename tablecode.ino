/** 
 *  Code for tables.
 *  
 *  This code connects to a network and an MQTT broker,
 *  then loops to check for a cup being placed down. 
 *  
 *  If a cup is  placed, then we identify the cup, then
 *  publish a MQTT message with the cup, coaster and table
 *  details.
 *  
 */


/**
   Typical pin layout used:
   -----------------------------------------------------------------------------------------
               MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
               Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
   Signal      Pin          Pin           Pin       Pin        Pin              Pin
   -----------------------------------------------------------------------------------------
   RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
   SPI SS 1    SDA(SS)      ** custom, take a unused pin, only HIGH/LOW required *
   SPI SS 2    SDA(SS)      ** custom, take a unused pin, only HIGH/LOW required *
   SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
   SPI MISO    MISO         12 / ICSP-1   50        D12  9016      ICSP-1           14
   SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
*/

#include <WiFiNINA.h>
#include <SPI.h>
#include <MFRC522.h>
#include <MQTT.h>

const char WIFI_SSID[] = "iot-net"; // WiFI ssid 
const char WIFI_PASS[] = "interactive"; //WiFI password

const char mqttServer[] = "broker.shiftr.io"; // broker, with shiftr.io it's "broker.shiftr.io"
const int mqttServerPort = 1883; // broker mqtt port
const char key[] = "c44aa92f"; // broker key
const char secret[] = "fbd69143e3eb2f56"; // broker secret
const char device[] = "arduinolili"; // broker device identifier

#define RST_PIN         9          // Configurable, see typical pin layout above
#define SS_1_PIN        10         // Configurable, take a unused pin, only HIGH/LOW required, must be diffrent to SS 2
#define SS_2_PIN        8          // Configurable, take a unused pin, only HIGH/LOW required, must be diffrent to SS 1

#define NR_OF_READERS   2

int Continent;

byte ssPins[] = {SS_1_PIN, SS_2_PIN};


int status = WL_IDLE_STATUS;
WiFiClient net;
MQTTClient client;

unsigned long lastMillis = 0;

void connect() {
  Serial.print("checking wifi...");
  while ( status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected to WiFi!\n");

  client.begin(mqttServer, mqttServerPort, net);

  Serial.println("connecting to broker...");
  while (!client.connect(device, key, secret)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Connected to MQTT");

  client.onMessage(messageReceived);

  client.subscribe("/hello");
  // client.unsubscribe("/hello");
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

int A = 0;

int interval = 1000;
int oldTime = millis();


MFRC522 mfrc522[NR_OF_READERS];   // Create MFRC522 instance.

void setup() {  
  Serial.begin(9600); // Initialize serial communications with the PC


  //Begin Wifi and MQTT activities
  connect();

  //Begin card reader activities

  SPI.begin();        
  // Init SPI bus

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    mfrc522[reader].PCD_Init(ssPins[reader], RST_PIN); // Init each MFRC522 card
    Serial.print(F("Reader "));
    Serial.print(reader);
    Serial.print(F(": "));
    mfrc522[reader].PCD_DumpVersionToSerial();
  }
}

char currentVal;

void loop() {
  client.loop();
  
  //Begin card reader activities

  for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {
    // Look for new cards
    if (mfrc522[reader].PICC_IsNewCardPresent() && mfrc522[reader].PICC_ReadCardSerial()) {
      Serial.print(F("Reader "));
      Serial.print(reader);

      // Show some details of the PICC (that is: the tag/card)
      Serial.print(F(": Card UID:"));
      printDec(mfrc522[reader].uid.uidByte, mfrc522[reader].uid.size);
      Serial.println();
      //Serial.println((char*)mfrc522[reader].uid.uidByte);
      //Send UID to server
      oldTime = millis();
      Serial.println("sending req");

      
  if (!net.connected()) {
    connect();
  }

      client.publish("/hello", String(Continent) + ":" + String(reader) + ":" + String(A));

      Serial.print(F(": Card UID DEC:"));

      Serial.println();
      Serial.print(F("PICC type: "));
      MFRC522::PICC_Type piccType = mfrc522[reader].PICC_GetType(mfrc522[reader].uid.sak);
      Serial.println(mfrc522[reader].PICC_GetTypeName(piccType));
      Serial.println(Continent);
      // Halt PICC
      mfrc522[reader].PICC_HaltA();
      // Stop encryption on PCD
      mfrc522[reader].PCD_StopCrypto1();
    }
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/**
   Helper routine to dump a byte array as hex values to Serial.
*/
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
   Helper routine to dump a byte array as dec values to Serial.
*/
void printDec(byte *buffer, byte bufferSize) {
  String uniqueKey = "";

  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i], DEC);
    uniqueKey += buffer[i];
  }
  if (uniqueKey == "411732143" || uniqueKey =="42210214624101129" || uniqueKey == "42298612292100128" ) {
    Serial.println("America");
    Continent = 0;
    
  }
  else if (uniqueKey == "423410214624101128" || uniqueKey =="4248542178101128" || uniqueKey == "4158542178101128" ) {
    Serial.println("Europe");
    Continent = 1;
  }
  else if (uniqueKey == "4218542178101128" || uniqueKey =="4187542178101128" || uniqueKey == "424010214624101128" ) {
    Serial.println("Africa");
    Continent = 2;
  }
  else if (uniqueKey == "42310214624101129" || uniqueKey =="416910214624101128") {
    Serial.println("Asia");
    Continent = 3;
  }
}
