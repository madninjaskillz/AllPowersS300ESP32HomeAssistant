/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <WiFi.h>
#include <ArduinoHA.h>
#include <ESPmDNS.h>

#define bleServerName "AP S300 V2.0"
#define BROKER_ADDR IPAddress(192,168,10,1)

byte mac[] = {0x00, 0x10, 0xFA, 0x6F, 0x38, 0x4A};
WiFiClient client;
HADevice hadevice(mac, sizeof(mac));
HAMqtt mqtt(client, hadevice);
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
int scanTime = 5; //In seconds
bool keepScanning = true;
BLEAdvertisedDevice device;
BLEScan* pBLEScan;
static BLEAddress *pServerAddress;
static BLEUUID serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");
static BLEUUID    charUUID("0000fff1-0000-1000-8000-00805f9b34fb");
static bool haveAnnounced = false;
static int nonAnnounceCount = 0;

// Client variables
char linebuf[80];
int charcount = 0;

static BLERemoteCharacteristic* pRemoteCharacteristic;

HASensorNumber outputPower("outputPower", HASensorNumber::PrecisionP1);
HASensorNumber inputPower("inputPower", HASensorNumber::PrecisionP1);
HASensorNumber minutesRemaining("minutesRemaining", HASensorNumber::PrecisionP1);
HASensorNumber batteryLevel("batteryLevelPercentage", HASensorNumber::PrecisionP1);

HABinarySensor acsensor("acOn");
HABinarySensor dcsensor("dcOn");
HABinarySensor ledsensor("ledOn");
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("Found ");
      Serial.println(advertisedDevice.getName().c_str());
      if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
        advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
        pServerAddress = new BLEAddress(advertisedDevice.getAddress());
        Serial.println("Found correct device...");
        doConnect = true;
      }
    }
};

static bool connectToWifi = false;
void initWiFi(String ssid, char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print(F("Connecting to WiFi .."));
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  connectToWifi = true;

  Serial.println("Starting MQTT");
  startMqtt();
  Serial.println("MQTT started");

  Serial.println("Starting mDNS");
  if (!MDNS.begin("esp32")) {
    Serial.println("Error starting mDNS");
    return;
  }
  Serial.println("Started mDNS");
}

void scanWiFi() {
  // WiFi.scanNetworks will return the number of networks found
  Serial.println("Scanning Wifi");
  int n = WiFi.scanNetworks();

  if (n == 0) {
    Serial.println("no networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");


      if (WiFi.SSID(i) == "PatMorita") {
        Serial.println("Looks like a great network!");
        initWiFi(WiFi.SSID(i), "rubberduck");
        return;
      }

      if (WiFi.SSID(i) == "PatButcher" || WiFi.SSID(i) == "PatSharp" ) {
        Serial.println("Looks like a good network!");
        initWiFi(WiFi.SSID(i), "fredfred");
        return;
      }

      delay(10);
    }
  }
  Serial.println("");
}

void setup() {
  Serial.begin(115200);
}

void triggerBLEScan() {
  Serial.println("Scanning Bluetooth...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void startMqtt() {
  hadevice.setName("AllPowerss S300");
  hadevice.setSoftwareVersion("1.0.1");

  outputPower.setIcon("mdi:transmission-tower-export");
  outputPower.setName("Output Wattage");
  outputPower.setUnitOfMeasurement("Watts");

  inputPower.setIcon("mdi:transmission-tower-import");
  inputPower.setName("Input Wattage");
  inputPower.setUnitOfMeasurement("Watts");

  batteryLevel.setIcon("mdi:battery");
  batteryLevel.setName("Battery Level");
  batteryLevel.setUnitOfMeasurement("percent");

  minutesRemaining.setIcon("mdi:battery-clock");
  minutesRemaining.setName("Battery Remaining");
  minutesRemaining.setUnitOfMeasurement("minutes");

  acsensor.setName("AC Power");
  acsensor.setDeviceClass("plug");
  acsensor.setIcon("mdi:power-socket-uk");

  dcsensor.setName("DC Power");
  dcsensor.setDeviceClass("plug");
  dcsensor.setIcon("mdi:usb-port");

  ledsensor.setName("Flashlight");
  ledsensor.setDeviceClass("running");
  ledsensor.setIcon("mdi:flashlight");

  mqtt.begin(BROKER_ADDR, "allpowers", "allpowers");
}


static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  byte b = (byte)pData[7];
  acsensor.setState(bitRead(b, 1) == 1);
  dcsensor.setState(bitRead(b, 0) == 1);
  ledsensor.setState(bitRead(b, 4) == 1);

  outputPower.setValue((256 * pData[11]) + pData[12]);
  inputPower.setValue((pData[9] * 256) + pData[10]);

  batteryLevel.setValue(pData[8]);
  minutesRemaining.setValue((256 * pData[13]) + pData[14]);
  haveAnnounced = true;
  nonAnnounceCount = 0;
  Serial.println("Announced");
}


class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
    }

    void onDisconnect(BLEClient* pclient) {
      connected = false;
      Serial.println("onDisconnect");
    }
};

BLEClient* connectToServer(BLEAdvertisedDevice* device) {
  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  if (pClient->connect(device)) { // Connect to the remote BLE Server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      pClient->disconnect();
    }

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      pClient->disconnect();
    }
    connected = true;

    if (pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    return pClient;
  }
  else {
    return NULL;
  }
}

bool foundBattery = false;
void scanForBattery() {
  Serial.println("Scanning BLE Results for battery");
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  for (int i = 0; i < foundDevices.getCount(); i++) {
    device = foundDevices.getDevice(i);

    String deviceName = device.getName().c_str();
    if (deviceName == "AP S300 V2.0") {
      BLEAdvertisedDevice* myDevice = new BLEAdvertisedDevice(device);
      keepScanning = false;
      delay(2000);
      connectToServer(myDevice);
      BLEClient*  pClient  = BLEDevice::createClient();
      pClient->setClientCallbacks(new MyClientCallback());
      pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
      Serial.println(" - Connected to server");
      foundBattery = true;
    }
  }
}

int batteryScan = 0;
int haveTriggeredBLEScan = 0;
void loop() {

  if (!connectToWifi) {
    scanWiFi();
    delay(1000);
  }

  if (connectToWifi) {
    mqtt.loop();

  }

  if (!doConnect) {
    Serial.println("Still waiting to find battery");
    if (haveTriggeredBLEScan == 0) {
      haveTriggeredBLEScan = 100;
      triggerBLEScan();
    } else {
      Serial.print("Waiting to retrigger BLE Scan: ");
      Serial.println(haveTriggeredBLEScan);
      haveTriggeredBLEScan--;

      scanForBattery();

    }
  } else {

    if (!foundBattery) {
      if (batteryScan == 0) {
        batteryScan = 50;
        scanForBattery();
      } else {
        batteryScan-- ;
        Serial.print("BLE Scan count down...");
        Serial.println(batteryScan);
      }
    }
  }

  if (foundBattery) {
    nonAnnounceCount++;
    if (!haveAnnounced) {
      Serial.println("Waiting to announce");

      if (nonAnnounceCount > 30) {

        Serial.println("Rebooting sue to non announce");
        ESP.restart();
      }
    }

  }

  delay(1000);
}