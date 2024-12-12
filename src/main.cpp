#include <esp_now.h>
#include <time.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <esp_camera.h>
#include <sd_read_write.h>
#include <SD_MMC.h>

#define PIR_SENSOR_PIN 33 // 33 instaed of 14
#define LED_PIN 32        // 32 instead of 2 (on board led used by SD card)

// Select camera model
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define SD_MMC_CMD 15 // Please do not modify it.
#define SD_MMC_CLK 14 // Please do not modify it.
#define SD_MMC_D0 2   // Please do not modify it.

// MAC ADRESSES
#define DEVICE_0 {0x88, 0x13, 0xbf, 0x69, 0xca, 0x68}
#define DEVICE_1 {0x0c, 0xb8, 0x15, 0x07, 0x43, 0x7c}
#define DEVICE_2 {0x1c, 0x69, 0x20, 0xe6, 0x27, 0x80}
#define DEVICE_3 {0xac, 0x15, 0x18, 0xed, 0x44, 0xcc}
#define DEVICE_4 {0x88, 0x13, 0xbf, 0x69, 0x9e, 0xfc}
#define DEVICE_5 {0xcc, 0x7b, 0x5c, 0x97, 0xf0, 0xc8}
#define DEVICE_6 {0x10, 0x06, 0x1c, 0xd6, 0x46, 0xd8}

// Build the matrix with knowned devices
// Remember node 0 is a special node (BLE link with app)
// uint8_t nodes[][6] = {DEVICE_0, DEVICE_1, DEVICE_2, DEVICE_3, DEVICE_4};

// in this example, we have 3 device in the network, DEVICE_2 is the HUB (node0), DEVICE_3 is node1, DEVICE_3 is node2
uint8_t nodes[][6] = {DEVICE_5, DEVICE_6, DEVICE_1};

const int cur_node = 1;           // Node reference number (change this for each node you rebuild)
const int num_nodes = 3;          // Number of nodes
const int hub_node = 0;           // Hub, use for BLE connection with external app
const boolean pictureMode = true; // To enable or not the picture management

unsigned long lastMessageTime = 0;
const unsigned long messageInterval = 5000; // 5 seconds interval

#define MAX_IMAGE_CHUNK_SIZE 240 // Max size to ensure it fits within ESP-NOW payload limits
#define MAX_IMAGE_SIZE 50000     // Adjust according to your expected image size
uint8_t image_buffer[MAX_IMAGE_SIZE];
size_t total_bytes_received = 0;
bool receiving_image = false;

struct struct_message
{
  char msg[32];
  int origin_node;
  int current_node;
  uint8_t image_data[MAX_IMAGE_CHUNK_SIZE];
  size_t data_length;
};

struct_message myData;
esp_now_peer_info_t peerInfo;

NimBLEServer *pServer = NULL;
NimBLECharacteristic *pCharacteristic = NULL;   // motion notification
NimBLECharacteristic *pCharacteristic_2 = NULL; // health check
NimBLECharacteristic *pCharacteristic_3 = NULL; // send picture

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
int img_counter = 0; // Global variable for the image counter

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "50c07f71-e239-4f5c-825e-2cc13e914778"
#define CHARACTERISTIC_UUID_3 "0193b6d1-4e1b-745d-ac16-ba9af8fbb405"

camera_config_t config;

void config_init()
{
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_SVGA;   // Use SVGA resolution for better quality and memory usage
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM; // Use PSRAM for the frame buffer if available
  config.jpeg_quality = 5;                 // Decrease for higher quality (ranges from 0-63)
  config.fb_count = 2;
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root)
  {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels)
      {
        listDir(fs, file.path(), levels - 1);
      }
    }
    else
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char *path)
{
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path))
  {
    Serial.println("Dir created");
  }
  else
  {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char *path)
{
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path))
  {
    Serial.println("Dir removed");
  }
  else
  {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available())
  {
    Serial.write(file.read());
  }
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
}

void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2))
  {
    Serial.println("File renamed");
  }
  else
  {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char *path)
{
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path))
  {
    Serial.println("File deleted");
  }
  else
  {
    Serial.println("Delete failed");
  }
}
String getFileName()
{
  String filename = "R3E" + String(cur_node) + "_" + String(img_counter);
  img_counter++;
  return filename;
}

void takePicture()
{
  // Capture the picture
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }

  Serial.println("Picture taken, saving to SD card...");

  // Save the image to the SD card
  // Generate filename with date and time
  String path = "/" + getFileName() + ".jpg";
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    esp_camera_fb_return(fb);
    return;
  }
  file.write(fb->buf, fb->len);
  file.close();

  // Free the frame buffer after use
  esp_camera_fb_return(fb);

  Serial.println("Image saved to SD card successfully");
}

class MyServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(NimBLEServer *pServer)
  {
    deviceConnected = false;
  }
};

void handleHealthCheck(uint32_t node_src)
{
  strcpy(myData.msg, "HEALTH CHECK");
  myData.origin_node = node_src;
  myData.current_node = cur_node;
  for (int i = 0; i < 5; i++)
  {
    // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED_PIN, HIGH);
    // wait for a second
    delay(500);
    // turn the LED off by making the voltage LOW
    digitalWrite(LED_PIN, LOW);
    // wait for a second
    delay(500);
  }

  Serial.print("Node source (HC) :");
  Serial.println(node_src);
  Serial.print("cur_node :");
  Serial.println(cur_node);

  if (cur_node < num_nodes - 1)
  {
    esp_err_t result = esp_now_send(nodes[cur_node + 1], (uint8_t *)&myData, sizeof(myData));
    if (result == ESP_OK)
    {
      Serial.println("Sent to next node with success");
    }
    else
    {
      Serial.println("Error sending data to next node");
    }
  }
}

class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    Serial.print("Value written: ");
    Serial.println(value.c_str());
    handleHealthCheck(hub_node);
  }
};

void createBLEServer()
{
  // Create the BLE Device
  NimBLEDevice::init("R3E-HUB");

  // Create the BLE Server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  // used for motion notification
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::NOTIFY);

  // Create another BLE Characteristic with write property
  // used for Helth check mecanism
  pCharacteristic_2 = pService->createCharacteristic(CHARACTERISTIC_UUID_2, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  pCharacteristic_2->setCallbacks(new MyCharacteristicCallbacks());

  // used to send picture
  pCharacteristic_3 = pService->createCharacteristic(
      CHARACTERISTIC_UUID_3,
      NIMBLE_PROPERTY::NOTIFY);

  // Start the service
  pService->start();

  // Start advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  NimBLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");
}

void sendFileViaBLE(fs::FS &fs, String path)
{
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }
  const size_t chunkSize = 500; // Adjust the chunk size as needed
  uint8_t buffer[chunkSize];
  while (file.available())
  {
    size_t bytesRead = file.read(buffer, chunkSize);
    pCharacteristic_3->setValue(buffer, bytesRead);
    pCharacteristic_3->notify();
    delay(50); // Small delay to allow BLE transmission
  }
  file.close();
  Serial.println("File sent via BLE successfully");
}

void notifyBLEClient(uint32_t valueToNotify)
{
  // notify changed value
  Serial.println("Notifying BLE Client...");

  if (deviceConnected)
  {
    Serial.print("Setting the value : ");
    Serial.println(valueToNotify);

    pCharacteristic->setValue(valueToNotify);
    pCharacteristic->notify();
    delay(1000);
    // sending the last image taken
    if (pictureMode)
    {
      String filename = "R3E" + String(cur_node) + "_" + String(img_counter - 1);
      String path = "/" + filename + ".jpg";

      Serial.print("Sending file : " + path);
      sendFileViaBLE(SD_MMC, path);
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

void sendImageChunk(const struct_message *myData)
{
  // esp_err_t result = esp_now_send(nodes[cur_node - 1], (uint8_t *)myData, sizeof(*myData));
  esp_err_t result = esp_now_send(nodes[cur_node - 1], (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data" + result);
  }
}

void handleMotionDetection(uint32_t valueToNotify, camera_fb_t *fb)
{
  unsigned long currentTime = millis();
  if (currentTime - lastMessageTime > messageInterval)
  {
    lastMessageTime = currentTime;

    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);

    if (pictureMode && valueToNotify == cur_node) // take the picture only when the motion was detected on the current device, not relayed froman other device
    {                                             // Capture the picture
      Serial.println("Taking a snapshot..");

      camera_fb_t *image_data = esp_camera_fb_get();
      if (!image_data)
      {
        Serial.println("Camera capture failed");
        // return;
      }
      else
        fb = image_data;
    }
    // Save the image to the SD card
    // Generate filename with date and time
    if (cur_node == 0 && fb) // save all images on the HUB SD card
    {
      Serial.println("Saving File on SD card..");

      String path = "/" + getFileName() + ".jpg";
      File file = SD_MMC.open(path, FILE_WRITE);
      if (!file)
      {
        Serial.println("Failed to open file for writing");
        esp_camera_fb_return(fb);
        // return;
      }
      else
      {
        file.write(fb->buf, fb->len);
        file.close();
      }
    }

    Serial.print("ValueToNotify :");
    Serial.println(valueToNotify);
    Serial.print("cur_node :");
    Serial.println(cur_node);

    // Send message to previous node
    // Set values to send
    strcpy(myData.msg, "MOTION DETECTED");
    myData.origin_node = valueToNotify;
    myData.current_node = cur_node;
    if (cur_node > 0 && valueToNotify >= cur_node) // don't send if on the first node, don't send if msg is not coming from the next node in the network (reverse relay)
    {
      //--------
      // esp_err_t result = esp_now_send(nodes[cur_node - 1], (uint8_t *)&myData, sizeof(myData));
      size_t bytesToSend = fb->len;
      size_t bytesSent = 0;
      while (bytesToSend > 0)
      {
        // Serial.print("w1");

        size_t chunkSize = (bytesToSend > MAX_IMAGE_CHUNK_SIZE) ? MAX_IMAGE_CHUNK_SIZE : bytesToSend;
        // Serial.print("w2");

        memcpy(myData.image_data, fb->buf + bytesSent, chunkSize);
        // Serial.print("w3");

        myData.data_length = chunkSize;
        Serial.println("w4");

        sendImageChunk(&myData);
        Serial.println("w5");

        bytesSent += chunkSize;
        bytesToSend -= chunkSize;
        delay(10); // Small delay to ensure the receiver processes the data
      }

      //----------
      // if (result == ESP_OK)
      //{
      Serial.println("Sent to previous node with success");
      //}
      // else
      //{
      //  Serial.println("Error sending data to previous node");
      //}
    }
    // Node 0 is a special node, as the first node in the chain it is responsible for notifying the client (smartphone app)
    if (cur_node == 0)
    {
      notifyBLEClient(valueToNotify);
    }
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  //---------
  struct_message myData;
  memcpy(&myData, incomingData, sizeof(myData));
  // if (strcmp(myData.msg, "IMG_DATA") == 0)
  //{
  if (!receiving_image)
  {
    total_bytes_received = 0;
    receiving_image = true;
    Serial.println("Starting to receive image...");
  }
  memcpy(image_buffer + total_bytes_received, myData.image_data, myData.data_length);
  total_bytes_received += myData.data_length;
  Serial.printf("Received chunk of length: %d, Total bytes received: %d\n", myData.data_length, total_bytes_received);
  if (myData.data_length < MAX_IMAGE_CHUNK_SIZE)
  {
    receiving_image = false;
    Serial.println("Image received successfully");

    //}
    //----------
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("Char: ");
    Serial.println(myData.msg);
    Serial.print("Origin Node: ");
    Serial.println(myData.origin_node);
    Serial.print("Current Node: ");
    Serial.println(myData.current_node);
    Serial.println();

    if (strcmp(myData.msg, "HEALTH CHECK") == 0)
    {
      // relay msg to the next Node
      handleHealthCheck(myData.origin_node);
    }
    else
    {
      // Relay the notification to the previous Node
      handleMotionDetection(myData.origin_node, (camera_fb_t *)image_buffer);
    }
  }
}

void addPeer(uint8_t *address)
{
  memcpy(peerInfo.peer_addr, address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // init SD card
  if (pictureMode)
  {
    // camera init
    config_init();
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }
    if (cur_node == 0)
    { // only safe pictures when on the HUB
      SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
      if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5))
      {
        Serial.println("Card Mount Failed");
        return;
      }
      uint8_t cardType = SD_MMC.cardType();
      if (cardType == CARD_NONE)
      {
        Serial.println("No SD_MMC card attached");
        return;
      }
      Serial.print("SD_MMC Card Type: ");
      if (cardType == CARD_MMC)
      {
        Serial.println("MMC");
      }
      else if (cardType == CARD_SD)
      {
        Serial.println("SDSC");
      }
      else if (cardType == CARD_SDHC)
      {
        Serial.println("SDHC");
      }
      else
      {
        Serial.println("UNKNOWN");
      }
      uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
      Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);
      listDir(SD_MMC, "/", 0);
      createDir(SD_MMC, "/mydir");
      listDir(SD_MMC, "/", 0);
      removeDir(SD_MMC, "/mydir");
      listDir(SD_MMC, "/", 2);
      writeFile(SD_MMC, "/hello.txt", "Hello ");
      appendFile(SD_MMC, "/hello.txt", "World!\n");
    }
  }
  //--------------

  // Node 0 needs to notify the client app through BLE
  if (cur_node == 0)
    createBLEServer();

  Serial.print("Number of nodes: ");
  Serial.println(num_nodes);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send and receive callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add peers
  if (cur_node > 0)
  {
    addPeer(nodes[cur_node - 1]); // Add previous node as peer
  }
  if (cur_node < num_nodes - 1)
  {
    addPeer(nodes[cur_node + 1]); // Add next node as peer
  }
}

void loop()
{
  // Set values to send
  // strcpy(myData.msg, "THIS IS A CHAR");
  // myData.origin_node = cur_node;
  // myData.current_node = cur_node;

  // Send message to previous node
  if (digitalRead(PIR_SENSOR_PIN) == HIGH)
  {
    Serial.println("Sensor motion detected...");
    camera_fb_t *fb;
    handleMotionDetection(cur_node, fb);
  }

  delay(2000);
  // Serial.println("Looping...");
}
