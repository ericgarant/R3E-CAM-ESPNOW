#include <esp_now.h>
#include <time.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <esp_camera.h>
#include <sd_read_write.h>
#include <SD_MMC.h>
#include <esp_crc.h>
#include <esp_wifi.h>

// Pin Definitions
#define PIR_SENSOR_PIN 33 // PIR sensor pin
#define LED_PIN 32        // LED pin

// Camera Model Pins
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

// SD Card Pins
#define SD_MMC_CMD 15
#define SD_MMC_CLK 14
#define SD_MMC_D0 2

// MAC Addresses for ESP-NOW communication (Replace with your actual MAC addresses)
// #define DEVICE_0 {0x88, 0x13, 0xbf, 0x69, 0xca, 0x68} // no cam
// #define DEVICE_1 {0x0c, 0xb8, 0x15, 0x07, 0x43, 0x7c}
// #define DEVICE_2 {0x1c, 0x69, 0x20, 0xe6, 0x27, 0x80}
// #define DEVICE_3 {0xac, 0x15, 0x18, 0xed, 0x44, 0xcc}
// #define DEVICE_4 {0x88, 0x13, 0xbf, 0x69, 0x9e, 0xfc}
// #define DEVICE_5 {0xcc, 0x7b, 0x5c, 0x97, 0xf0, 0xc8}
// #define DEVICE_6 {0x10, 0x06, 0x1c, 0xd6, 0x46, 0xd8}

// Node Configuration
// uint8_t nodes[][6] = {DEVICE_5, DEVICE_6, DEVICE_1, DEVICE_0}; // List of nodes
//==============CHANGE THESE VALUE FOR EACH DEVICE ===========================
// const int currentNode = 0; // Current node index
// const int totalNodes = 2; // Total number of nodes
// const bool hasCam = true;
//============================================================================
//----
struct Node
{
  int nodeID;
  bool isHub;
  bool hasCam;
  uint8_t currentMAC[6];
  uint8_t previousMAC[6];
  uint8_t nextMACsCount;
  uint8_t nextMACs[2][6];
};
struct R3ENetwork
{
  uint8_t nodeCount;
  Node nodeList[3];
};

Node node0 = {
    .nodeID = 0,
    .isHub = true,
    .hasCam = true,
    .currentMAC = {0xac, 0x15, 0x18, 0xf4, 0x40, 0xd8}, // Device 0
    .previousMAC = {0, 0, 0, 0, 0, 0},                  // Use empty array instead of NULL
    .nextMACsCount = 2,
    .nextMACs = {
        {0x0c, 0xb8, 0x15, 0x07, 0x43, 0x7c}, // Device 1
        {0xac, 0x15, 0x18, 0xf4, 0x77, 0xe4}, // Device 6
    }};

Node node1 = {
    .nodeID = 1,
    .isHub = false,
    .hasCam = true,
    .currentMAC = {0x0c, 0xb8, 0x15, 0x07, 0x43, 0x7c},  // Device 1
    .previousMAC = {0xac, 0x15, 0x18, 0xf4, 0x40, 0xd8}, // Device 0
    .nextMACsCount = 0,
    .nextMACs = {{0}} // Use empty array instead of NULL
};

Node node2 = {
    .nodeID = 2,
    .isHub = false,
    .hasCam = true,
    .currentMAC = {0xac, 0x15, 0x18, 0xf4, 0x77, 0xe4},  // Device 6
    .previousMAC = {0xac, 0x15, 0x18, 0xf4, 0x40, 0xd8}, // Device 0
    .nextMACsCount = 0,
    .nextMACs = {{0}} // Use empty array instead of NULL
};

R3ENetwork _myNetwork = {
    .nodeCount = 3,
    .nodeList = {node0, node1, node2},
};

Node _curNode;

Node FindNode(R3ENetwork network)
{
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  Node _node = {
      .nodeID = -1,
      .isHub = false,
      .hasCam = true,
      .currentMAC = {0},
      .previousMAC = {0},
      .nextMACsCount = 0,
      .nextMACs = {{0}}};
  if (ret == ESP_OK)
  {
    for (int i = 0; i <= network.nodeCount; i++)
    {
      if (baseMac[0] == network.nodeList[i].currentMAC[0] &&
          baseMac[1] == network.nodeList[i].currentMAC[1] &&
          baseMac[2] == network.nodeList[i].currentMAC[2] &&
          baseMac[3] == network.nodeList[i].currentMAC[3] &&
          baseMac[4] == network.nodeList[i].currentMAC[4] &&
          baseMac[5] == network.nodeList[i].currentMAC[5])
      {
        return network.nodeList[i];
      }
    }
    return _node;
  }
  else
  {
    Serial.println("Failed to read MAC address");
    return _node;
  }
}
//----
const int hubNode = 0; // Hub node index
unsigned long lastMessageTime = 0;
const unsigned long messageInterval = 5000; // Interval between messages in milliseconds
#define MAX_IMAGE_SIZE 1000000              // Maximum image size
#define CHUNK_SIZE 200                      // Chunk size to fit within ESP-NOW payload limits (240 bytes)
#define MAX_RETRIES 5

// Data Structure for ESP-NOW messages
struct Message
{
  char text[32];   // Message text
  int originNode;  // Origin node index
  int currentNode; // Current node index
};

struct ImageChunk
{
  uint8_t chunkData[CHUNK_SIZE]; // stay within the 250-byte limit
  uint32_t chunkSize;
  uint32_t chunkIndex;
  uint32_t fileSize;
  uint32_t checksum; // Checksum field
};
// Global variables to manage image sending state
uint8_t *imageBuffer = nullptr; // Pointer to the image buffer
uint32_t imageSize = 0;         // Total image size
uint32_t currentChunkIndex = 0; // Index of the current chunk being sent
uint32_t totalChunks = 0;       // Total number of chunks
uint32_t lastChunkSize = 0;     // Size of the last chunk
camera_fb_t *fb = nullptr;      // Frame buffer to be freed after sending
uint8_t retryCount = 0;         // Number of retries for the current chunk

bool sendingChunk = false;

// Message myData;
esp_now_peer_info_t peerInfo;
NimBLEServer *bleServer = NULL;
NimBLECharacteristic *motionCharacteristic = NULL;
NimBLECharacteristic *healthCheckCharacteristic = NULL;
NimBLECharacteristic *pictureCharacteristic = NULL;
NimBLECharacteristic *pictureReadyCharacteristic = NULL;
bool deviceConnected = false;
bool previousDeviceConnected = false;
uint32_t notificationValue = 0;
int imageCounter = 0;

// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define MOTION_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define HEALTH_CHECK_CHARACTERISTIC_UUID "50c07f71-e239-4f5c-825e-2cc13e914778"
#define PICTURE_CHARACTERISTIC_UUID "0193b6d1-4e1b-745d-ac16-ba9af8fbb405"
#define PICTURE_READY_CHARACTERISTIC_UUID "2f47b012-ba1d-4a90-b28b-49ca6bcf2a8b"

uint8_t *imageRecvBuffer = nullptr;
bool receivingImage = false;
uint32_t totalBytesReceived = 0; // Track total bytes received

void processImageChunk(const ImageChunk &chunk);
void finalizeImageReception(const ImageChunk &chunk);
// void saveImageToSDCard(const uint8_t *buffer, uint32_t fileSize);
void forwardImageToNextDevice();
void processMessage(const Message &myData);
// void sendCapturedImage();
void notifyBLEClient(int node);
void notifyPictureReadyBLEClient(uint32_t fileSize);
void captureAndSendImageTask(void *pvParameters);
void sendImageToBLEServer(const uint8_t *buffer, uint32_t imgSize);

// Camera Configuration
camera_config_t cameraConfig;

uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
  return esp_crc32_le(0, data, length);
}

// Initialize camera configuration
void initializeCameraConfig()
{
  cameraConfig.ledc_channel = LEDC_CHANNEL_0;
  cameraConfig.ledc_timer = LEDC_TIMER_0;
  cameraConfig.pin_d0 = Y2_GPIO_NUM;
  cameraConfig.pin_d1 = Y3_GPIO_NUM;
  cameraConfig.pin_d2 = Y4_GPIO_NUM;
  cameraConfig.pin_d3 = Y5_GPIO_NUM;
  cameraConfig.pin_d4 = Y6_GPIO_NUM;
  cameraConfig.pin_d5 = Y7_GPIO_NUM;
  cameraConfig.pin_d6 = Y8_GPIO_NUM;
  cameraConfig.pin_d7 = Y9_GPIO_NUM;
  cameraConfig.pin_xclk = XCLK_GPIO_NUM;
  cameraConfig.pin_pclk = PCLK_GPIO_NUM;
  cameraConfig.pin_vsync = VSYNC_GPIO_NUM;
  cameraConfig.pin_href = HREF_GPIO_NUM;
  cameraConfig.pin_sccb_sda = SIOD_GPIO_NUM;
  cameraConfig.pin_sccb_scl = SIOC_GPIO_NUM;
  cameraConfig.pin_pwdn = PWDN_GPIO_NUM;
  cameraConfig.pin_reset = RESET_GPIO_NUM;
  cameraConfig.xclk_freq_hz = 20000000;
  cameraConfig.frame_size = FRAMESIZE_SVGA;
  cameraConfig.pixel_format = PIXFORMAT_JPEG;
  cameraConfig.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  cameraConfig.fb_location = CAMERA_FB_IN_PSRAM;
  cameraConfig.jpeg_quality = 5;
  cameraConfig.fb_count = 1; // used to be 2
}
void blink(int count)
{
  for (int i = 0; i < count; i++)
  {
    // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED_PIN, HIGH);
    // wait for a second
    delay(200);
    // turn the LED off by making the voltage LOW
    digitalWrite(LED_PIN, LOW);
    // wait for a second
    delay(200);
  }
}

// List files in a directory
void listDirectory(fs::FS &fs, const char *dirname, uint8_t levels)
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
      Serial.print(" DIR : ");
      Serial.println(file.name());
      if (levels)
      {
        listDirectory(fs, file.path(), levels - 1);
      }
    }
    else
    {
      Serial.print(" FILE: ");
      Serial.print(file.name());
      Serial.print(" SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// Create a directory
void createDirectory(fs::FS &fs, const char *path)
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

// Remove a directory
void removeDirectory(fs::FS &fs, const char *path)
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

// Read a file
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

// Write to a file
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

// Append to a file
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

// Rename a file
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

// Delete a file
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

// Generate a filename for the image
String generateFileName(uint32_t valueToNotify)
{
  String filename = "R3E" + String(valueToNotify) + "_" + String(imageCounter);
  imageCounter++;
  return filename;
}

// BLE server callbacks for connection management
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
void sendDataToPreviousDevice(const Message &receivedData)
{
  Serial.println("sendDataToPreviousDevice ");

  if (_curNode.previousMAC[0] == 0) // no previous node
    return;

  if (esp_now_send(_curNode.previousMAC, (uint8_t *)&receivedData, sizeof(receivedData)) != ESP_OK)
  {
    Serial.println("Error sending the to next device");
  }
  else
    Serial.println("Sent to next device with no errors");
}

void sendDataToNextDevice(const Message &receivedData)
{
  Serial.println("sendDataToNextDevice ");

  //if (_curNode.nextMACs[0][0] == 0) // no next device
  //  return;
  for (int i = 0; i < _curNode.nextMACsCount; i++)
  {
    if (esp_now_send(_curNode.nextMACs[i], (uint8_t *)&receivedData, sizeof(receivedData)) != ESP_OK)
    {
      Serial.println("Error sending the to next device");
    }
    else
      Serial.println("Sent to next device with no errors");
  }
}

class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (!value.empty())
    {

      int targetNode = (int)value[0];
      Serial.println(targetNode);

      if (targetNode == _curNode.nodeID && _curNode.hasCam)
      {
        // we have reached our target
        blink(3);
        //========
        // sendCapturedImage();
        // if (targetNode == currentNode)
        //{
        // Create a task to handle image capture and sending
        xTaskCreatePinnedToCore(captureAndSendImageTask, "CaptureImageTask",
                                4096, NULL, 5, NULL, 0);
        //}
        //==============
      }
      else
      {
        blink(2);

        // Prepare the message
        Message myData;
        myData.currentNode = _curNode.nodeID;
        myData.originNode = targetNode;
        strcpy(myData.text, "HEALTH CHECK");

        // Send the data to the next device
        sendDataToNextDevice(myData);
      }
    }
    else
    {
      Serial.println("Received empty or invalid value");
    }
  }
};

// Create and configure the BLE server
void createBLEServer()
{
  // Initialize the BLE Device
  NimBLEDevice::init("R3E-HUB");

  // Create the BLE Server
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  NimBLEService *service = bleServer->createService(SERVICE_UUID);

  // Create BLE Characteristics
  motionCharacteristic = service->createCharacteristic(
      MOTION_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::NOTIFY);

  healthCheckCharacteristic = service->createCharacteristic(
      HEALTH_CHECK_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  healthCheckCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pictureCharacteristic = service->createCharacteristic(
      PICTURE_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::NOTIFY);

  pictureReadyCharacteristic = service->createCharacteristic(
      PICTURE_READY_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::NOTIFY);

  // Start the service
  service->start();

  // Start advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // Set value to 0x00 to not advertise this parameter
  NimBLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");
}

// Send a file via BLE in chunks
/*void sendFileViaBLE(fs::FS &fs, String path)
{

  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  // File file = fs.open("/hello.txt");

  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  const size_t chunkSize = 200; // Size of each chunk (can adjust depending on BLE MTU)
  uint8_t buffer[chunkSize];
  size_t bytesRead = 0;
  size_t totalBytesSent = 0;
  size_t totalFileSize = file.size(); // Get total file size for progress reporting
  Serial.printf("File Size : %d\n", totalFileSize);

  // notifify server that the file is comming !!!
  notifyPictureReadyBLEClient((uint32_t)totalFileSize);

  while (file.available())
  {
    // Serial.print(".");
    bytesRead = file.read(buffer, chunkSize);           // Read chunk from file
    pictureCharacteristic->setValue(buffer, bytesRead); // Set value to notify
    pictureCharacteristic->notify();                    // Send notification
    // pictureReadyCharacteristic->setValue(buffer, bytesRead); // Set value to notify
    // pictureReadyCharacteristic->notify();                    // Send notification

    totalBytesSent += bytesRead; // Track bytes sent for progress
    float progress = (float)totalBytesSent / totalFileSize * 100.0f;
    Serial.printf("Sending file: %.2f%%\n", progress); // Display progress (optional)

    // delay(1000); // Optional delay to avoid flooding the BLE client
  }
  file.close(); // Close the file when done
  Serial.println("File sent via BLE successfully");
}*/

// Notify the BLE client with the given value
void notifyBLEClient(int node)
{
  Serial.println("Notifying BLE Client...");

  if (deviceConnected)
  {
    Serial.print("Setting the value: ");
    Serial.println(node);

    // Send the originNode as a BLE notification
    motionCharacteristic->setValue(node); // Ensure motionCharacteristic is properly initialized
    motionCharacteristic->notify();       // Notify the connected BLE client
    delay(500);                           // Add delay to avoid flooding the BLE client with notifications
  }

  // Handle case when the device is disconnected and advertising needs to start
  if (!deviceConnected && previousDeviceConnected)
  {
    Serial.println("Start Advertising...");

    delay(500);                    // Small delay before starting advertising (to avoid too rapid retries)
    bleServer->startAdvertising(); // Start advertising to reconnect
    Serial.println("Start advertising");
    previousDeviceConnected = deviceConnected;
  }

  // If the device just connected, update the previousDeviceConnected flag
  if (deviceConnected && !previousDeviceConnected)
  {
    previousDeviceConnected = deviceConnected;
  }

  // Consider removing or shortening this delay to make the loop more responsive
  // delay(5000); // Long delay, may want to reduce or remove for real-time responsiveness
}

void notifyPictureReadyBLEClient(uint32_t fileSize)
{
  Serial.println("Notifying BLE Client that the picture is ready for transfer...");

  if (deviceConnected)
  {
    // Send the originNode as a BLE notification
    pictureReadyCharacteristic->setValue(fileSize);
    pictureReadyCharacteristic->notify();

    delay(10);
  }

  // Handle case when the device is disconnected and advertising needs to start
  if (!deviceConnected && previousDeviceConnected)
  {
    Serial.println("Start Advertising...");

    delay(500);                    // Small delay before starting advertising (to avoid too rapid retries)
    bleServer->startAdvertising(); // Start advertising to reconnect
    Serial.println("Start advertising");
    previousDeviceConnected = deviceConnected;
  }

  // If the device just connected, update the previousDeviceConnected flag
  if (deviceConnected && !previousDeviceConnected)
  {
    previousDeviceConnected = deviceConnected;
  }
}

// Function to send the next chunk
void sendNextChunk()
{
  if (currentChunkIndex > totalChunks)
  {
    sendingChunk = false;
    Serial.println("All chunks sent successfully!");
    if (fb)
    {
      esp_camera_fb_return(fb); // Free the frame buffer
      fb = nullptr;
    }
    return;
  }

  // Prepare the current chunk
  ImageChunk chunk;
  chunk.chunkIndex = currentChunkIndex;
  chunk.chunkSize = (currentChunkIndex == totalChunks) ? lastChunkSize : CHUNK_SIZE;
  chunk.fileSize = imageSize;

  memcpy(chunk.chunkData, imageBuffer + (currentChunkIndex * CHUNK_SIZE), chunk.chunkSize);
  chunk.checksum = calculateCRC32(chunk.chunkData, chunk.chunkSize);

  Serial.printf("Sending chunk %d, size: %d\n", currentChunkIndex, chunk.chunkSize);

  esp_err_t result = esp_now_send(_curNode.previousMAC, (uint8_t *)&chunk, sizeof(ImageChunk));

  if (result != ESP_OK)
  {
    Serial.print("Error sending chunk: ");
    Serial.println(esp_err_to_name(result));
    retryCount++;
    if (retryCount <= MAX_RETRIES)
    {
      Serial.printf("Retrying chunk %d (Attempt %d of %d)\n", currentChunkIndex, retryCount, MAX_RETRIES);
      delay(200); // Small delay before retrying
      sendingChunk = true;
      sendNextChunk(); // Retry sending the current chunk
    }
    else
    {
      Serial.println("Max retries reached. Aborting transmission.");
      if (fb)
      {
        esp_camera_fb_return(fb); // Free the frame buffer on failure
        fb = nullptr;
      }
    }
  }
  else
  {
    retryCount = 0; // Reset retry count on successful send
  }

  // Add delay every 25 chunks
  if (currentChunkIndex % 5 == 0)
  {
    delay(225); // Delay of 500ms every 25 chunks
  }
  else
    delay(75);
}

void captureAndSendImageTask(void *pvParameters)
{
  fb = esp_camera_fb_get(); // there is is bug where this buffer can be from previous capture so as a workarround it is discarded and captured again
  esp_camera_fb_return(fb); // Free the frame buffer after sending all chunks
  fb = NULL;
  fb = esp_camera_fb_get();

  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }
  if (fb->buf == nullptr || fb->len == 0)
  {
    Serial.println("No image data to send or invalid image size.");
    esp_camera_fb_return(fb);
    fb = nullptr;
    return;
  }

  // Serial.printf("Image size: %d\n", fb->len);

  if (_curNode.isHub)
  {
    // Send via BLE to phone
    Serial.printf("Image size: %d bytes\n", fb->len);
    // saveImageToSDCard(fb->buf, fb->len);
    sendImageToBLEServer(fb->buf, fb->len);
    esp_camera_fb_return(fb); // Free the frame buffer after sending all chunks
  }
  else
  {
    imageBuffer = fb->buf;
    imageSize = fb->len;
    totalChunks = imageSize / CHUNK_SIZE;
    lastChunkSize = imageSize % CHUNK_SIZE;
    currentChunkIndex = 0;
    retryCount = 0;

    Serial.printf("Image size: %d bytes\n", imageSize);
    Serial.printf("Total chunks: %d\n", totalChunks);
    Serial.printf("Last chunk size: %d bytes\n", lastChunkSize);

    // Start sending the first chunk
    sendingChunk = true;
    sendNextChunk();
  }
  // sendingChunk = false;

  vTaskDelete(NULL); // Delete the task after completion
}

// ESP-NOW send callback function
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Chunk sent successfully" : "Chunk send failed");
  if (!sendingChunk)
    return;

  if (status == ESP_NOW_SEND_SUCCESS)
  {
    // Move to the next chunk and send it
    currentChunkIndex++;
    sendingChunk = true;
    sendNextChunk();
  }
  else
  {
    // Retry the current chunk if sending failed
    retryCount++;
    if (retryCount <= MAX_RETRIES)
    {
      Serial.printf("Retrying chunk %d (Attempt %d of %d)\n", currentChunkIndex, retryCount, MAX_RETRIES);
      delay(200); // Small delay before retrying
      sendingChunk = true;
      sendNextChunk();
    }
    else
    {
      Serial.println("Max retries reached. Aborting transmission.");
      if (fb)
      {
        esp_camera_fb_return(fb); // Free the frame buffer on failure
        fb = nullptr;
      }
    }
  }
}

// Main onDataReceived function
void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == sizeof(ImageChunk))
  {
    ImageChunk chunk;
    memcpy(&chunk, incomingData, len);
    // Verify the checksum
    uint32_t calculatedChecksum = calculateCRC32(chunk.chunkData, chunk.chunkSize);
    if (calculatedChecksum != chunk.checksum)
    {
      Serial.printf("Checksum mismatch! Received: %u, Calculated: %u\n", chunk.checksum, calculatedChecksum);
      Serial.println("Chunk data corrupted. Requesting resend...");
      return;
    }
    // processImageChunk(chunk);
    if (!_curNode.isHub) // forward the chun directly if not on hub
    {
      sendingChunk = false;
      for (int i = 0; i <= MAX_RETRIES; i++)
      {
        esp_err_t result = esp_now_send(_curNode.previousMAC, (uint8_t *)&chunk, len);
        if (result != ESP_OK)
        {
          Serial.print("Error forwarding chunk: ");
          Serial.println(esp_err_to_name(result));
          Serial.printf("Retrying chunk  (Attempt %d of %d)\n", i, MAX_RETRIES);
          delay(200); // Small delay before retrying
        }
        else
          return;
      }
    }
    else
      processImageChunk(chunk);

    return;
  }

  if (len == sizeof(Message))
  {
    Message myData;
    memcpy(&myData, incomingData, len);
    processMessage(myData);
  }
}

// Helper function to process incoming image chunks
void processImageChunk(const ImageChunk &chunk)
{
  if (!receivingImage)
  {
    receivingImage = true;
    imageRecvBuffer = (uint8_t *)malloc(MAX_IMAGE_SIZE);
    /*uint8_t *imageBuffer = (uint8_t *)heap_caps_malloc(MAX_IMAGE_SIZE, MALLOC_CAP_DMA);
    if (imageBuffer == NULL)
    {
      Serial.println("Failed to allocate memory for imageBuffer");
      // Handle the error gracefully
    }*/
    totalBytesReceived = 0;
  }

  memcpy(imageRecvBuffer + (chunk.chunkIndex * CHUNK_SIZE), chunk.chunkData, chunk.chunkSize);
  totalBytesReceived += chunk.chunkSize;

  if (chunk.chunkSize < CHUNK_SIZE)
  {
    finalizeImageReception(chunk);
  }
}

// Finalize the image reception and decide what to do with the image
void finalizeImageReception(const ImageChunk &chunk)
{
  Serial.printf("File received complete: %d\n", totalBytesReceived);
  receivingImage = false;

  if (_curNode.isHub)
  {
    // Send image via BLE to phone
    Serial.println("Forwarding image to BLE");
    Serial.printf("Bytes received: %d; expected: %d\n", totalBytesReceived, chunk.fileSize);
    // saveImageToSDCard(imageRecvBuffer, chunk.fileSize);
    sendImageToBLEServer(imageRecvBuffer, chunk.fileSize);
  }
  else
  {
    // Forward image to the next device
    forwardImageToNextDevice();
  }

  // Free memory after processing
  if (imageRecvBuffer != nullptr)
  {
    free(imageRecvBuffer);
    imageRecvBuffer = nullptr;
    Serial.println("Memory allocation for imageRecvBuffer is now free!");
  }
}

void sendImageToBLEServer(const uint8_t *buffer, uint32_t imgSize)
{
  const size_t chunkSize = 200; // Size of each chunk (can adjust depending on BLE MTU)
  size_t bytesRead = 0;
  size_t totalBytesSent = 0;
  size_t chkCnt = imgSize / chunkSize;
  size_t lstChkSize = imgSize % chunkSize;
  Serial.printf("Image Size : %d\n", imgSize);
  // Notify server that the file is coming
  notifyPictureReadyBLEClient(imgSize);
  for (size_t i = 0; i < chkCnt; i++)
  {
    // Copy chunk from the buffer
    pictureCharacteristic->setValue(buffer + totalBytesSent, chunkSize); // Set value to notify
    pictureCharacteristic->notify();                                     // Send notification
    totalBytesSent += chunkSize;                                         // Track bytes sent for progress
    float progress = (float)totalBytesSent / imgSize * 100.0f;
    Serial.printf("Sending file: %.2f%%\n", progress); // Display progress (optional)
    delay(20);                                         // Optional delay to avoid flooding the BLE client
  } // Send the last chunk if it exists
  if (lstChkSize > 0)
  {
    pictureCharacteristic->setValue(buffer + totalBytesSent, lstChkSize); // Set value to notify
    pictureCharacteristic->notify();                                      // Send notification
    totalBytesSent += lstChkSize;                                         // Track bytes sent for progress
    float progress = (float)totalBytesSent / imgSize * 100.0f;
    Serial.printf("Sending file: %.2f%%\n", progress); // Display progress (optional)
  }
  Serial.println("Image transfer complete.");
}

// Save the received image to the SD card
/*void saveImageToSDCard(const uint8_t *buffer, uint32_t fileSize)
{
  if (buffer == nullptr || fileSize == 0)
  {
    Serial.println("No image data to save or invalid image size.");
    return;
  }

  String path = "/" + generateFileName(999) + ".jpg";
  Serial.println("Saving to path: " + path);

  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing.");
    return;
  }

  size_t bytesWritten = file.write(buffer, fileSize);
  if (bytesWritten != fileSize)
  {
    Serial.println("Failed to write the entire image to the SD card.");
  }
  else
  {
    Serial.println("Image saved to SD card successfully.");
  }
  file.close();
  delay(2000);
  sendFileViaBLE(SD_MMC, path);
}*/

// Forward the received image to the next device
void forwardImageToNextDevice()
{
  Serial.println("Forwarding image to next device");

  imageBuffer = imageRecvBuffer;
  imageSize = totalBytesReceived;
  totalChunks = imageSize / CHUNK_SIZE;
  lastChunkSize = imageSize % CHUNK_SIZE;
  currentChunkIndex = 0;
  retryCount = 0;

  Serial.printf("Bytes received: %d\n", imageSize);
  Serial.printf("Fwd total chunks: %d\n", totalChunks);
  Serial.printf("Fwd last chunk size: %d bytes\n", lastChunkSize);

  sendingChunk = true;
  sendNextChunk();
}

void processMessage(const Message &myData)
{
  Serial.println("Processing Message...");

  if (strcmp(myData.text, "MOTION DETECTED") == 0)
  {
    if (_curNode.isHub)
    {
      notifyBLEClient(myData.originNode);
    }
    else
    {
      sendDataToPreviousDevice(myData);
    }
  }
  else if (strcmp(myData.text, "HEALTH CHECK") == 0)
  {
    Serial.println("Handling HC");

    if (myData.originNode == _curNode.nodeID && _curNode.hasCam)
    {
      blink(3);
      // sendCapturedImage();
      //  Create a task to handle image capture and sending
      xTaskCreatePinnedToCore(captureAndSendImageTask, "CaptureImageTask",
                              4096, NULL, 5, NULL, 0);
    }
    else
    {
      blink(2);
      sendDataToNextDevice(myData);
    }
  }
}

// Add a peer for ESP-NOW communication
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

// Setup function
void setup()
{
  Serial.begin(9600);
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  //=== init ESP NOW ===
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //---
  _curNode = FindNode(_myNetwork);
  if (_curNode.nodeID < 0)
    return;
  else
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  _curNode.currentMAC[0], _curNode.currentMAC[1], _curNode.currentMAC[2],
                  _curNode.currentMAC[3], _curNode.currentMAC[4], _curNode.currentMAC[5]);
  Serial.println(_curNode.isHub);
  //---

  // Register send and receive callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);

  // Add peers
  if (!_curNode.isHub)
    addPeer(_curNode.previousMAC); // Add previous node as peer
  for (int i = 0; i < _curNode.nextMACsCount; i++)
  {
    addPeer(_curNode.nextMACs[i]); // Add next node as peer
  }
  //====================

  if (_curNode.hasCam)
  {
    initializeCameraConfig();
    esp_err_t err = esp_camera_init(&cameraConfig);
    if (err != ESP_OK)
    {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }
    // fb = esp_camera_fb_get(); // there is is bug where this buffer can be from previous capture so as a workarround it is discarded and captured again
    esp_camera_fb_return(fb); // Free the frame buffer after sending all chunks
    fb = NULL;
  }

  if (false) // keep for futur use (SD Card)
  {

    if (_curNode.isHub)
    {
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
      listDirectory(SD_MMC, "/", 0);
      createDirectory(SD_MMC, "/mydir");
      listDirectory(SD_MMC, "/", 0);
      removeDirectory(SD_MMC, "/mydir");
      listDirectory(SD_MMC, "/", 2);
      writeFile(SD_MMC, "/hello.txt", "Hello ");
      appendFile(SD_MMC, "/hello.txt", "World! and the univers also! hope every living creates are happy and fully realise there development and exploring potial.  Lets all meet soon and share knowledge!!!\n");
      readFile(SD_MMC, "/hello.txt");
    }
  }

  // Node 0 needs to notify the client app through BLE
  if (_curNode.isHub)
    createBLEServer();
}

void loop()
{
  if (digitalRead(PIR_SENSOR_PIN) == HIGH && !sendingChunk)
  // if (digitalRead(PIR_SENSOR_PIN) == HIGH)

  {
    unsigned long currentTime = millis();
    if (currentTime - lastMessageTime > messageInterval)
    {
      lastMessageTime = currentTime;
      Serial.println("Sensor motion detected...begin loop");
      blink(1);
      if (_curNode.isHub)
      {
        notifyBLEClient(_curNode.nodeID);
      }
      else
      {
        Message myData;
        strcpy(myData.text, "MOTION DETECTED");
        myData.currentNode = _curNode.nodeID;
        myData.originNode = _curNode.nodeID;
        sendDataToPreviousDevice(myData);
      };
      delay(5000);
      Serial.println("Sensor motion detected...end of loop");
    }
  }
  delay(1000);
}