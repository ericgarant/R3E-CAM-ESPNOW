#include <esp_now.h>
#include <time.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <esp_camera.h>
#include <sd_read_write.h>
#include <SD_MMC.h>

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

// MAC Addresses for ESP-NOW communication
#define DEVICE_0 {0x88, 0x13, 0xbf, 0x69, 0xca, 0x68}
#define DEVICE_1 {0x0c, 0xb8, 0x15, 0x07, 0x43, 0x7c}
#define DEVICE_2 {0x1c, 0x69, 0x20, 0xe6, 0x27, 0x80}
#define DEVICE_3 {0xac, 0x15, 0x18, 0xed, 0x44, 0xcc}
#define DEVICE_4 {0x88, 0x13, 0xbf, 0x69, 0x9e, 0xfc}
#define DEVICE_5 {0xcc, 0x7b, 0x5c, 0x97, 0xf0, 0xc8}
#define DEVICE_6 {0x10, 0x06, 0x1c, 0xd6, 0x46, 0xd8}

// Node Configuration
uint8_t nodes[][6] = {DEVICE_5, DEVICE_6, DEVICE_1}; // List of nodes
const int currentNode = 1;                           // Current node index
const int totalNodes = 3;                            // Total number of nodes
const int hubNode = 0;                               // Hub node index
const bool enablePictureMode = true;                 // Enable picture mode
unsigned long lastMessageTime = 0;
const unsigned long messageInterval = 5000; // Interval between messages in milliseconds
#define MAX_IMAGE_SIZE 1000000              // Maximum image size
#define MAX_IMAGE_CHUNK_SIZE 220            // Chunk size to fit within ESP-NOW payload limits (240 bytes)

// Data Structure for ESP-NOW messages
struct Message
{
  char text[32];   // Message text
  int originNode;  // Origin node index
  int currentNode; // Current node index

  uint8_t *imageData; // Pointer to image data
  uint32_t imageSize; // Size of the image data

  // Constructor
  Message()
  {
    imageData = (uint8_t *)malloc(MAX_IMAGE_SIZE);
    if (imageData == NULL)
    {
      Serial.println("Memory allocation failed for imageData");
    }
    else
    {
      memset(imageData, 0, MAX_IMAGE_SIZE); // Initialize memory to zero
    }
  }

  // Destructor
  ~Message()
  {
    if (imageData != NULL)
    {
      free(imageData);
      imageData = NULL;
      Serial.println("Memory allocation is now free!");
    }
  }
};
struct MessageHeader
{
  char text[32];
  int originNode;
  int currentNode;
  uint32_t imageSize;
};

struct ImageChunk
{
  uint8_t chunkData[MAX_IMAGE_CHUNK_SIZE]; // stay within the 250-byte limit
  uint32_t chunkSize;
  uint32_t chunkIndex;
};

// Message myData;
esp_now_peer_info_t peerInfo;
NimBLEServer *bleServer = NULL;
NimBLECharacteristic *motionCharacteristic = NULL;
NimBLECharacteristic *healthCheckCharacteristic = NULL;
NimBLECharacteristic *pictureCharacteristic = NULL;
bool deviceConnected = false;
bool previousDeviceConnected = false;
uint32_t notificationValue = 0;
int imageCounter = 0;

// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define MOTION_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define HEALTH_CHECK_CHARACTERISTIC_UUID "50c07f71-e239-4f5c-825e-2cc13e914778"
#define PICTURE_CHARACTERISTIC_UUID "0193b6d1-4e1b-745d-ac16-ba9af8fbb405"

// Camera Configuration
camera_config_t cameraConfig;

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
  cameraConfig.fb_count = 2;
}
void blink(int count)
{
  for (int i = 0; i < count; i++)
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

// Save the captured image to the SD card
void saveImageToSD(Message &myData)
{
  // Ensure SD card is initialized
  if (!SD_MMC.begin())
  {
    Serial.println("SD card mount failed.");
    return; // Exit if SD card is not mounted
  }

  // Check if the imageData pointer is valid
  if (myData.imageData == nullptr || myData.imageSize == 0)
  {
    Serial.println("No image data to save or invalid image size.");
    return;
  }

  // Generate a file name based on the origin node
  String path = "/" + generateFileName(myData.originNode) + ".jpg";
  Serial.println("Saving to path: " + path);

  // Open the file for writing
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing.");
    return;
  }

  // Write the image data to the file
  size_t bytesWritten = file.write(myData.imageData, myData.imageSize);
  if (bytesWritten != myData.imageSize)
  {
    Serial.println("Failed to write the entire image to the SD card.");
    file.close();
    return;
  }

  // Close the file after writing
  file.close();
  Serial.println("Image saved to SD card successfully.");
}

// Capture an image using the camera
void captureImage(Message &myData)
{
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }

  // Log the frame buffer pointer and its size for debugging
  Serial.println("Frame buffer address: " + String((uintptr_t)fb));
  Serial.println("Frame buffer size: " + String(fb->len));

  // Ensure the image size does not exceed the maximum allowed size
  if (fb->len > MAX_IMAGE_SIZE)
  {
    Serial.println("Captured image size exceeds maximum allowed size");
    esp_camera_fb_return(fb); // Return the frame buffer before returning from function
    return;
  }

  // If imageData is not already allocated or is too small, allocate memory
  if (myData.imageData == NULL)
  {
    // Allocate the memory for image data
    myData.imageData = (uint8_t *)malloc(MAX_IMAGE_SIZE);
    if (myData.imageData == NULL)
    {
      Serial.println("Memory allocation failed for imageData");
      esp_camera_fb_return(fb); // Return the frame buffer
      return;
    }
  }
  else
  {
    // Clear any previous data if the buffer was already allocated
    memset(myData.imageData, 0, MAX_IMAGE_SIZE);
  }

  // Copy the image data into the struct
  Serial.println("Saving picture in memory...");
  memcpy(myData.imageData, fb->buf, fb->len); // Copy the image data to the struct
  myData.imageSize = fb->len;                 // Set the image size

  // Free the frame buffer after use
  esp_camera_fb_return(fb);

  // Log image size
  Serial.println("Picture taken...");
  Serial.println("Image size: " + String(fb->len));
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

void sendHeader(const uint8_t *peerAddress, MessageHeader &header)
{
  if (esp_now_send(peerAddress, (uint8_t *)&header, sizeof(header)) != ESP_OK)
  {
    Serial.println("Error sending the header");
  }
}

void sendImageChunks(const uint8_t *peerAddress, uint8_t *imageData, uint32_t imageSize)
{
  if (imageSize == 0)
    return;
  uint32_t chunks = (imageSize + MAX_IMAGE_CHUNK_SIZE - 1) / MAX_IMAGE_CHUNK_SIZE; // Calculate the number of chunks
  Serial.printf("Chunkse: %d\n", chunks);

  for (uint32_t i = 0; i < chunks; i++)
  {
    ImageChunk chunk;
    chunk.chunkSize = (i == chunks - 1) ? (imageSize % MAX_IMAGE_CHUNK_SIZE) : MAX_IMAGE_CHUNK_SIZE;
    chunk.chunkIndex = i;
    //Serial.printf("chunkSize: %d\n", chunk.chunkSize);
    //Serial.printf("chunkIndex: %d\n", chunk.chunkIndex);

    memcpy(chunk.chunkData, imageData + (i * MAX_IMAGE_CHUNK_SIZE), chunk.chunkSize);

    /*if (esp_now_send(peerAddress, (uint8_t *)&chunk, sizeof(chunk)) != ESP_OK)
    {
      Serial.println("Error sending image chunk");
    }
    delay(100); // Small delay to ensure proper transmission*/
    esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&chunk, sizeof(chunk));

    if (result != ESP_OK)
    {
      Serial.print("Error sending image chunk: ");
      Serial.println(esp_err_to_name(result));
      delay(500); // Longer delay to allow the queue to clear
    }
    else
    {
      //Serial.printf("Chunk %d sent successfully\n", chunk.chunkIndex);
    }

    delay(100); // Adjust delay for stability
  }
}

void sendDataToNextDevice(int8_t nextDeviceIndex, Message &receivedData)
{
  if (currentNode + nextDeviceIndex < 0 || currentNode + nextDeviceIndex >= totalNodes)
    return;
  MessageHeader header;
  strcpy(header.text, receivedData.text);
  header.originNode = receivedData.originNode;
  header.currentNode = currentNode;
  header.imageSize = receivedData.imageSize; // Example image size

  //Serial.printf("Sending header: %s\n", header.text);
  //Serial.printf("Sending img size: %d\n", header.imageSize);

  sendHeader(nodes[currentNode + nextDeviceIndex], header);

  // uint8_t imageData[1024]; // Example image data
  sendImageChunks(nodes[currentNode + nextDeviceIndex], receivedData.imageData, header.imageSize);
}

// BLE characteristic callbacks for handling writes
class MyCharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    Serial.print("Value written: ");
    Serial.println(value.c_str());
    blink(3);

    Message myData;
    myData.currentNode = currentNode;
    myData.originNode = currentNode;
    // myData.imageData=null;
    strcpy(myData.text, "HEALTH CHECK");
    myData.imageSize = 0;

    sendDataToNextDevice(1, myData);
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
void sendFileViaBLE(fs::FS &fs, String path)
{
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  const size_t chunkSize = 500; // Size of each chunk (can adjust depending on BLE MTU)
  uint8_t buffer[chunkSize];
  size_t bytesRead = 0;
  size_t totalBytesSent = 0;
  size_t totalFileSize = file.size(); // Get total file size for progress reporting

  // Read and send file in chunks
  while (file.available())
  {
    bytesRead = file.read(buffer, chunkSize);           // Read chunk from file
    pictureCharacteristic->setValue(buffer, bytesRead); // Set value to notify
    pictureCharacteristic->notify();                    // Send notification

    totalBytesSent += bytesRead; // Track bytes sent for progress
    float progress = (float)totalBytesSent / totalFileSize * 100.0f;
    Serial.printf("Sending file: %.2f%%\n", progress); // Display progress (optional)

    delay(50); // Optional delay to avoid flooding the BLE client
  }

  file.close(); // Close the file when done
  Serial.println("File sent via BLE successfully");
}

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
    delay(1000);                          // Add delay to avoid flooding the BLE client with notifications

    // If picture mode is enabled, send the image file to the BLE client
    /*
    if (enablePictureMode)
    {
      String filename = "R3E" + String(currentNode) + "_" + String(imageCounter - 1);
      String path = "/" + filename + ".jpg"; // Path to the image on the SD card
      Serial.print("Sending file: " + path);

      // Ensure this function is non-blocking and does not stall the loop
      sendFileViaBLE(SD_MMC, path); // Function to send file via BLE
    }*/
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
  delay(5000); // Long delay, may want to reduce or remove for real-time responsiveness
}

void sendCapturedImage()
{
  MessageHeader header;
  strcpy(header.text, "MOTION DETECTED");
  header.originNode = currentNode;
  header.currentNode = currentNode;
  if (!enablePictureMode)
  {
    Serial.println("Sending with no image");
    header.imageSize = 0; // Example image size
    sendHeader(nodes[currentNode - 1], header);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }

  Serial.println("Picture taken, sending via ESP-NOW...");

  header.imageSize = fb->len; // Example image size

  sendHeader(nodes[currentNode - 1], header);

  // uint8_t imageData[1024]; // Example image data
  sendImageChunks(nodes[currentNode - 1], fb->buf, header.imageSize);

  esp_camera_fb_return(fb);
  Serial.println("Image sent via ESP-NOW successfully");
}

void saveCapturedImage(const int node)

{ // Capture the picture
  if (!enablePictureMode)
    return;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }
  Serial.println("Picture taken, saving on SD card..");
  // Ensure SD card is initialized
  if (!SD_MMC.begin())
  {
    Serial.println("SD card mount failed.");
    return; // Exit if SD card is not mounted
  }

  // Check if the imageData pointer is valid
  if (fb->buf == nullptr || fb->len == 0)
  {
    Serial.println("No image data to save or invalid image size.");
    return;
  }

  // Generate a file name based on the origin node
  String path = "/" + generateFileName(node) + ".jpg";
  Serial.println("Saving to path: " + path);

  // Open the file for writing
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing.");
    return;
  }

  // Write the image data to the file
  size_t bytesWritten = file.write(fb->buf, fb->len);
  if (bytesWritten != fb->len)
  {
    Serial.println("Failed to write the entire image to the SD card.");
    file.close();
    return;
  }

  // Close the file after writing
  file.close();
  Serial.println("Image saved to SD card successfully.");
}

// Callback function for ESP-NOW data sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//-----
MessageHeader receivedHeader;
uint8_t *receivedImageData = nullptr;
uint32_t receivedImageSize = 0;
uint32_t receivedChunks = 0;

void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == sizeof(MessageHeader))
  {
    memcpy(&receivedHeader, incomingData, sizeof(MessageHeader));
    receivedImageSize = receivedHeader.imageSize;
    receivedImageData = (uint8_t *)malloc(receivedImageSize);
    receivedChunks = 0;
    Serial.print("Header received: ");
    Serial.println(receivedHeader.text);
  }
  else if (len == sizeof(ImageChunk))
  {
    ImageChunk chunk;
    memcpy(&chunk, incomingData, sizeof(chunk));
    memcpy(receivedImageData + (chunk.chunkIndex * MAX_IMAGE_CHUNK_SIZE), chunk.chunkData, chunk.chunkSize);
    receivedChunks++;
    //Serial.print("Chunk received: ");
    //Serial.println(chunk.chunkIndex);

    // Check if all chunks are received
    if ((chunk.chunkIndex + 1) * MAX_IMAGE_CHUNK_SIZE >= receivedImageSize)
    {
      Serial.println("All data received");
      // Process the complete image data
      Message myData;
      myData.currentNode = receivedHeader.currentNode;
      myData.originNode = receivedHeader.originNode;
      // myData.text = receivedHeader.text;
      strncpy(myData.text, receivedHeader.text, sizeof(receivedHeader.text));
      myData.imageSize = receivedImageSize;
      myData.imageData = receivedImageData;

      if (strcmp(myData.text, "MOTION DETECTED") == 0)
      {
        if (currentNode == hubNode)
        {
          saveImageToSD(myData);
          notifyBLEClient(myData.originNode);
        }
        else
          sendDataToNextDevice(-1, myData);
      }
    }
  }
  if (strcmp(receivedHeader.text, "HEALTH CHECK") == 0)
  {
    Serial.println("Handling HC");
    blink(3);
    Message myData;
    myData.currentNode = receivedHeader.currentNode;
    myData.originNode = receivedHeader.originNode;
    // myData.text = receivedHeader.text;
    strncpy(myData.text, receivedHeader.text, sizeof(receivedHeader.text));
    myData.imageSize = 0;
    sendDataToNextDevice(1, myData);
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

  if (enablePictureMode)
  {
    initializeCameraConfig();
    esp_err_t err = esp_camera_init(&cameraConfig);
    if (err != ESP_OK)
    {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }
    if (currentNode == hubNode)
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
      appendFile(SD_MMC, "/hello.txt", "World!\n");
    }
  }

  // Node 0 needs to notify the client app through BLE
  if (currentNode == hubNode)
    createBLEServer();

  Serial.print("Number of nodes: ");
  Serial.println(totalNodes);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send and receive callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);

  // Add peers
  if (currentNode > hubNode)
  {
    addPeer(nodes[currentNode - 1]); // Add previous node as peer
  }
  if (currentNode < totalNodes - 1)
  {
    addPeer(nodes[currentNode + 1]); // Add next node as peer
  }
}

void loop()
{
  if (digitalRead(PIR_SENSOR_PIN) == HIGH)
  {
    unsigned long currentTime = millis();
    if (currentTime - lastMessageTime > messageInterval)
    {
      lastMessageTime = currentTime;
      Serial.println("Sensor motion detected...begin loop");
      blink(1);
      // digitalWrite(LED_PIN, HIGH);
      // delay(1000);
      // digitalWrite(LED_PIN, LOW);
      if (currentNode == hubNode)
      {

        saveCapturedImage(currentNode);
        notifyBLEClient(currentNode);
      }
      else
        sendCapturedImage();
      /*Message myData; // Constructor automatically called here
      myData.currentNode = currentNode;
      handleMotionDetection(myData);
      Serial.print("Free heap: ");
      Serial.println(ESP.getFreeHeap());*/

      Serial.println("Sensor motion detected...end of loop");
    }
  }
  delay(2000);
}
