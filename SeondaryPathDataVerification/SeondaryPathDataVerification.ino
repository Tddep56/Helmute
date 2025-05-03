#include <SPI.h>
#include <SD.h>

#define SDCARD_CS_PIN BUILTIN_SDCARD

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {} // Wait for serial connection

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDCARD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialization done.");

  // Open the file
  File dataFile = SD.open("SecondaryPath.txt");
  if (dataFile) {
    Serial.println("Reading first 34 indexes from SecondaryPath.txt:");

    int index = 0; // Index to track lines read
    while (dataFile.available() && index < 34) {
      String line = dataFile.readStringUntil('\n'); // Read each line
      float value = line.toFloat(); // Convert to a float

      // Print the value
      Serial.print("Index ");
      Serial.print(index);
      Serial.print(": ");
      Serial.println(value, 6); // Print 

      index++;
    }
    dataFile.close();

    Serial.println("Finished reading first 32 indexes.");
  } else {
    Serial.println("Error opening SecondaryPath.txt for reading!");
  }
}

void loop() {
  // Can be left empty
}

