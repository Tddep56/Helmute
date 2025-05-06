#include <SD.h>
#include <SPI.h>

void setup() {
    Serial.begin(115200);
    while (!Serial) ; // Wait for Serial to be ready
    delay(1000);

    Serial.println("Initializing SD card...");
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized successfully.");

    File dataFile = SD.open("SecondaryPath.txt");
    if (!dataFile) {
        Serial.println("Could not open SecondaryPath.txt");
        return;
    }

    Serial.println("\nReading first 30 lines from SecondaryPath.txt:\n");
    
    // Read header and first 30 lines
    for (int i = 0; i < 31 && dataFile.available(); i++) {
        String line = dataFile.readStringUntil('\n');
        Serial.print("Line ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(line);
        Serial.println(); // Add blank line for readability
    }

    dataFile.close();
    Serial.println("Finished reading.");
}

void loop() {
    // Nothing to do in loop
}