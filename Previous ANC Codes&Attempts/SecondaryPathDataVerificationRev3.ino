#include <SD.h>
#include <SPI.h>

#define SDCARD_CS_PIN    BUILTIN_SDCARD

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect
    }

    Serial.println("Initializing SD card...");
    
    if (!SD.begin(SDCARD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialized successfully.\n");
    
    // Read and display the transfer function data
    File dataFile = SD.open("SecondaryPath.txt");
    if (dataFile) {
        Serial.println("Contents of SecondaryPath.txt:");
        Serial.println("--------------------------------");
        
        // Read header
        String header = dataFile.readStringUntil('\n');
        Serial.println(header);
        
        // Read and parse each line
        while (dataFile.available()) {
            String line = dataFile.readStringUntil('\n');
            
            // Parse tab-separated values
            int tab1 = line.indexOf('\t');
            int tab2 = line.indexOf('\t', tab1 + 1);
            int tab3 = line.indexOf('\t', tab2 + 1);
            int tab4 = line.indexOf('\t', tab3 + 1);
            
            if (tab1 > 0 && tab2 > 0 && tab3 > 0 && tab4 > 0) {
                int bin = line.substring(0, tab1).toInt();
                float freq = line.substring(tab1 + 1, tab2).toFloat();
                float mag = line.substring(tab2 + 1, tab3).toFloat();
                float phase = line.substring(tab3 + 1, tab4).toFloat();
                float unwrappedPhase = line.substring(tab4 + 1).toFloat();
                
                // Print formatted output
                Serial.print("Bin ");
                Serial.print(bin);
                Serial.print(" (");
                Serial.print(freq, 1);
                Serial.print(" Hz): Magnitude = ");
                Serial.print(mag, 6);
                Serial.print(", Phase = ");
                Serial.print(phase, 6);
                Serial.print(" rad, Unwrapped Phase = ");
                Serial.print(unwrappedPhase, 6);
                Serial.println(" rad");
            }
        }
        
        dataFile.close();
        Serial.println("--------------------------------");
        Serial.println("File read complete.");
    } else {
        Serial.println("Error opening SecondaryPath.txt");
    }
}

void loop() {
    // Nothing to do in loop
}