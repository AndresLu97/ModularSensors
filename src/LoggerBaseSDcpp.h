#ifndef SRC_LOGGERBASESDCPP_H
#define SRC_LOGGERBASESDCPP_H

// ===================================================================== //
// Public functions extensions for Class Logger
// ===================================================================== //

void printDirectory(File dir, int numTabs) {    //This is a function that displays all the files on the SD card to the serial monitor.
    //Long File Names can be up to 255 characters, define how much is acceptable
    #define MS_LIST_MAX_FN_SZ 31
    char fn_nxt_entry[MS_LIST_MAX_FN_SZ ];
    #define MS_MAX_NUM_FILES_TO_LIST  20
    uint8_t max_num_files=0;
    while (true) 
    {
        if (MS_MAX_NUM_FILES_TO_LIST < ++max_num_files) {
            PRINTOUT("Stopped listing at filenumber",MS_MAX_NUM_FILES_TO_LIST );
        }
        File entry =  dir.openNextFile();
        if (! entry) {
            // no more files
            //Serial.println("No more files");
            break;
        }
        for (uint8_t i = 0; i < numTabs; i++) {
            Serial.print('\t');
        }
        memset(fn_nxt_entry,0,MS_LIST_MAX_FN_SZ);
        if (entry.getName(fn_nxt_entry,MS_LIST_MAX_FN_SZ )) {
            Serial.print(fn_nxt_entry);
            if (entry.isDirectory()) {
                Serial.println("/");
                printDirectory(entry, numTabs + 1);
            } else {
                // files have sizes, directories do not
                Serial.print("\t\t");
                Serial.println((unsigned long)entry.size(), DEC);
            }
        } else {
            Serial.println("Failed to get filename");
        }
        entry.close();
    } // While
}

void Logger::SD1_ListDir() {
    File rootSD =  sd1_card_fatfs.open("/");
    Serial.println("Files on microSD card");
    printDirectory(rootSD, 0);
    Serial.println("");
}

void Logger::SD1_ListReadings() {
    MS_DBG("SD1 ListReadings");
    if (_fileName == "") 
    {
        Serial.println("SD1_ListReadings no file"); 
        return;
    }

    // Convert the string filename to a character file name for SdFat
    uint8_t fileNameLength = _fileName.length() + 1;
    char    charFileName[fileNameLength];
    _fileName.toCharArray(charFileName, fileNameLength);
    if (logFile.open(charFileName, O_RDONLY)) 
    {
        MS_DBG(F("Opened existing file:"), _fileName);
        //Need to have watchdog not print while listing
        while (logFile.available()) {
            Serial.write(logFile.read());
            delay(1); //Pace the output
        }
        logFile.close();
        Serial.println(); 
    } else {
        MS_DBG(F("Unable to to find file:"), _fileName);
    }
}

#endif  //SRC_LOGGERBASESDCPP_H