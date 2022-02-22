//
// Private function extensions
//
#define TEMP_BUFFER_SZ 37

#if defined USE_USB_MSC_SD1
//--------------------------------------------------------------------+
// SD Card callbacks
//--------------------------------------------------------------------+

int32_t Logger::sd1_card_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
    return Logger::sd1_card_phy.card()->readBlocks(lba, (uint8_t*)buffer,
                                                   bufsize / 512)
        ? bufsize
        : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t Logger::sd1_card_write_cb(uint32_t lba, uint8_t* buffer,
                                  uint32_t bufsize) {
    // digitalWrite(LED_BUILTIN, HIGH);

    return sd1_card_phy.card()->writeBlocks(lba, buffer, bufsize / 512)
        ? bufsize
        : -1;
}

// Callback invoked when WRITE10 command is completed (status received and
// accepted by host). used to flush any pending cache.
void Logger::sd1_card_flush_cb(void) {
    sd1_card_phy.card()->syncBlocks();

    // clear file system's cache to force refresh
    sd1_card_phy.cacheClear();

    sd1_card_changed = true;

    // digitalWrite(LED_BUILTIN, LOW);
}
#endif  // USE_USB_MSC_SD1
#if defined USE_USB_MSC_SD0
//--------------------------------------------------------------------+
// External Flash callbacks
//--------------------------------------------------------------------+

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t Logger::sdq_flashspi_read_cb(uint32_t lba, void* buffer,
                                     uint32_t bufsize) {
    // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
    // already include 4K sector caching internally. We don't need to cache it,
    // yahhhh!!
    return sdq_flashspi_phy.readBlocks(lba, (uint8_t*)buffer, bufsize / 512)
        ? bufsize
        : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t Logger::sdq_flashspi_write_cb(uint32_t lba, uint8_t* buffer,
                                      uint32_t bufsize) {
    // digitalWrite(LED_BUILTIN, HIGH);

    // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
    // already include 4K sector caching internally. We don't need to cache it,
    // yahhhh!!
    return sdq_flashspi_phy.writeBlocks(lba, buffer, bufsize / 512) ? bufsize
                                                                    : -1;
}

// Callback invoked when WRITE10 command is completed (status received and
// accepted by host). used to flush any pending cache.
void Logger::sdq_flashspi_flush_cb(void) {
    sdq_flashspi_phy.syncBlocks();

    // clear file system's cache to force refresh
    sd0_card_fatfs.cacheClear();

    sd0_card_changed = true;

    // digitalWrite(LED_BUILTIN, LOW);
}

// Callback invoked when USB system checking if available
bool Logger::sdq_ready(void) {
    PRINTOUT("USB check for sdq");
    usbDriveStatus = true;
    return true;  // unless sleeping in which case false
}
bool Logger::usbDriveActive(void) {
    bool retVal    = usbDriveStatus;
    usbDriveStatus = false;
    return retVal;
}

void print_rootdir(File* rdir);
void print_rootdir(File* rdir) {
    File file;

    // Open next file in root.
    // Warning, openNext starts at the current directory position
    // so a rewind of the directory may be required.
    while (file.openNext(rdir, O_RDONLY)) {
        file.printFileSize(&STANDARD_SERIAL_OUTPUT);
        // PRINTOUT(" ");
        STANDARD_SERIAL_OUTPUT.print(' ');
        file.printName(&STANDARD_SERIAL_OUTPUT);
        if (file.isDir()) {
            // Indicate a directory.
            // PRINTOUT('/');
            STANDARD_SERIAL_OUTPUT.print('/');
        }
        // PRINTOUT('\n');
        STANDARD_SERIAL_OUTPUT.print('\n');
        file.close();
    }
}  // print_rootdir()
#endif  // USE_USB_MSC
bool Logger::SDextendedInit(bool sd1Success) {
    bool retVal = sd1Success;

#if defined  BOARD_SDQ_QSPI_FLASH
#if !defined USE_TINYUSB
#error need -DUSE_TINYUSB
#endif  //
    usbDriveStatus = false;
    // If defined need to initiliaze else turns off ints
    usb_msc.setMaxLun(2);

    usb_msc.setID(0, "Adafruit", "External Flash", "1.0");
    usb_msc.setID(1, "Adafruit", "SD Card", "1.0");

    // Since initialize both external flash and SD card can take time.
    // If it takes too long, our board could be enumerated as CDC device only
    // i.e without Mass Storage. To prevent this, we call Mass Storage begin
    // first LUN readiness will always be set later on
    usb_msc.begin();

    //------------- Lun 0 for external flash -------------//
    sdq_flashspi_phy
        .begin();  // should have assigned &sdq_flashspi_transport_QSPI);
    sd0_card_fatfs.begin(&sdq_flashspi_phy);
    MS_DBG(F("Successfully setup SD0"));

#if defined USE_USB_MSC_SD0
    usb_msc.setCapacity(
        0, sdq_flashspi_phy.pageSize() * sdq_flashspi_phy.numPages() / 512,
        512);
    usb_msc.setReadWriteCallback(0, sdq_flashspi_read_cb, sdq_flashspi_write_cb,
                                 sdq_flashspi_flush_cb);
    usb_msc.setReadyCallback(0, sdq_ready);
    usb_msc.setUnitReady(0, true);

    sd0_card_changed = true;  // to print contents initially
    MS_DBG(F("SD0 Supported on USB"));
#if defined USE_USB_MSC_SD1
    //------------- Lun 1 for SD card -------------//
    if (sd1Success) {
        uint32_t block_count = sd1_card_phy.card()->cardSize();
        usb_msc.setCapacity(1, block_count, 512);
        usb_msc.setReadWriteCallback(1, sd1_card_read_cb, sd1_card_write_cb,
                                     sd1_card_flush_cb);
        usb_msc.setUnitReady(1, true);

        sd1_card_changed = true;  // to print contents initially
        SerialTty.println("SD1 Card suppored on USB");
    }
#endif  // USE_USB_MSC_SD1
#endif  // USE_USB_MSC_SD0
#endif  // BOARD_SDQ_QSPI_FLASH
    return retVal;
}

void        Logger::SDusbPoll(uint8_t sdActions) {
#if defined USE_USB_MSC_SD0
    if (sd0_card_changed) {
        File root_fs;
        root_fs = sd0_card_fatfs.open("/");

        PRINTOUT("Flash contents:");
        print_rootdir(&root_fs);
        PRINTOUT("\n");

        root_fs.close();

        sd0_card_changed = false;
    }
#if defined USE_USB_MSC_SD1
    if (sd1_card_changed) {
        File root_fs;
        root_fs = sd1_card_phy.open("/");

        PRINTOUT("SD contents:\n");
        print_rootdir(&root_fs);
        PRINTOUT("\n");

        root_fs.close();

        sd1_card_changed = false;
    }
#endif  // USE_USB_MSC_SD1
#endif  // USE_USB_MSC_SD0
}
// ===================================================================== //
// Public functions extensions
// ===================================================================== //
#if 0
void Logger::setLoggingInterval_atl485(uint16_t loggingIntervalMinutes)
{
    _loggingIntervalMinutes = loggingIntervalMinutes;
#ifdef DEBUGGING_SERIAL_OUTPUT
        const char* prtout1 = "Logging interval:";
        PRINTOUT(prtout1, _loggingIntervalMinutes);
#endif
}
#endif
void Logger::setLoggerId(const char* newLoggerId, bool copyId,
                         uint8_t LoggerIdMaxSize) {
    uint8_t LoggerIdSize = strlen(newLoggerId) + 2;
    _loggerID            = newLoggerId;

    if (copyId) {
        /* Do size checks, allocate memory for the LoggerID, copy it there
         *  then set assignment.
         */
        if (LoggerIdSize > LoggerIdMaxSize) {
            char* newLoggerId2 = (char*)newLoggerId;
            PRINTOUT(F("\n\r   Logger:setLoggerId too long: Trimmed to "),
                     LoggerIdMaxSize);
            newLoggerId2[LoggerIdMaxSize] = 0;  // Trim max size
            LoggerIdSize                  = LoggerIdMaxSize;
        }
        if (NULL == _LoggerId_buf) {
            _LoggerId_buf = new char[LoggerIdSize + 2];  // Allow for trailing 0
        } else {
            PRINTOUT(F("\nLogger:setLoggerId error - expected NULL ptr"));
        }
        if (NULL == _LoggerId_buf) {
            // Major problem
            PRINTOUT(F("\nLogger::setLoggerId error -no buffer "), newLoggerId);
        } else {
            strcpy((char*)_LoggerId_buf, newLoggerId);
            _loggerID = _LoggerId_buf;
        }
        MS_DBG(F("\nsetLoggerId cp "), _loggerID, " sz: ", LoggerIdSize);
    }
}

void Logger::setBatHandler(bool (*bat_handler_atl)(lb_pwr_req_t)) {
    _bat_handler_atl = bat_handler_atl;
}

// ===================================================================== //
// Parse an ini file for customization
// ===================================================================== //

#define INI_USE_STACK 1
/* Maximum line length for any line in INI file (stack or heap). Note that
   this must be 3 more than the longest line (due to '\r', '\n', and '\0'). */
//#define INI_MAX_LINE 200
#define INI_MAX_LINE 100
#define MAX_SECTION 50
#define MAX_NAME 50
/* Nonzero to allow multi-line value parsing, in the style of Python's
   configparser. If allowed, ini_parse() will call the handler with the same
   name for each subsequent line parsed. */
#ifndef INI_ALLOW_MULTILINE
#define INI_ALLOW_MULTILINE 1
#endif

/* Nonzero to allow a UTF-8 BOM sequence (0xEF 0xBB 0xBF) at the start of
   the file. See https://github.com/benhoyt/inih/issues/21 */
#ifndef INI_ALLOW_BOM
#define INI_ALLOW_BOM 1
#endif

/* Chars that begin a start-of-line comment. Per Python configparser, allow
   both ; and # comments at the start of a line by default. */
#ifndef INI_START_COMMENT_PREFIXES
#define INI_START_COMMENT_PREFIXES ";#"
#endif

/* Nonzero to allow inline comments (with valid inline comment characters
   specified by INI_INLINE_COMMENT_PREFIXES). Set to 0 to turn off and match
   Python 3.2+ configparser behaviour. */
#ifndef INI_ALLOW_INLINE_COMMENTS
#define INI_ALLOW_INLINE_COMMENTS 1
#endif
#ifndef INI_INLINE_COMMENT_PREFIXES
#define INI_INLINE_COMMENT_PREFIXES ";"
#endif

/* Strip whitespace chars off end of given string, in place. Return s. */
static char* rstrip(char* s) {
    char* p = s + strlen(s);
    while (p > s && isspace((unsigned char)(*--p))) *p = '\0';
    return s;
}

/* Return pointer to first non-whitespace char in given string. */
static char* lskip(const char* s) {
    while (*s && isspace((unsigned char)(*s))) s++;
    return (char*)s;
}

/* Return pointer to first char (of chars) or inline comment in given string,
   or pointer to null at end of string if neither found. Inline comment must
   be prefixed by a whitespace character to register as a comment. */
static char* find_chars_or_comment(const char* s, const char* chars) {
#if INI_ALLOW_INLINE_COMMENTS
    int was_space = 0;
    while (*s && (!chars || !strchr(chars, *s)) &&
           !(was_space && strchr(INI_INLINE_COMMENT_PREFIXES, *s))) {
        was_space = isspace((unsigned char)(*s));
        s++;
    }
#else
    while (*s && (!chars || !strchr(chars, *s))) { s++; }
#endif
    return (char*)s;
}

/* Version of strncpy that ensures dest (size bytes) is null-terminated. */
static char* strncpy0(char* dest, const char* src, size_t size) {
    strncpy(dest, src, size - 1);
    dest[size - 1] = '\0';
    return dest;
}

//////// this is pretty challenging
#define PROCESS_LOCAL_INIH 0
#if PROCESS_LOCAL_INIH
/* Process sections of INI file.
   May have [section]s,
   name=value pairs (whitespace stripped)
   if [Section] not found then pass on to callers  unhandledFn1()

   */
ini_handler unhandledFn1;  // Address of handler if local parsing can't find it
// expect to be in near space
//#define EDIY_PROGMEM PROGMEM
#define mCONST_UNI(p1) const char p1##_pm[] PROGMEM = #p1
const char BOOT_pm[] EDIY_PROGMEM           = "BOOT";
const char VER_pm[] EDIY_PROGMEM            = "VER";
const char MAYFLY_SN_pm[] EDIY_PROGMEM      = "MAYFLY_SN";
const char MAYFLY_REV_pm[] EDIY_PROGMEM     = "MAYFLY_REV";
const char MAYFLY_INIT_ID_pm[] EDIY_PROGMEM = "MAYFLY_INIT_ID";

const char COMMON_pm[] EDIY_PROGMEM    = "COMMON";
const char LOGGER_ID_pm[] EDIY_PROGMEM = "LOGGER_ID";
// mCONST_UNI(LOGGER_ID);// = "nh07k" ;
const char LOGGING_INTERVAL_MIN_pm[] EDIY_PROGMEM = "LOGGING_INTERVAL_MIN";
const char LIION_TYPE_pm[] EDIY_PROGMEM           = "LIION_TYPE";
const char TIME_ZONE_pm[] EDIY_PROGMEM            = "TIME_ZONE";
const char GEOGRAPHICAL_ID_pm[] EDIY_PROGMEM      = "GEOGRAPHICAL_ID";

const char NETWORK_pm[] EDIY_PROGMEM = "NETWORK";
const char apn_pm[] EDIY_PROGMEM     = "apn";
const char WiFiId_pm[] EDIY_PROGMEM  = "WiFiId";
const char WiFiPwd_pm[] EDIY_PROGMEM = "WiFiPwd";

const char PROVIDER_pm[] EDIY_PROGMEM           = "PROVIDER";
const char CLOUD_ID_pm[] EDIY_PROGMEM           = "CLOUD_ID";
const char REGISTRATION_TOKEN_pm[] EDIY_PROGMEM = "REGISTRATION_TOKEN";
const char SAMPLING_FEATURE_pm[] EDIY_PROGMEM   = "SAMPLING_FEATURE";

const char     UUIDs_pm[] EDIY_PROGMEM = "UUIDs";
const char     index_pm[] EDIY_PROGMEM = "index";
static uint8_t uuid_index              = 0;
static int     inihandlerFn(const char* section, const char* name,
                            const char* value) {
    int retVal = 0
#if 0
    if (strcmp_P(section,PROVIDER_pm)== 0)
    {
        if        (strcmp_P(name,REGISTRATION_TOKEN_pm)== 0) {
            strcpy(ps.provider.s.registration_token, value);
            Serial.print(F("PROVIDER Setting registration token: "));
            Serial.println(ps.provider.s.registration_token );
            EnviroDIYPOST.setToken(ps.provider.s.registration_token);
        } else if (strcmp_P(name,CLOUD_ID_pm)== 0) {
            strcpy(ps.provider.s.cloudId, value);
            Serial.print(F("PROVIDER Setting cloudId: "));
            Serial.println(ps.provider.s.cloudId );
        } else if (strcmp_P(name,SAMPLING_FEATURE_pm)== 0) {
            strcpy(ps.provider.s.sampling_feature, value);
            Serial.print(F("PROVIDER Setting SamplingFeature: "));
            Serial.println(ps.provider.s.sampling_feature );
            dataLogger.setSamplingFeatureUUID(ps.provider.s.sampling_feature);
        } else {
            Serial.print(F("PROVIDER not supported:"));
            Serial.print(name);
            Serial.print("=");
            Serial.println(value);
        }
    } else
#endif  // 0

        if (strcmp_P(section, UUIDs_pm) == 0) {
        /*FUT:Add easier method to mng
        perhaps1) "UUID_label"="UUID"
        then search variableList till find UUID_label
        perhaps2) "dummyDefaultUID"="UUID"
            "ASQ212_PAR"="UUID"
            then search variablList till find dummyDefaultUID and replace with
        UUID perhaps3) "SensorName"="UUID" "ApogeeSQ212_PAR"="UUID" then search
        variablList till fin SensorsName

        */
        if (strcmp_P(name, index_pm) == 0) {
            PRINTOUT(F("["));
            PRINTOUT(uuid_index);
            PRINTOUT(F("]={"));
            PRINTOUT(value);
            if (uuid_index < variableCount) {
                PRINTOUT(F("} replacing {"));
                PRINTOUT(variableList[uuid_index]->getVarUUID());
                PRINTOUT(F("}\n\r"));

                strcpy(&ps.provider.s.uuid[uuid_index][0], value);
                // variableList[uuid_index]->setVarUUID(&ps.provider.s.uuid[uuid_index][0]);

            } else {
                Serial.println(F("} out of range. Notused"));
            }
            uuid_index++;
        } else {
            PRINTOUT(F("UUIDs not supported:"));
            PRINTOUT(name);
            PRINTOUT("=");
            PRINTOUT(value, "\n\r");
        }
        //} else if (strcmp_P(section,COMMON_pm)== 0) {
    }
    else {
        MS_DBG(F("LBE: Nothandled Sect:"), section);
        retVal = unhandledFn1(section, name, value);
    }

    return retVal;
}
#endif  // PROCESS_LOCAL_INIH

/* Parse given INI-style file.
   May have [section]s,
   name=value pairs (whitespace stripped), and
   comments starting with ';' (semicolon).
   Section  is "" if name=value pair parsed before any section heading.
   name:value     pairs are also supported as a concession to Python's
   configparser.

   For each name=value pair parsed, call handler function with given user
   pointer as well as section, name, and value (data only valid for duration
   of handler call).
   Handler should return nonzero on success, zero on error.

   Returns 0 on success,
     line number of last error on parse error (doesn't stop on first error),

   https://en.wikipedia.org/wiki/INI_file
   https://github.com/benhoyt/inih
*/
int8_t Logger::inihParseFile(ini_handler_atl485 handler_fn) {
    /* Uses a fair bit of stack (use heap instead if you need to) */
#if INI_USE_STACK
    char line[INI_MAX_LINE];
    int  max_line = INI_MAX_LINE;
#else
    char* line;
    int   max_line = INI_INITIAL_ALLOC;
#endif
#if INI_ALLOW_REALLOC && !INI_USE_STACK
    char* new_line;
    int   offset;
#endif
    char section[MAX_SECTION] = "";
    char prev_name[MAX_NAME]  = "";

    char* start;
    char* end;
    char* name;
    char* value;
    int   lineno = 0;
    int   error  = 0;

#if !INI_USE_STACK
    line = (char*)malloc(INI_INITIAL_ALLOC);
    if (!line) { return -2; }
#endif

    /* Scan through stream line by line */
#define readerLog_fn(line1, max_line1) logFile.fgets(line1, max_line1)
    while (readerLog_fn(line, max_line) != 0) {
#if INI_ALLOW_REALLOC && !INI_USE_STACK
        offset = strlen(line);
        while (offset == max_line - 1 && line[offset - 1] != '\n') {
            max_line *= 2;
            if (max_line > INI_MAX_LINE) max_line = INI_MAX_LINE;
            new_line = realloc(line, max_line);
            if (!new_line) {
                free(line);
                return -2;
            }
            line = new_line;
            if (readerLog_fn(line + offset, max_line - offset) == NULL) break;
            if (max_line >= INI_MAX_LINE) break;
            offset += strlen(line + offset);
        }
#endif

        lineno++;

        start = line;
#if INI_ALLOW_BOM
        if (lineno == 1 && (unsigned char)start[0] == 0xEF &&
            (unsigned char)start[1] == 0xBB &&
            (unsigned char)start[2] == 0xBF) {
            start += 3;
        }
#endif
        start = lskip(rstrip(start));

        if (strchr(INI_START_COMMENT_PREFIXES, *start)) {
            /* Start-of-line comment */
        }
#if INI_ALLOW_MULTILINE
        else if (*prev_name && *start && start > line) {
            /* Non-blank line with leading whitespace, treat as continuation
               of previous name's value (as per Python configparser). */
            if (!handler_fn(section, prev_name, start) && !error)
                error = lineno;
        }
#endif
        else if (*start == '[') {
            /* A "[section]" line */
            end = find_chars_or_comment(start + 1, "]");
            if (*end == ']') {
                *end = '\0';
                strncpy0(section, start + 1, sizeof(section));
                *prev_name = '\0';
            } else if (!error) {
                /* No ']' found on section line */
                error = lineno;
            }
        } else if (*start) {
            /* Not a comment, must be a name[=:]value pair */
            end = find_chars_or_comment(start, "=:");
            if (*end == '=' || *end == ':') {
                *end  = '\0';
                name  = rstrip(start);
                value = end + 1;
#if INI_ALLOW_INLINE_COMMENTS
                end = find_chars_or_comment(value, NULL);
                if (*end) *end = '\0';
#endif
                value = lskip(value);
                rstrip(value);

                /* Valid name[=:]value pair found, call handler */
                strncpy0(prev_name, name, sizeof(prev_name));
                if (!handler_fn(section, name, value) && !error) error = lineno;
            } else if (!error) {
                /* No '=' or ':' found on name[=:]value line */
                error = lineno;
            }
        }

#if INI_STOP_ON_FIRST_ERROR
        if (error) break;
#endif
    }

#if !INI_USE_STACK
    free(line);
#endif

    return error;
}

bool Logger::parseIniSd(const char* ini_filename, ini_handler_atl485 unhandledFnReq) {
    //Parses a 'ini' file.
    // if there is ini_filename+'0' parse and then rename so won't be parsed again
    // if there is ini_filename+'1' parse and then rename so won't be parsed again
    // if there is ini_filename parse

    // Initialise the SD card
    // skip everything else if there's no SD card, otherwise it might hang
    if (!initializeSDCard()) return false;

    #if defined(__AVR__)
    //Mayfly has internal EEPROM that can be reformatted
    //const char ini_ext='0';
    parseAndRename('0', ini_filename, unhandledFnReq);
    if (parseAndRename('1', ini_filename, unhandledFnReq)) {
        /// Good bye world, reset  
        forceSysReset(0,4567);
    }
    #endif // _AVR__
    if (sd1_card_fatfs.exists(ini_filename)) {
        parseIniFile(ini_filename, unhandledFnReq);
    } else {
        PRINTOUT(F("Parse ini; No file "), ini_filename);
        return false;
    }
    return true;
}

bool Logger::parseIniFile(const char * ini_filename, ini_handler_atl485 unhandledFnReq) {
   uint8_t ini_err;
   bool retStatus;
   retStatus = logFile.open(ini_filename); 
   if (retStatus) {
        PRINTOUT( ini_filename,F(": parsing----"));
#if PROCESS_LOCAL_INIH
        unhandledFn1 = unhandledFnReq;
        ini_err =
            inihParseFile(inihandlerFn);  // handle found sections locall first
#else                                     // PROCESS_LOCAL_INIH
        ini_err = inihParseFile(unhandledFnReq);  // handle found sections
#endif                                    //
        logFile.close();
        if (ini_err) {
            PRINTOUT(F("Error on line :"), ini_err);
        } else {
            PRINTOUT(F("Completed."));
        }
    } 
    return retStatus;
}//parseIniFile

#if defined(__AVR__)
bool Logger::parseAndRename(const char ini_ext, const char* ini_filename, ini_handler_atl485 unhandledFnReq) {
    bool retStatus=true; //Was action completed
#define FN_EXT_LEN_DEF strlen(ini_filename)+3
    String fn_ext(FN_EXT_LEN_DEF);
    fn_ext = ini_filename;
    fn_ext += ini_ext;

    // Check if exists 0 or 1 variants, parse and rename 
    if (sd1_card_fatfs.exists(fn_ext)) {
        String fn_ren(strlen(ini_filename)+4);
        fn_ren = ini_filename;
        fn_ren += ini_ext;
        fn_ren += "run";

        parseIniFile(fn_ext.c_str(), unhandledFnReq);
        PRINTOUT(F("ParseIniSd  "), fn_ext,F("renamed to "),fn_ren);

        if (sd1_card_fatfs.exists(fn_ren)) {
            // delete so can rename next file
            if (!sd1_card_fatfs.remove(fn_ren)) {
                PRINTOUT(F("piSd err couldn't del "), fn_ren);
            }
        }
        if (!sd1_card_fatfs.rename(fn_ext,fn_ren)) {
            if (sd1_card_fatfs.remove(fn_ext)) {
                PRINTOUT(F("piSd err couldn't rename "), fn_ext,F(" deleted"));
            }else {
                PRINTOUT(F("piSd err del/ren "), fn_ext);
                retStatus = false;
            }
        }
    } else {
        MS_DBG(F("ParseIniSd not found file"), fn_ext);
        retStatus = false;
    }
    return retStatus;
} //parseAndRename
#endif // __AVR__

void Logger::forceSysReset(uint8_t source, uint16_t simpleMagicNumber) {
    
    if (4567 !=simpleMagicNumber) return;

    PRINTOUT(F("Forcing reset"), source);
    delay(20);
    watchDogTimer.setupWatchDog(1);
    watchDogTimer.enableWatchDog();
    delay(100000); //Expect watchdog to kick in within 8secs
} //forceReset

#ifdef USE_MS_SD_INI
void Logger::setPs_cache(persistent_store_t* ps_ram) {
    ps_cache = ps_ram;
}

extern const String build_ref;
extern const char*  configDescription;
void                Logger::printFileHeaderExtra(Stream* stream) {
    if (NULL == ps_cache) return;
    stream->print(F("Location: "));
    stream->println((char*)ps_cache->app.msc.s.geolocation_id);
    stream->print(F("  LoggingInterval (min): "));
    stream->println(ps_cache->app.msc.s.logging_interval_min);
    stream->print(F(" Tz: "));
    stream->println(ps_cache->app.msc.s.time_zone);
    stream->print(F("  BatteryType: "));
    stream->println(ps_cache->app.msc.s.battery_type);
    stream->print(F("Sw Name: "));
    stream->print(configDescription);
    stream->print(F(". Sw Build: "));
    stream->println(build_ref);
    stream->print(F("ModularSensors vers "));
    stream->println(MODULAR_SENSORS_VERSION);
#if defined USE_PS_HW_BOOT
    stream->print(F("Board: "));
    stream->print((char*)ps_cache->hw_boot.board_name);
    stream->print(F(" rev:'"));
    stream->print((char*)ps_cache->hw_boot.rev);
    stream->print(F("' sn:'"));
    stream->print((char*)ps_cache->hw_boot.serial_num);
    stream->println(F("'"));
#endif  // USE_PS_HW_BOOT

    //Add known modemDetail - assumes been initialized
    { 
        String modemDetails = _logModem->getModemDevId();
        if (modemDetails.length()) {
            stream->print(modemDetails);
            stream->println();  
        }
    }
    //Add sensor details if known  assumes been initialized
    String sensorDetails;
    for (uint8_t i = 0; i < getArrayVarCount(); i++) {
        sensorDetails =  getParentSensorDetails(i);
        if (sensorDetails.length()) {
            stream->print(sensorDetails);
            stream->print(F(" for ")); 
            stream->print( getVarNameAtI(i));  
            stream->println();  
        }
    }     


}
#endif  // USE_MS_SD_INI

#if defined USE_RTCLIB
USE_RTCLIB* Logger::rtcExtPhyObj() {
    return &rtcExtPhy;
}
#endif  // USE_RTCLIB

// End parse.ini

// ===================================================================== //
// Reliable Delivery functions
// see class headers
// ===================================================================== //

// This is a one-and-done to log data
void Logger::logDataAndPubReliably(uint8_t cia_val_override) {
    // Reset the watchdog
    watchDogTimer.resetWatchDog();


    // Assuming we were woken up by the clock, check if the current time is
    // an even interval of the logging interval
    uint8_t cia_val = checkInterval();
    if (cia_val_override) {
        cia_val = cia_val_override;
        wakeUpTime_secs = getNowEpochTz();//Set reference time
        markTime();
        PRINTOUT(F("logDataAndPubReliably - overide with "),cia_val);
    }
    if (NULL != _bat_handler_atl) {
        _bat_handler_atl(LB_PWR_USEABLE_REQ);  // Set battery status
        if (!_bat_handler_atl(LB_PWR_SENSOR_USE_REQ)) {
            // Squash any activity
            //PRINTOUT(F("logDataAndPubReliably - all cancelled"));
            const static char ALL_CANCELLED_pm[] EDIY_PROGMEM = 
            "logDataAndPubReliably - all cancelled"; 
            PRINT_LOGLINE_P(ALL_CANCELLED_pm);
            cia_val = 0;
        }
        if (!_bat_handler_atl(LB_PWR_MODEM_USE_REQ)) {
            if (CIA_POST_READINGS & cia_val) {
                // Change publish attempt to saving for next publish attempt
                cia_val &= ~CIA_POST_READINGS;
                cia_val |= CIA_RLB_READINGS;  //
                //PRINTOUT(F("logDataAndPubReliably - tx cancelled"));
                const static char TX_CANCELLED_pm[] EDIY_PROGMEM = 
                "logDataAndPubReliably - tx cancelled";
                PRINT_LOGLINE_P(TX_CANCELLED_pm);
            }
        }
    }

    if (cia_val) {
        // Flag to notify that we're in already awake and logging a point
        Logger::isLoggingNow = true;
        // Reset the watchdog
        watchDogTimer.resetWatchDog();

        // Print a line to show new reading
        //PRINTOUT(F("---logDataAndPubReliably ("),cia_val,F(")----"));
        STANDARD_SERIAL_OUTPUT.print(F("---logDataAndPubReliably (0x"));
        STANDARD_SERIAL_OUTPUT.print(cia_val,HEX);
        STANDARD_SERIAL_OUTPUT.println(F(")----"));
        // Turn on the LED to show we're taking a reading
        alertOn();
        // Power up the SD Card
        // TODO(SRGDamia1):  Decide how much delay is needed between turning on
        // the card and writing to it.  Could we turn it on just before writing?
        turnOnSDcard(false);
        if (cia_val & CIA_NEW_READING) {
            // Do a complete update on the variable array.
            // This this includes powering all of the sensors, getting
            // updated values, and turing them back off. NOTE:  The wake
            // function for each sensor should force sensor setup to run if
            // the sensor was not previously set up.
            PRINTOUT(F("Read sensors..."));
            watchDogTimer.resetWatchDog();
            _internalArray->completeUpdate();
            watchDogTimer.resetWatchDog();

            // Create a csv data record and save it to the log file
            logToSD();

            serzRdel_Line();  // Start Que
        }
        if (cia_val & CIA_POST_READINGS) {
            if (_logModem != NULL) {
                MS_DBG(F("Waking up"), _logModem->getModemName(), F("..."));
                if (_logModem->modemWake()) {
                    // Connect to the network
                    watchDogTimer.resetWatchDog();
                    PRINTOUT(F("Connecting to the Internet with"),_logModem->getModemName());
                    if (_logModem->connectInternet()) {
                        const static char CONNECT_INTERNET_pm[] EDIY_PROGMEM = 
                        "Connected Internet"; 
                        PRINT_LOGLINE_P(CONNECT_INTERNET_pm);
                        // be nice to add _logModem->getModemName()
                        //This doesn't work PRINT_LOGLINE_P2(CONNECT_INTERNET_pm,_logModem->getModemName().c_str());
                        // Publish data to remotes
                        watchDogTimer.resetWatchDog();
                        publishDataQuedToRemotes(true);
                        watchDogTimer.resetWatchDog();

// Sync the clock at midnight or on the hour
#define NIST_SYNC_DAY 86400
#define NIST_SYNC_HR 3600
#if defined NIST_SYNC_HOURLY
#define NIST_SYNC_RATE NIST_SYNC_HR 
#else
#define NIST_SYNC_RATE NIST_SYNC_DAY
#endif //NIST_SYNC_HOURLY
                        uint32_t logIntvl_sec = _loggingIntervalMinutes * 60; 
                        uint32_t timeToday_sec = markedEpochTime % NIST_SYNC_RATE;
                        bool doSyncTimeCheck = (timeToday_sec< logIntvl_sec);
                        /*MS_DBG*/PRINTOUT(F("SyncTimeCheck "),doSyncTimeCheck," modulo_sec",timeToday_sec," Time",Logger::markedEpochTime);
                        if (doSyncTimeCheck) {
                            MS_DBG(F("Running an NIST clock sync..."));
                            if(setRTClock(_logModem->getNISTTime())) {
                                const static char CLOCK_NIST_OK_pm[] EDIY_PROGMEM ="Clock Nist Synced"; 
                                PRINT_LOGLINE_P(CLOCK_NIST_OK_pm);                                       
                            } else {
                                const static char CLOCK_NIST_FAIL_pm[] EDIY_PROGMEM ="Clock Nist Failed"; 
                                PRINT_LOGLINE_P(CLOCK_NIST_FAIL_pm);       
                            }
                        }
                        watchDogTimer.resetWatchDog();

                        // Update the modem metadata
                        MS_DBG(F("Updating modem metadata..."));
                        _logModem->updateModemMetadata();

                        // Disconnect from the network
                        MS_DBG(F("Disconnecting from the Internet..."));
                        _logModem->disconnectInternet();
                    } else {
                        //RINTOUT(F("Connect to the internet failed with"),_logModem->getModemName());
                        const static char CONNECT_FAILED_pm[] EDIY_PROGMEM = 
                        "Connected Internet Failed"; 
                        PRINT_LOGLINE_P(CONNECT_FAILED_pm);
                        watchDogTimer.resetWatchDog();
                    }
                } else {
                    PRINTOUT(F("Failed to wake "), _logModem->getModemName());
                }
                // Turn the modem off
                _logModem->modemSleepPowerDown();
            } else
                PRINTOUT(F("Internet failed, no _logModem "));
        } else if (cia_val & CIA_RLB_READINGS) {
            // Values not transmitted,  save readings for later transmission
            PRINTOUT(F("logDataAndPubReliably - store readings, no pub"));
            publishDataQuedToRemotes(false);
        }


        // TODO(SRGDamia1):  Do some sort of verification that minimum 1 sec has
        // passed for internal SD card housekeeping before cutting power It
        // seems very unlikely based on my testing that less than one second
        // would be taken up in publishing data to remotes
        // Cut power from the SD card - without additional housekeeping wait
        turnOffSDcard(false);

        // Turn off the LED
        alertOff();
        // Print a line to show reading ended
        PRINTOUT(F("---logDataAndPubReliably  Complete----------"));

        // Unset flag
        Logger::isLoggingNow = false;
        dumpFreeRam(8256); //large Number
    }

    // Check if it was instead the testing interrupt that woke us up
    if (Logger::startTesting) testingMode();

    // Call the processor sleep
    systemSleep();
}

#if defined(__AVR__)
//Verify EveryMinute on the RTC DS3231M
void Logger::setExtRtcSleep() {
    uint8_t isRtcRegBad;

    #if defined ARDUINO_AVR_ENVIRODIY_MAYFLY
    #define MAYFLY_WR_RETRYS 3
    for (uint8_t chklp=0;chklp<MAYFLY_WR_RETRYS;chklp++) {
        isRtcRegBad = rtc.enableInterruptsCheckAlm1(EveryMinute);
        if (0==isRtcRegBad) {
            MS_DBG(F("RTC Alarm good." ));
            break;
        }
        Serial.print(chklp);
        Serial.print(F("]RTC Alarm set for every minute. Reg check was 0x"));
        Serial.println(isRtcRegBad,HEX);
        rtc.enableInterrupts(EveryMinute);
        //rtc.enableInterruptsAlm2(EveryMinute);
    } 
    #else
    rtc.enableInterrupts(EveryMinute);
    #endif 
}
#endif // __AVR__

void Logger::publishDataQuedToRemotes(bool internetPresent) {
    // Assumes that there is an internet connection
    // bool    useQue = false;
    int16_t  rspCode = 0;
    uint32_t tmrGateway_ms;
    bool     dslStatus = false;
    bool     retVal    = false;
    // MS_DBG(F("Pub Data Qued"));
    MS_DBG(F("pubDQTR from"), serzRdelFn_str, internetPresent);

    // Open debug file
#if defined MS_LOGGERBASE_POSTS
    retVal = postLogOpen(postsLogFn_str);
#endif  // MS_LOGGERBASE_POSTS

    for (uint8_t i = 0; i < MAX_NUMBER_SENDERS; i++) {
        if (dataPublishers[i] != NULL) {
            _dataPubInstance = i;
            PRINTOUT(F("\npubDQTR Sending data to ["), i, F("]"),
                     dataPublishers[i]->getEndpoint());
            // open the qued file for serialized readings
            // (char*)serzQuedFn_str


            // dataPublishers[i]->publishData(_logModem->getClient());
            // Need to manage filenames[i]

            /* TODO njh check power availability
            ps_Lbatt_status_t Lbatt_status;
            Lbatt_status =
            mcuBoard.isBatteryStatusAbove(true,PS_PWR_USEABLE_REQ);
            if (no power) break out for loop;
            */

            if (dataPublishers[i]->getQuedStatus()) {
                uint16_t delay_posted_pacing_ms = dataPublishers[i]->getTimerPostPacing_mS();
                uint16_t published_this_pass =0;
                serzQuedStart((char)('0' + i));
                deszRdelStart();
                // MS_START_DEBUG_TIMER;
                tmrGateway_ms = millis();
                uint32_t tmrThisPublish_ms;
                while ((dslStatus = deszRdelLine())) {
                    tmrThisPublish_ms = millis();
                    if (internetPresent) {
                        rspCode = dataPublishers[i]->publishData();
                    } else {
                        rspCode = HTTPSTATUS_NC_902;
                    }

                    watchDogTimer.resetWatchDog();
                    // MS_DBG(F("Rsp"), rspCode, F(", in"),
                    // MS_PRINT_DEBUG_TIMER,    F("ms\n"));
                    postLogLine( (millis() -tmrThisPublish_ms), rspCode);

                    if (HTTPSTATUS_CREATED_201 != rspCode) {
#define DESLZ_STATUS_UNACK '1'
#define DESLZ_STATUS_MAX '8'
#define DESLZ_STATUS_POS 0
#if 0
                        if (++deszq_line[0] > '8') {
                            deszq_line[DESLZ_STATUS_POS] = DESLZ_STATUS_UNACK;
                        }
#endif  // if x
#if defined(USE_PS_modularSensorsNetwork)
                        if ((desz_pending_records >= _sendQueSz_num)&&(MMWGI_SEND_QUE_SZ_NUM_NOP != _sendQueSz_num )) {
                                PRINTOUT(F("pubDQTR QuedFull, skip reading. sendQue "),  _sendQueSz_num);
                                postLogLine(0,rspCode); //Log skipped readings
                        } else 
 #endif // USE_PS_modularSensorsNetwork
                        {
                            retVal = serzQuedFile.print(deszq_line);
                            if (0 >= retVal) {
                                PRINTOUT(F("pubDQTR serzQuedFil err"), retVal);
                            }
                            desz_pending_records++;  // TODO: njh per publisher
                        }
                        /*TODO njh process
                        if (HTTPSTATUS_NC_901 == rspCode) {
                            MS_DBG(F("pubDQTR abort this
                        servers POST " "attempts"));

                        However, will also have to cleanup/copy lines from
                        serzQuedFile to before deszRdelClose

                        break;

                        }

                        */
                    } else {
                        /*A publish has been sucessfull.
                         * Slow Down sending based on publishers acceptance rate
                         * Each publish creates and tears down a TCP connection */
                        /*TODO njh create intergrate all POSTS to one tcp/ip connection */
                        published_this_pass++;
                        MS_DBG(F("pubDQTR1 delay"),delay_posted_pacing_ms ,F("mS : posted"), published_this_pass);
                        delay(delay_posted_pacing_ms);
                    }
                }  // while reading line
                deszRdelClose(true);
                serzQuedCloseFile(false);
                // retVal = serzQuedFile.close();
                // if (!retVal)
                //    PRINTOUT(
                //        F("publishDataQuedToRemote serzQuedFile.close err"));

                PRINTOUT(F("Sent"), deszLinesRead, F("readings in"),
                         ((float)(millis() - tmrGateway_ms)) / 1000,
                         F("sec. Queued readings="), desz_pending_records);

                if (HTTPSTATUS_CREATED_201 == rspCode) {
                    // Do retrys through publisher - if file exists
                    if (sd1_card_fatfs.exists(serzQuedFn)) {
                        uint16_t tot_posted           = 0;
                        uint16_t cnt_for_pwr_analysis = 1;
                        MS_DBG(F("pubDQTR retry from"), serzQuedFn);
                         deszQuedStart();
                        while ((dslStatus = deszQuedLine()) )  {

                            /*At least one publish has been sucessfull.
                             * Slow Down sending based on publishers acceptance rate
                             * Each publish creates and tears down a TCP connection */
                            MS_DBG(F("pubDQTR2 delay"),delay_posted_pacing_ms ,F("mS : total posted"), published_this_pass);
                            delay(delay_posted_pacing_ms);

                            // setup for publisher to call deszqNextCh()
                            rspCode = dataPublishers[i]->publishData();
                            watchDogTimer.resetWatchDog();
                            postLogLine(i, rspCode);
                            if (HTTPSTATUS_CREATED_201 != rspCode) break;

                            tot_posted++;
                            published_this_pass++;

                            deszq_line[0] = 0;  // Show completed

                            // Check for enough battery power
                            if (cnt_for_pwr_analysis++ >=
                                _sendAtOneTimeMaxX_num) {
                                if (NULL != _bat_handler_atl) {
                                    // Measure  battery
                                    _bat_handler_atl(LB_PWR_USEABLE_REQ);
                                    if (!_bat_handler_atl(
                                            LB_PWR_MODEM_USE_REQ)) {
                                        // stop transmission
                                        cnt_for_pwr_analysis = 0;
                                        PRINTOUT(F("pubDQTR not enough power available"));
                                        break;
                                    }
                                }
                                cnt_for_pwr_analysis = 1;
                            }
                            if ((tot_posted  >= _postMax_num) && (0 != _postMax_num)) {
                                PRINTOUT(F("pubDQTR POST_MAX_NUM reached"), tot_posted);
                                break; /// unsent lines are copied through
                            }
                        } //while
// increment status of number attempts
#if 0
                        if (deszq_line[DESLZ_STATUS_POS]++ >=
                            DESLZ_STATUS_MAX) {
                            deszq_line[DESLZ_STATUS_POS] = DESLZ_STATUS_MAX;
                        }
#endif  // if z
        // deszQuedCloseFile() is serzQuedCloseFile(true)
                        if (tot_posted) {
                            // At least one POST was accepted, if 2 or more, the last may have failed
                            // and still be in deszq_line
                            serzQuedCloseFile(true);
                        } else {
                            serzQuedCloseFile(false);
                        }
                    } else { MS_DBG(F("pubDQTR no queued file"), serzQuedFn);}
                } else {
                    MS_DBG(F("pubDQTR drop retrys. rspCode"), rspCode);
                }
            }
        }
    }
    postLogClose();
}

// ===================================================================== //
// Serialize/deserialize functions
// see class headers
// ===================================================================== //

#define DELIM_CHAR2 ','
#define SERZQUED_OFLAGS
bool Logger::serzQuedStart(char uniqueId) {
    strcpy(serzQuedFn, serzQuedFn_str);
    strncat(serzQuedFn, &uniqueId, 1);
    strcat(serzQuedFn, ".TXT");

    if (!serzQuedFile.open(serzQuedFn, (O_WRITE | O_CREAT | O_AT_END))) {
        PRINTOUT(F("serzQuedStart open err"));
        return false;
    } else {
        MS_DEEP_DBG(F("serzQuedStart open"), serzQuedFn);
    }
    return true;
}

bool Logger::serzQuedCloseFile(bool flush) {
    /* This closes the file, removing the sent messages
     Assumes serzQuedFile points incoming file if flush==true
    */
    bool    retBool=true;

    if (flush) {   
        // There may be 0, or more of unsent records left in serzQued
        uint16_t num_lines = serzQuedFlushFile();

        PRINTOUT(F("seQCF Que for next pass unsent records"), num_lines);
        desz_pending_records = num_lines;

    } else { // !flush simple clean
        retBool = serzQuedFile.close();
        if (!retBool) {
            sd1_Err("seQCF serzQuedFile.close2 err");
            return false;
        }
    }
    return retBool;
}

#define TEMP_BASE_FN_STR "TMP01.TXT"
#define QUEOLD_BASE_FN_STR "QUEDEL01.TXT"
inline uint16_t Logger::serzQuedFlushFile() {
    /*  The flush algorithim is, 
     copy unsent lines to a temporary_file up to _sendQueSz_num, and then discard rest
     Assumes serzQuedFile points incoming file
     when complete rename serzQuedFile  to delete_file
     rename temporary_file to serzQuedFile to complete flush
    */
    const char* tempFn = TEMP_BASE_FN_STR;
    const char* queDelFn = QUEOLD_BASE_FN_STR;
    File    tgtoutFile;
    int16_t retNum;
    int16_t  num_char ;
    uint16_t num_lines = 0;   
    uint16_t num_skipped=0;
    bool    retBool;

    // Check if exists and delete
    if (sd1_card_fatfs.exists(tempFn)) {
        if (!sd1_card_fatfs.remove(tempFn)) {
            PRINTOUT(F("seQFF remove1 err"), tempFn);
            sd1_Err("seQFF err6 remove");
        } else {
            MS_DEEP_DBG(F("seQFF remove "), tempFn);
        }
    }  
    retBool = tgtoutFile.open(tempFn, (O_WRITE | O_CREAT));
    if (!retBool) {
        PRINTOUT(F("seQFF open2 err"), tempFn);
        // sd1_Err("seQCF open4");
        //todo close all other files
        return 0;
    } else {
        MS_DEEP_DBG(F("seQFF opened "), tempFn);
    }

    num_char  = strlen(deszq_line);
    if (num_char) {  // Test could be min size, but this unknown
        MS_DBG(F("seQFF Last POST Failed "),  deszq_line);
        retNum = tgtoutFile.write(deszq_line, num_char);
        if (retNum != num_char) {
            PRINTOUT(F("seQFF tgtoutFile write1 err"), num_char);
            // sd1_Err("seQCF write2");
        }
    } 

    MS_DBG(F("seQFF cpy lines across"));
    while (0 < (num_char = serzQuedFile.fgets(deszq_line,
                                                QUEFILE_MAX_LINE))) {

#if defined(USE_PS_modularSensorsNetwork)
        if ((num_lines>=_sendQueSz_num)&&(MMWGI_SEND_QUE_SZ_NUM_NOP != _sendQueSz_num )) {
            /*Limit sendQueSz on Copy, implicitly this not on creation 
            This is the first pass at limiting the size of the que by dumping the newest.
            FIFO.
            Future may want to keep the latest readings 
            */
            postLogLine((MAX_NUMBER_SENDERS+1),HTTPSTATUS_NC_903);
            num_skipped++;
        } else
#endif // USE_PS_modularSensorsNetwork 
        {

            retNum = tgtoutFile.write(deszq_line, num_char);
            // Squelch last char LF
            deszq_line[sizeof(deszq_line) - 1] = 0;
            MS_DBG(deszq_line);
            if (retNum != num_char) {
                PRINTOUT(F("seQFF tgtoutFile write3 err"), num_char,
                            retNum);
                // sd1_Err("seQFF write4");
                break;
            }
            num_lines++;
        }
    }
    if (num_skipped){ 
        PRINTOUT(F("seQFF sendQue Size "), _sendQueSz_num, F(",queued"),num_lines, F(",latest readings discarded"),num_skipped);
    };
    //Cleanup flushed serzQuedFile to del_file as debugging aid
    if (sd1_card_fatfs.exists(queDelFn)) {
        if (!sd1_card_fatfs.remove(queDelFn)) {
            PRINTOUT(F("seQFF remove2 err"), queDelFn);
            sd1_Err("seQFF err7 remove");
        }
        if (sd1_card_fatfs.exists(queDelFn)) {
            PRINTOUT(F("seQFF err failed remove"), queDelFn);
        }
    }     

    retBool = serzQuedFile.rename(queDelFn);
    if (!retBool) {
        PRINTOUT(F("seQFF REBOOT rename1 err"), queDelFn);
        //Problem - unrecoverable, so reboot
        retBool = serzQuedFile.close();
        if (!retBool) {
            PRINTOUT(F("seQFF close1 failed err"), serzQuedFn);
        }
        forceSysReset(1,4567);
        //sd1_card_fatfs.remove(serzQuedFn);
        // sd1_Err("seQFF rename2");
        //return num_lines;
    } else {
        MS_DBG(F("seQFF cleanup rename "), serzQuedFn, F("to"), queDelFn);

        retBool = serzQuedFile.close();
        if (!retBool) {
            sd1_Err("seQFF serzQuedFile.close2 err");
            return  num_lines;
        } else {MS_DEEP_DBG(F("seQFF close serzQuedFile")); }

        retBool = tgtoutFile.rename(serzQuedFn);
        if (!retBool) {
            sd1_Err("seQFF tgtoutFile.rename err");
            return  num_lines;
        } else {MS_DEEP_DBG(F("seQFF rename "), tempFn, F("to"), serzQuedFn); }

        retBool = tgtoutFile.close();
        if (!retBool) {
            sd1_Err("seQFF tgtoutFile.close1 err");
            return  num_lines;
        } else {MS_DEEP_DBG(F("seQFF closed tgtoutFile")); }
    }

    return  num_lines;
} //serzQuedFlushFile

/*
For serialize, create ASCII CSV records of the form
status,<marked epoch time> n*[<,values>]
*/
#define RDEL_OFLAG (O_WRITE | O_CREAT | O_AT_END)
bool Logger::serzRdel_Line() {
    if (serzRdelFile.open(serzRdelFn_str, RDEL_OFLAG)) {
        uint16_t outputSz;
        // String csvString(Logger::markedEpochTime);
        outputSz = serzRdelFile.print("0,");  // Start READINGS_STATUS
        outputSz += serzRdelFile.print(Logger::markedEpochTime);
        for (uint8_t i = 0; i < getArrayVarCount(); i++) {
            // csvString += ',';
            outputSz += serzRdelFile.print(',' + getValueStringAtI(i));
        }
        outputSz += serzRdelFile.println();
        // setFileAccessTime(serzRdelFile);
        serzRdelFile.close();
        MS_DEEP_DBG(F("serzRdel_Line on"), serzRdelFn_str, F(" at "),
               markedEpochTime, F(" size="), outputSz);
    } else {
        PRINTOUT(F("serzRdel_Line; No file"), serzRdelFn_str);
        return false;
    }
    return true;
}  // Logger::serzLine


/* Deserializer functions

For deserialize, read  ASCII CSV records of the form
<marked epoch time> n*[<,values>]

deszRdelStart()   ~ to open file
deszLine()  to populate
    deszq_epochTime &

*/

/* Find fixed delimeter
 * behave as strchrnul() if goes past end of string
 */
char* Logger::deszFind(const char* in_line, char caller_id) {
    char* retResult = strchr(in_line, DELIM_CHAR2);
    if (NULL != retResult) return retResult;
    MS_DEEP_DBG(F("deszFind NULL found on "), caller_id);
    // For NULL return pointer as per strchrnul
    // should only occur on last search
    return (char*)(in_line + strlen(in_line));

    /*    The strchrnul() function is like strchr() except that if \p c is not
        found in \p s, then it returns a pointer to the null byte at the end
        of \p s, rather than \c NULL. (Glibc, GNU extension.)

        \return The strchrnul() function returns a pointer to the matched
        character, or a pointer to the null byte at the end of \p s (i.e.,
        \c s+strlen(s)) if the character is not found.
    //char *strchrnul(const char *in, int delim_char) */
}


bool Logger::deszRdelStart() {
    deszLinesRead = deszLinesUnsent = 0;

    deszq_nextChar = deszq_line;
    // Open - RD & WR. WR needed to be able to delete when complete.
    if (!serzRdelFile.open(serzRdelFn_str, (O_RDWR | O_CREAT))) {
        PRINTOUT(F("deRS; No file "), serzRdelFn_str);
        return false;
    } else {
        MS_DEEP_DBG(F("deRS open RDWR"), serzRdelFn_str);
    }
    return true;
}

bool Logger::deszQuedStart() {
    deszLinesRead = deszLinesUnsent = 0;

    deszq_nextChar = deszq_line;
    // Open - RD & WR. WR needed to be able to delete when complete.
    // Expect serzQuedFn to be setup in serzQuedStart
    if (!serzQuedFile.open(serzQuedFn, O_RDWR)) {
        // This could be that there aren;t any Qued readings
        MS_DEEP_DBG(F("deQS; No file "), serzQuedFn);
        // sd1_card_fatfs.ls();
        return false;
    } else {
        MS_DEEP_DBG(F("deQS open READ"), serzQuedFn);
    }

    return true;
}
bool Logger::deszLine(File* filep) {
    char* errCheck;
    /* Scan through one line. Expect format
      <ascii Digits>,   representing integer STATUS
      <ascii Digitis>, represnting inteeger marked Epoch Time
      .... <ascii Digits> representing reading values

    Not renetrant, assumption: there is only deserialize going on at a time.
    Uses
    char    deszq_line[],
    uint8_t deszq_status
    long    deszq_epochTime
    char   *deszq_nextChar
            deszq_nextCharSz
    */

    uint16_t num_char = filep->fgets(deszq_line, QUEFILE_MAX_LINE);
    char*    orig_nextChar;

    if (0 == num_char) return false;
    deszLinesRead++;
    // First is the Status of record
    deszq_status = strtol(deszq_line, &errCheck, 10);
    if (errCheck == deszq_line) {
        PRINTOUT(F("deszLine Status err'"), deszq_line, F("'"));
        return false;  // EIO;
    }
    // Find next DELIM and go past it
    deszq_nextChar = 1 + deszFind(deszq_line, '1');
    if (deszq_nextChar == deszq_line) {
        PRINTOUT(F("deszLine epoch start not found"), deszq_line, F("'"));
        deszq_nextCharSz = 0;
        return false;
    }
    // Second is the epochTime,
    deszq_epochTime = strtol(deszq_nextChar, &errCheck, 10);
    if (errCheck == deszq_line) {
        PRINTOUT(F("deszLine Epoch err'"), deszq_line, F("'"));
        return false;  // EIO;
    }
    // Find next DELIM and go past it
    orig_nextChar  = deszq_nextChar;
    deszq_nextChar = 1 + deszFind(deszq_nextChar, '2');
    if (orig_nextChar == deszq_nextChar) {
        PRINTOUT(F("deszLine readung start not found"), deszq_line, F("'"));
        deszq_nextCharSz = 0;
        return false;
    }
    // Find sz of this field
    char* nextCharEnd = deszFind(deszq_nextChar, '3');
    deszq_nextCharSz  = nextCharEnd - deszq_nextChar;

    deszq_timeVariant_sz = strlen(deszq_nextChar) - 1;
    MS_DBG(F("deszLine Reading sz"), deszq_timeVariant_sz, F(":"), deszq_nextChar,
           F(":"));
    return true;
}

bool Logger::deszqNextCh(void) {
    char* deszq_old = deszq_nextChar;
    // Find next DELIM and go past it
    deszq_nextChar = 1 + deszFind(deszq_nextChar, 'L');
    if ((deszq_old == deszq_nextChar)) {
        deszq_nextCharSz = 0;
        PRINTOUT(F("deszqNextCh 1error:"), deszq_nextChar, F("'"));
        return false;
    }
    /* Find sz of this field
        either
    <value>,[..]
    <value><CR><LF>EOL
    EOF
    */
    char* nextCharEnd = strchr(deszq_nextChar, DELIM_CHAR2);
    deszq_nextCharSz  = strlen(deszq_nextChar);
    if ((0 == deszq_nextCharSz)) {
        // Found end of line
        MS_DBG(F("dSRN unexpected EOL "));
        return false;
    } else if (NULL == nextCharEnd) {
        // Found <value>EOF ~ nextSr_sz is valid
        deszq_nextCharSz -= 1;  // take off turds <LF>
        MS_DEEP_DBG(F("dSRN info "), deszq_nextCharSz, " '", deszq_nextChar,
                    "'");
        // return true
    } else {
        // expect to have found <value>,[..]
        // if found ,, then invalid and finish
        deszq_nextCharSz = nextCharEnd - deszq_nextChar;
        if (0 == deszq_nextCharSz) {
            MS_DEEP_DBG(F("dSRN unexpected 0 bytes "));
            return false;
        }
    }
    return true;
}

bool Logger::deszRdelClose(bool deleteFile) {
    bool retVal = false;

    if (!(retVal = serzRdelFile.close())) {
        PRINTOUT(F("deSRC close err"), serzRdelFn_str);
        sd1_Err("serzBegin err close");
    } else {
        MS_DEEP_DBG(F("deSRC closed"), serzRdelFn_str);
    }
    if (deleteFile) {
        // if (!(retVal = serzRdelFile.remove())) {
        if (!(retVal = sd1_card_fatfs.remove(serzRdelFn_str))) {
            PRINTOUT(F("deSRC remove err"), serzRdelFn_str);
            sd1_Err("serzBegin err remove");
        }
        MS_DEEP_DBG(F("deSRC removed"), serzRdelFn_str);
    }

    return retVal;
}

/* Prototyping des example
 */

uint16_t serialCnt = 0;
// SdFat    sdfat_phy;
// SdFile   rootDir;
bool Logger::deszDbg(void) {
    // char* next_token;
#define TEMPBUF_SZ 37
    char tempBuffer[TEMPBUF_SZ] = "";
    if (++serialCnt >= SERIALIZE_sendEveryX_NUM) {
        String d_str(80);
        serialCnt = 0;
        deszRdelStart();
        while (deszRdelLine()) {
            d_str = formatDateTime_ISO8601(deszq_epochTime) + ';';
            // next_token = find_chars_or_comment(deszq_nextChar,
            // DELIM_CHAR2);
            tempBuffer[0] = 0;
            strncat(tempBuffer, deszq_nextChar, deszq_nextCharSz);
            strcat(tempBuffer, ";");
            // PRINTOUT("Sn=", tempBuffer);
            d_str.concat(tempBuffer);
            // getline
            while (deszqNextCh()) {
                tempBuffer[0] = 0;
                strncat(tempBuffer, deszq_nextChar, deszq_nextCharSz);
                strcat(tempBuffer, ";");
                d_str.concat(tempBuffer);
                // PRINTOUT("t='", tempBuffer, F("'"));
            }
            PRINTOUT("L=", d_str, "Stat=", deszq_status);
        }
        deszRdelClose(true);  // Delete serial file
    }
    return true;
}

bool Logger::postLogOpen(const char* postLogNam_str) {
    bool retVal = false;
#if defined MS_LOGGERBASE_POSTS
    // Generate the file name from logger ID and date
    // Create rotating log of 4 chars YYMM - formatDateTime is YYYY MM DD
     String nameTemp = formatDateTime_str(getNowEpochTz());

    // Drop middle _ and get YYMM
    String fileName = String(postLogNam_str + nameTemp.substring(2, 4) + nameTemp.substring(5, 7) + ".log");

    // Convert the string filename to a character file name for SdFat
    uint16_t fileNameLength = fileName.length()+2;
    MS_DBG(F("PLO postLog file"), fileName, F("res len"),fileNameLength);
    char    charFileName[fileNameLength];
    fileName.toCharArray(charFileName, fileNameLength);

    // Attempt to open an existing file
    retVal = postsLogHndl.open(charFileName, (O_WRITE | O_AT_END));
    if (!retVal) {
        retVal = postsLogHndl.open(charFileName,
                                   (O_CREAT | O_WRITE | O_AT_END));
        if (!retVal) {
            PRINTOUT(F("logPLO err opening"), charFileName);

        } else {
            setFileTimestampTz(postsLogHndl, T_CREATE);
            MS_DBG(F("logPLO new file"), charFileName);
        }
    }
#endif  // MS_LOGGERBASE_POSTS
    return retVal;
}
bool Logger::postLogOpen() {
    bool     retVal    = false;
#if defined MS_LOGGERBASE_POSTS
    retVal =postLogOpen(postsLogFn_str);
#endif // 
    return retVal;
}
void        Logger::postLogClose() {
#if defined MS_LOGGERBASE_POSTS

    setFileTimestampTz(postsLogHndl, (T_WRITE));  //| T_ACCESS
    postsLogHndl.close();


#endif  // MS_LOGGERBASE_POSTS
}

void Logger::postLogLine(uint32_t tmr_ms, int16_t rspParam) {
// If debug ...keep record
#if defined MS_LOGGERBASE_POSTS
#if 0
    if (0 == postsLogHndl.print(getNowEpochUTC())) {
        PRINTOUT(F("publishDataQuedToRemote postsLog err"));
    }
#else

    char tempBuffer[TEMP_BUFFER_SZ];
    //Print internal time
    formatDateTime_str(getNowEpochTz())
        .toCharArray(tempBuffer, TEMP_BUFFER_SZ);
    postsLogHndl.print(tempBuffer);
#endif
    postsLogHndl.print(F(",POST,"));
    itoa(rspParam, tempBuffer, 10);
    postsLogHndl.print(tempBuffer);
        postsLogHndl.print(F(","));
        itoa(tmr_ms, tempBuffer, 10);
        postsLogHndl.print(tempBuffer);
        postsLogHndl.print(F(","));
    postsLogHndl.print(deszq_line);
#endif  //#if defined MS_LOGGERBASE_POSTS
}

void Logger::postLogLine(const char *logMsg,bool addCRNL) {
#if defined MS_LOGGERBASE_POSTS 
    bool wasOpen =true;   
    if (!postsLogHndl.isOpen()) {
        wasOpen=false;
        if (!postLogOpen()) {
            PRINTOUT(F("postLogLine can't open file"));      
            //TODO possible reboot         
            return;
        } 
    }
    char tempBuffer[TEMP_BUFFER_SZ];
    //Print internal time
    formatDateTime_str(getNowEpochTz())
        .toCharArray(tempBuffer, TEMP_BUFFER_SZ);    
    postsLogHndl.print(tempBuffer);
    postsLogHndl.print(F(",MSG,"));
    postsLogHndl.print(logMsg);
    if (addCRNL)postsLogHndl.println();

    if (!wasOpen) postLogClose();
#endif //MS_LOGGERBASE_POSTS
}
/*
Cleanup if necessary
*/

bool Logger::listFile(File* filep, char* fn_str, char* uid) {
    char    loc_line[QUEFILE_MAX_LINE];
    int16_t num_char;
    int16_t num_cnt = 0;

    if (!filep->open(fn_str, O_READ)) {
        PRINTOUT(F("listFile; No file "), fn_str);
        sd1_Err("listFile: no file2");
        return false;
    } else {
        MS_DBG(F("listFile"), fn_str, uid, F("<beg>"));
    }

    while (0 < (num_char = filep->fgets(loc_line, QUEFILE_MAX_LINE))) {
        PRINTOUT(++num_cnt, loc_line);
    }
    if (0 > num_char) {
        PRINTOUT(F("listFile err"), num_char);
        sd1_Err("listFile err2");
    }
    if (!filep->close()) {
        PRINTOUT(F("listFile; close err "), fn_str);
        sd1_Err("listFile close err2");
        return false;
    }
    MS_DBG(F("listFile"), uid, F("<end>"));
    return true;
}

/* This tests all the primitives used to access the SD card.
 */
bool Logger::serzBegin(void) {
    bool dslStat_bool;

    MS_DBG(F("serzBegin list1---"));
    if (!sd1_card_fatfs.ls()) {
        // MS_DBG(F("serzBegin ls err"));
        sd1_Err("serzBegin err ls");
    } else {
        MS_DBG(F("---1Complete"));
    }

    // Test  RDELAY.TXT
    serzRdelFile.open(serzRdelFn_str, RDEL_OFLAG);
    serzRdelFile.println(F("1,1595653100,1,4.730,-38"));
    serzRdelFile.println(F("1,1595653200,2,4.730,-38"));
    serzRdelFile.close();

    serzRdelFile.open(serzRdelFn_str, RDEL_OFLAG);
    serzRdelFile.println(F("1,1595653300,3,4.730,-38"));
    serzRdelFile.println(F("1,1595653400,4,4.730,-38"));
    serzRdelFile.close();

    PRINTOUT(F("serzBegin list2---"));
    if (!sd1_card_fatfs.ls()) {
        // MS_DBG(F("serzBegin ls err"));
        sd1_Err("serzBegin err ls");
    } else {
        PRINTOUT(F("---2Complete"));
    }
    listFile(&serzRdelFile, (char*)serzRdelFn_str, (char*)"1");


    deszRdelStart();
    int16_t cnt_num = 0;
    while (0 < (dslStat_bool = deszRdelLine())) {
        PRINTOUT(++cnt_num, F("] "), dslStat_bool, deszq_line);
    }

    deszRdelClose(true);  // Test for delete
    PRINTOUT(F("serzBegin list3---"));
    if (!sd1_card_fatfs.ls()) {
        // MS_DBG(F("serzBegin ls err"));
        sd1_Err("serzBegin err ls");
    } else {
        PRINTOUT(F("---3Complete"));
    }

// Test QUED algorithims ~ use QUE7.txt
#define QUE_TST '7'
#define TESTQ_FN_STR "QUE7.TXT"
    MS_DBG(F("TESTQ START"));
    if (sd1_card_fatfs.exists(TESTQ_FN_STR)) {
        // PRINTOUT(F("serzBegin removing "), TESTQ_FN_STR);
        if (!sd1_card_fatfs.remove(TESTQ_FN_STR)) {
            PRINTOUT(F("serzBegin err remove"), TESTQ_FN_STR);
            sd1_Err("serzBegin remove");
        }
    } else {
        MS_DBG(F("serzBegin no "), TESTQ_FN_STR);
    }
    // Test1 ** QUED new file name & update
    MS_DBG(F("TESTQ1"));
    serzQuedStart((char)(QUE_TST));
    serzQuedFile.println(F("1,1595654100,1,4.7,-38"));
    serzQuedFile.println(F("1,1595654200,2,4.7,-38"));
    serzQuedCloseFile(false);


    // Test2 ** QUED file update
    MS_DBG(F("TESTQ2"));
    if (!serzQuedFile.open(serzQuedFn, (O_WRITE | O_AT_END))) {
        // Could be that there are no retrys.
        PRINTOUT(F("serzQuedFile.open err"), serzQuedFn);
        sd1_Err("serzQuedFile.open err2");
        return false;
    } else {
        PRINTOUT(F("Testq2 Opened"), serzQuedFn);
    }
    serzQuedFile.println(F("1,1595654300,3,4.7,-38"));
    serzQuedFile.println(F("1,1595654400,4,4.7,-38"));
    if (!serzQuedCloseFile(false)) return false;

    PRINTOUT(F("serzBegin list4---"));
    if (!sd1_card_fatfs.ls()) {
        // MS_DBG(F("serzBegin ls err"));
        sd1_Err("serzBegin err4 ls");
        return false;
    } else {
        PRINTOUT(F("---4Complete"));
    }
    listFile(&serzQuedFile, serzQuedFn, (char*)"2");

    // Test3 ** QUED file rollover
    MS_DBG(F("TESTQ3"));
    if (!deszQuedStart()) return false;

    dslStat_bool = deszQuedLine();
    MS_DBG(F("1: deszq_line"), dslStat_bool, deszq_line);
    if (!dslStat_bool) return false;
    dslStat_bool = deszQuedLine();
    MS_DBG(F("2: deszq_line"), dslStat_bool, deszq_line);
    if (!dslStat_bool) return false;
    // only the 1: should be dropped
    dslStat_bool = serzQuedCloseFile(true);
    PRINTOUT(F("serzBegin list5---"));
    if (!sd1_card_fatfs.ls()) {
        // MS_DBG(F("serzBegin ls err"));
        sd1_Err("serzBegin err5 ls");
        return false;
    } else {
        PRINTOUT(F("---5Complete"));
    }
    listFile(&serzQuedFile, serzQuedFn, (char*)"3");
    if (!dslStat_bool) return false;

    if (sd1_card_fatfs.exists(serzQuedFn)) {
        PRINTOUT(F("serzBegin removing "), serzQuedFn);
        if (!sd1_card_fatfs.remove(serzQuedFn)) {
            PRINTOUT(F("serzBegin err remove"), serzQuedFn);
            sd1_Err("serzBegin err6 remove");
        }
    } else {
        PRINTOUT(F("serzBegin no "), TEMP_BASE_FN_STR);
    }

    // Cleanup

    MS_DBG(F("TESTQ CLEANUP"));
    if (sd1_card_fatfs.exists(TEMP_BASE_FN_STR)) {
        PRINTOUT(F("serzBegin removing "), TEMP_BASE_FN_STR);
        if (!sd1_card_fatfs.remove(TEMP_BASE_FN_STR)) {
            PRINTOUT(F("serzBegin err remove"), TEMP_BASE_FN_STR);
            sd1_Err("serzBegin err6 remove");
        }
    } else {
        MS_DBG(F("serzBegin no "), TEMP_BASE_FN_STR);
    } /* */
    MS_DBG(F("TESTQ END END END \n\n"));
    return true;
}

// Convert a date-time object into a formatted string
String Logger::formatDateTime_str(DateTime& dt) {
    String dateTimeStr;
    dt.addToString(dateTimeStr);
    return dateTimeStr;
}

// Convert an epoch time into a formatted string
String Logger::formatDateTime_str(uint32_t epochTime) {
    DateTimeClass(dt1, epochTime);
    return formatDateTime_str(dt1);
}
// End LoggerBaseExtCpp.h
