// SARA5_controller.h
#ifndef SARA5_CONTROLLER_H
#define SARA5_CONTROLLER_H

#include <SoftwareSerial.h>
#include <SparkFun_u-blox_SARA-R5_Arduino_Library.h>
#include <Arduino.h>
#include <String.h>


//DEFINE SARA SERIAL (hardware serial for ESP UART2)
//#define saraSerial Serial2
#define rx_pin 27
#define tx_pin 26


// Define the error codes as an enum type
typedef enum
{
    SARA5_ERROR_INVALID = -1,
    SARA5_ERROR_SUCCESS = 0,
    SARA5_ERROR_TIMEOUT = 1,
    SARA5_ERROR_NOT_CONNECTED = 2,
    SARA5_ERROR_ERROR = 3
} SARA5_Error_t;

struct GPSData {
    double latitude;
    double longitude;
};

// Declare the SARA5_controller class
class SARA5_controller
{
public:
    // Constructor and Destructor
    SARA5_controller();
    ~SARA5_controller();

    // Member functions
    SARA5_Error_t SARA5_setup(bool debug);
    SARA5_Error_t read_GNNS(bool debug);
    SARA5_Error_t GPSpowerOFF();
    SARA5_Error_t GNSS_publish(bool debug);
    SARA5_Error_t _read();
    SARA5_Error_t Console_mode(const char *command, char *responseDest); // New function for entering console mode
    SARA5_Error_t Wake_device();

    //get func
    GPSData getGPSData() const { return gpsData; }
    String getWriteAPIKey() const { return myWriteAPIKey; }

private:
    //software
    SoftwareSerial saraSerial;

    SARA_R5 mySARA; //Parent class

    String myWriteAPIKey = "NTAEMW4QKGPB8ZFF"; // Change this to your API key

    String serverName = "api.thingspeak.com"; // Domain Name for HTTP POST / GET
    

    //GPS INFOMATION STORAGE:
    String utc;
    char fixStatus;
    char mode;
    GPSData gpsData; 

    // Private member that might be used in the implementations
    int _connectionStatus;
    int _GPSStatus;

    //functions
    void printGPS(const String& utc, const char& fixStatus, const GPSData& gps, const char& mode);
    void parseRMCString(const String& rmcData, String& utc, char& fixStatus, GPSData& gps, char& mode);
    double convertToDecimalDegrees(const String& degreesMinutes, char hemisphere);
    
};

#endif // SARA5_CONTROLLER_H
