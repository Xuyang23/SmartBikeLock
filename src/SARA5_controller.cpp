// SARA5_controller.cpp
#include "SARA5_controller.h"

// Constructor
SARA5_controller::SARA5_controller()
{
  _connectionStatus = 0;  // Assume 0 is disconnected
  _GPSStatus = 0;
}

// Destructor
SARA5_controller::~SARA5_controller()
{
    // Cleanup resources if necessary
}

//.....------------------------Public------------------------.....
// Setup the SARA5 module
SARA5_Error_t SARA5_controller::SARA5_setup(bool debug)
{   
    String currentOperator = "";

    if (debug == true) mySARA.enableDebugging();

    // Initialize the SARA //SOFTWARE
    Serial.println(F("Configuring SoftwareSerial saraSerial"));
    saraSerial.begin(9600, SWSERIAL_8N1, rx_pin, tx_pin, false);
    saraSerial.end();

    if (mySARA.begin(saraSerial, 9600))
    {
      Serial.println(F("SARA-R5 connected!"));
    }
    else
    {
      Serial.println(F("Not connected to sara..."));
      return SARA5_ERROR_ERROR;
    }

    //Setup Internet
    // First check to see if we're connected to an operator:
    // Constants
    const int MAX_ATTEMPTS = 10;
    // Variables
    int attempts = 0;
    // Main Loop or Function
    while (attempts < MAX_ATTEMPTS)
    {
        if (mySARA.getOperator(&currentOperator) == SARA_R5_SUCCESS)
        {
            Serial.print(F("Connected to: "));
            Serial.println(currentOperator);
            break;  // Exit the loop on success
        }
        else
        {
            Serial.println(F("The SARA is not yet connected to an operator. Retrying..."));
            Serial.printf("Failed %d", attempts);
            Serial.println("");
            attempts++;  // Increment the attempt counter

            // Add a delay of 5 seconds between retries
            Serial.println("Waiting 5 secounds");
            delay(5000);  // Delay for 5 seconds before retrying

            if (attempts >= MAX_ATTEMPTS)
            {
                Serial.println(F("Failed to connect after 5 attempts."));
                return SARA5_ERROR_ERROR;
            }
        }
    }

    // Deactivate the PSD profile - in case one is already active
    if (mySARA.performPDPaction(0, SARA_R5_PSD_ACTION_DEACTIVATE) != SARA_R5_SUCCESS)
    {
      Serial.println(F("Warning: performPDPaction (deactivate profile) failed. Probably because no profile was active."));
    }

    // Load the PSD profile from NVM - these were saved by a previous example
    if (mySARA.performPDPaction(0, SARA_R5_PSD_ACTION_LOAD) != SARA_R5_SUCCESS)
    {
      Serial.println(F("performPDPaction (load from NVM) failed! Retrying..."));
      return SARA5_ERROR_ERROR;
    }

    // Activate the PSD profile
    if (mySARA.performPDPaction(0, SARA_R5_PSD_ACTION_ACTIVATE) != SARA_R5_SUCCESS)
    {
      Serial.println(F("performPDPaction (activate profile) failed! Retrying..."));
      return SARA5_ERROR_ERROR;
    }

    // Reset HTTP profile 0
    mySARA.resetHTTPprofile(0);
    
    // Set the server name
    mySARA.setHTTPserverName(0, serverName);
    
    // Use HTTPS
    mySARA.setHTTPsecure(0, false); // Setting this to true causes the POST / GET to fail. Not sure why...

    // Set a callback to process the HTTP command result
    //mySARA.setHTTPCommandCallback(GETcallback_func);

    //SETUP GNNS
    //activate the unsolicited aiding result.
    //mySARA.gpsEnableRmc(); //enable RMC
    if(!mySARA.sendCustomCommandWithResponse("+UGIND=1", "OK", nullptr, 5000) == SARA_R5_SUCCESS)
    {
      Serial.println(F("Custom GNNS: +UGIND=1 failed..."));
      return SARA5_ERROR_ERROR;
    }
    /*
    Setup: AT+UGRMC=1
           AT+UGGLL=1
           AT+UGGSV=1
           AT+UGGGA=1
           ONCE PERSISTED THORUGH POWERDOWN
    */
    //Start the GNSS with GPS+SBAS+GLONASS systems and local aiding.
    if(!mySARA.sendCustomCommandWithResponse("+UGPS=1,1,67", "OK", nullptr, 5000) == SARA_R5_SUCCESS)
    {
      Serial.println(F("Custom GNNS: +UGPS=1,1,67 failed..."));
      return SARA5_ERROR_ERROR;
    }
    _GPSStatus = 1;

    //Setup sleepmode:
    if(!mySARA.sendCustomCommandWithResponse("+UPSV=1,2000", "OK", nullptr, 5000) == SARA_R5_SUCCESS)
    {
      Serial.println(F("Custom sleepmode: +UPSV=1,2000 failed..."));
      return SARA5_ERROR_ERROR;
    }
    
    //Done
    _connectionStatus = 1; // Assume 1 is connected
    return SARA5_ERROR_SUCCESS;
}

// Read GNNS
SARA5_Error_t SARA5_controller::read_GNNS(bool debug)
{
    bool Fix = false;

    if (_connectionStatus != 1) {
        return SARA5_ERROR_NOT_CONNECTED;
    }

    //Check for Power:
    if(_GPSStatus == 0){
      if(!mySARA.sendCustomCommandWithResponse("+UGPS=1,1,67", "OK", nullptr, 5000) == SARA_R5_SUCCESS)
      {
        Serial.println(F("Custom GNNS: +UGPS=1,1,67 failed..."));
        return SARA5_ERROR_ERROR;
      }
      _GPSStatus = 1;
    }

    //Start GNS (GNS UNTIL FIX!)
    char resp[256];
    String firstUtc = "";
    while(!Fix){
      if(!mySARA.sendCustomCommandWithResponse("+UGRMC?", "OK", resp, 5000) == SARA_R5_SUCCESS)
      {
        Serial.println(F("Custom GNNS: +UGRMC? failed..."));
        return SARA5_ERROR_ERROR;
      }
      String respStr = String(resp);
      parseRMCString(respStr, utc, fixStatus, gpsData, mode);
      if(firstUtc == "") {
        firstUtc = utc;
        Serial.println("UTC FIRST UPDATED");
      }
      if (debug==true) printGPS(utc, fixStatus, gpsData, mode);
      if (fixStatus == 'A') Fix = true;
      delay(1500); //IF NO FIX DO IT AGAIN
    }
    int timeItTook = utc.toInt() - firstUtc.toInt();
    Serial.print("UTC took: ");
    Serial.println(timeItTook);

    //upload aiding:
    if(!mySARA.sendCustomCommandWithResponse("+UGAOS=0", "OK", nullptr, 5000) == SARA_R5_SUCCESS)
    {
      Serial.println(F("Custom GNNS: +UGAOS=0 failed..."));
      return SARA5_ERROR_ERROR;
    }

    // Implement the publishing logic
    return SARA5_ERROR_SUCCESS;
}

// Gps power off
SARA5_Error_t SARA5_controller::GPSpowerOFF()
{
    if(!mySARA.sendCustomCommandWithResponse("+UGPS=0", "OK", nullptr, 5000) == SARA_R5_SUCCESS)
    {
      Serial.println(F("Custom GNNS: +UGPS=0 failed..."));
      return SARA5_ERROR_ERROR;
    }
    _GPSStatus = 0;

    // Implement the reading logic
    return SARA5_ERROR_SUCCESS;
}

// Publish data via GNSS
SARA5_Error_t SARA5_controller::GNSS_publish(bool debug)
{
    if (_connectionStatus != 1) {
        return SARA5_ERROR_NOT_CONNECTED;
    }

    String httpRequestData = "api_key=" + myWriteAPIKey + "&field1=" + String(gpsData.longitude, 6) + "&field2=" + String(gpsData.latitude, 6);
    if (debug == true){
      Serial.println("Sending this: ");
      Serial.print("Longitude: ");
      Serial.println(String(gpsData.longitude, 6));
      Serial.print("Latitude: ");
      Serial.println(gpsData.latitude, 6);
    }
    //send post
    if (mySARA.sendHTTPPOSTdata(0, "/update", "post_response.txt", httpRequestData, SARA_R5_HTTP_CONTENT_APPLICATION_X_WWW) != SARA_R5_SUCCESS)
    {
      return SARA5_ERROR_ERROR;
    }
    

    // Implement the publishing logic
    return SARA5_ERROR_SUCCESS;
}

// Wake Device
SARA5_Error_t SARA5_controller::Wake_device()
{
    if (_connectionStatus != 1) {
        return SARA5_ERROR_NOT_CONNECTED;
    }

    if(!mySARA.sendCustomCommandWithResponse("", "OK", nullptr, 5000) == SARA_R5_SUCCESS)
    {
      Serial.println(F("Custom wakedevice failed..."));
      return SARA5_ERROR_ERROR;
    }

    // Implement the reading logic
    return SARA5_ERROR_SUCCESS;
}

// Read data
SARA5_Error_t SARA5_controller::_read()
{
    if (_connectionStatus != 1) {
        return SARA5_ERROR_NOT_CONNECTED;
    }

    // Implement the reading logic
    return SARA5_ERROR_SUCCESS;
}

//Enter console
SARA5_Error_t SARA5_controller::Console_mode(const char *command, char *responseDest) {
    if (_connectionStatus != 1) {
        return SARA5_ERROR_NOT_CONNECTED;
    }

    //Console:
    if(mySARA.sendCustomCommandWithResponse(command, "OK", responseDest, 5000) == SARA_R5_SUCCESS)
    {
      return SARA5_ERROR_SUCCESS;
    }
    else
    {
      return SARA5_ERROR_ERROR;
    }
}

//.....------------------------Private------------------------.....
void SARA5_controller::parseRMCString(const String& rmcData, String& utc, char& fixStatus, GPSData& gps, char& mode) 
{
    int firstComma = rmcData.indexOf(',');
    int secondComma = rmcData.indexOf(',', firstComma + 1);

    // Extract and parse the RMC sentence
    String rmcSentence = rmcData.substring(secondComma + 1);
    int tokenPos[13]; // Array to store positions of tokens
    int tokenCount = 0;

    // Tokenizing the RMC sentence
    tokenPos[tokenCount++] = -1; // Starting position before the first token
    for (int i = 0; i < rmcSentence.length() && tokenCount < 13; i++) {
        if (rmcSentence[i] == ',') {
            tokenPos[tokenCount++] = i;
        }
    }

    // Assigning variables
    utc = rmcSentence.substring(tokenPos[0] + 1, tokenPos[1]);
    fixStatus = rmcSentence[tokenPos[1] + 1];
    mode = rmcSentence[tokenPos[11] + 1]; // Assuming the mode is in the usual place

    // Converting latitude and longitude from degrees and minutes to decimal
    String lat = rmcSentence.substring(tokenPos[2] + 1, tokenPos[3]);
    char latHemisphere = rmcSentence[tokenPos[3] + 1];
    String lon = rmcSentence.substring(tokenPos[4] + 1, tokenPos[5]);
    char lonHemisphere = rmcSentence[tokenPos[5] + 1];

    gps.latitude = convertToDecimalDegrees(lat, latHemisphere);
    gps.longitude = convertToDecimalDegrees(lon, lonHemisphere);
}

double SARA5_controller::convertToDecimalDegrees(const String& degreesMinutes, char hemisphere) 
{
    int pointPos = degreesMinutes.indexOf('.');
    int degrees = degreesMinutes.substring(0, pointPos - 2).toInt();
    double minutes = degreesMinutes.substring(pointPos - 2).toDouble();
    double decimalDegrees = degrees + (minutes / 60.0);
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimalDegrees = -decimalDegrees;
    }
    return decimalDegrees;
}

void SARA5_controller::printGPS(const String& utc, const char& fixStatus, const GPSData& gps, const char& mode)
{
    Serial.println("GPS Data Summary:");
    Serial.print("UTC Time: ");
    Serial.println(utc);

    Serial.print("Fix Status: ");
    if (fixStatus == 'A') {
        Serial.println("Active");
    } else if (fixStatus == 'V') {
        Serial.println("Void");
    } else {
        Serial.print("Unknown (");
        Serial.print(fixStatus);
        Serial.println(")");
    }

    Serial.print("Latitude: ");
    Serial.print(gps.latitude, 6); // Printing with 6 decimal places
    Serial.println(" degrees");

    Serial.print("Longitude: ");
    Serial.print(gps.longitude, 6); // Printing with 6 decimal places
    Serial.println(" degrees");

    Serial.print("Mode: ");
    switch(mode) {
        case 'A':
            Serial.println("Autonomous");
            break;
        case 'D':
            Serial.println("Differential");
            break;
        case 'E':
            Serial.println("Estimated (dead reckoning)");
            break;
        case 'M':
            Serial.println("Manual Input Mode");
            break;
        case 'S':
            Serial.println("Simulated Mode");
            break;
        case 'N':
            Serial.println("Data Not Valid");
            break;
        default:
            Serial.print("Unknown (");
            Serial.print(mode);
            Serial.println(")");
    }
    Serial.println("---------------------------");
}

