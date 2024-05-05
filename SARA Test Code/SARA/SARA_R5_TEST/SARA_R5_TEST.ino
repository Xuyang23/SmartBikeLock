#include "SARA5_controller.h"
#define _powerPin D2

#define GnnsIndicator D3
#define PostIndicator D5

SARA5_controller m_Controller;

void setup() {
  Serial.begin(115200); // Start the serial console
  pinMode(_powerPin, OUTPUT);
  pinMode(GnnsIndicator, OUTPUT);
  pinMode(PostIndicator, OUTPUT);
  digitalWrite(_powerPin, HIGH); //POWER DOWN!!!

  // Wait for user to press key to begin
  Serial.println(F("Press a key to start device"));
  while (!Serial.available()) // Wait for the user to press a key (send any serial character)
    ;
  while (Serial.available()) // Empty the serial RX buffer
    Serial.read();

  Serial.println("Waiting 5ses for device to startup...");
  digitalWrite(_powerPin, LOW); //POWER UP!!!
  delay(2000); //2secs low for power UP 
  digitalWrite(_powerPin, HIGH);
  delay(100);
  digitalWrite(_powerPin, LOW);
  delay(3000);

  // Wait for user to press key to begin
  Serial.println(F("Press a key to start code"));
  while (!Serial.available()) // Wait for the user to press a key (send any serial character)
    ;
  while (Serial.available()) // Empty the serial RX buffer
    Serial.read();

  // put your setup code here, to run once: FALSE/TRUE FOR DEBUG 
  if(m_Controller.SARA5_setup(true) == SARA5_ERROR_SUCCESS){
    Serial.println(F("Setup Sucess!"));
  }else{
    Serial.println(F("Setup Failed damn..."));
  }

  // Display the menu options after setup
  displayMenu();
}

void loop() {
  // Check if data is available to read from the serial buffer
  if (Serial.available() > 0) {
    // Read the incoming byte:
    char receivedChar = Serial.read();
    // Compare against expected commands
    switch (receivedChar) {
      case 'G':
        Serial.println("G");
        m_Controller.read_GNNS(true);
        break;
      case 'P':
        Serial.println("P");
        //handleHTTPPost();
        Serial.println(m_Controller.GNSS_publish(true));
        break;
      case 'I':
        Serial.println("I");
        m_Controller.GPSpowerOFF(); 
        break;
      case 'C':
        {
        Serial.println("C");
        //openConsole();
        //Wait for byte
        delay(1);
        Serial.println("Enter you command (example: +URAT?): ");
        while (Serial.available()) Serial.read(); // Empty the serial RX buffer
        while (!Serial.available()) // Wait for the user to press a key (send any serial character)
          ;
        char command[256]; // Buffer to store the command
        char response[256]; // Buffer to store the command

        size_t length = Serial.readBytesUntil('\n', command, sizeof(command) - 1);
        command[length] = '\0'; // Ensure the string is null-terminated

        Serial.print("Sending: ");
        Serial.println(command);
        
        m_Controller.Console_mode(command, response);

        Serial.print("Response: ");
        Serial.println(response);
        }
        break;
      case 'O':
        Serial.println("O");
        digitalWrite(_powerPin, HIGH);
        delay(500);
        digitalWrite(_powerPin, LOW);
        delay(2000);
        break;

      case 'F':
        {
        //start timing
        digitalWrite(GnnsIndicator, LOW);
        digitalWrite(PostIndicator, LOW);
        long int t1 = millis();
        //wake device Indicate waking up
        m_Controller.Wake_device();
        //start GNNS Indicate GNNS
        digitalWrite(GnnsIndicator, HIGH);
        m_Controller.read_GNNS(true);
        //power off GNNS Indicate GNNS off
        m_Controller.GPSpowerOFF();
        digitalWrite(GnnsIndicator, LOW);
        //Post GNNS indicate GNNS POST
        digitalWrite(PostIndicator, HIGH);
        m_Controller.GNSS_publish(true);
        digitalWrite(PostIndicator, LOW);
        //Display time it took for the whole case to run
        long int t2 = millis();
        Serial.print("Task took: "); Serial.print(t2-t1); Serial.println(" millisecounds");
        }
        break;
      default:
        // Handle unexpected input
        Serial.println("");
        Serial.println("Invalid input, please try again.");
    }
    //clear serial
    Serial.read();
    // Display the menu again after an action has been taken
    displayMenu();
  }
}

void displayMenu() {
  // Display available options
  Serial.println("Press 'G' for GNSS");
  Serial.println("Press 'I' for GNSS Power Off");
  Serial.println("Press 'P' for HTTP POST");
  Serial.println("Press 'C' for Console");
  Serial.println("Press 'O' for Power up (from sleep or off)");
  Serial.println("Press 'F' for full cycle: wakedevice ->GNNS fix->GNNS post->GNNS off with indicator pins");
  Serial.println("Enter your choice:");
}
