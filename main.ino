#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_NeoTrellis.h"
#include "Adafruit_seesaw.h"
#include <SPI.h>
#include <RH_RF69.h>

// Constants and Definitions
#define SCREEN_WIDTH 128       // OLED display width
#define SCREEN_HEIGHT 64       // OLED display height
#define OLED_RESET -1          // Reset pin (unused here)
#define SCREEN_ADDRESS 0x3C    // I2C address for the OLED

#define SEESAW_ADDR 0x49       // I2C address for the Seesaw chip

#define Y_DIM 4                // Number of rows on the NeoTrellis
#define X_DIM 4                // Number of columns on the NeoTrellis

// Button mappings for the Seesaw rotary encoder
#define SS_SWITCH_UP     2
#define SS_SWITCH_DOWN   4
#define SS_SWITCH_LEFT   3
#define SS_SWITCH_RIGHT  5
#define SS_SWITCH_SELECT 1

// Radio Frequency definitions for RFM69
#define RF69_FREQ 915.0        // Frequency in MHz
#define RFM69_CS   16          // Chip select pin
#define RFM69_INT  21          // Interrupt pin
#define RFM69_RST  17          // Reset pin
#define LED        LED_BUILTIN // Built-in LED

// Adafruit logo bitmap for OLED display
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] = {
  0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000
};

// Global Objects for hardware components
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // OLED display object
Adafruit_NeoTrellis trellis; // NeoTrellis object
Adafruit_seesaw ss;          // Seesaw object (for encoder)
RH_RF69 rf69(RFM69_CS, RFM69_INT); // RFM69 radio object

// Global Variables
int32_t encoder_position; // Tracks rotary encoder position
const int NUM_MENU_ITEMS = 4;  // Number of menu items
const char* menu_items[] = {"Connection", "Set Up", "Fire Control", "Reset Default"};
const int NUM_FIXED_ITEMS = 3;  // Number of fixed items
const char* fixed_items[] = {"Home", "Reset", "Help"}; // Fixed menu labels
int current_menu_item = 0;      // Index of the currently selected menu item
bool in_fixed_menu = false;     // Whether the fixed menu is active
bool button_pressed = false;    // Debounce flag for button presses
bool fire_unit_connected = false; // Radio connection status
//int num_fire_targets = 0;       // Number of fire targets
bool in_setup_menu = false;     // Whether setup menu is active
bool in_fire_control = false;   // Whether fire control menu is active
int16_t packetnum = 0;          // Packet number for radio communication
int16_t last_rssi = 0;          // RSSI (signal strength)
int ready_to_fire_button = -1;  // Button ready for firing
unsigned long last_blink_time = 0;  // Last blink timestamp for LEDs
bool blink_state = false;       // LED blink state

#define DEFAULT_FIRE_TARGETS 16
int num_fire_targets = DEFAULT_FIRE_TARGETS;
unsigned long last_button_press = 0;
const unsigned long DEBOUNCE_DELAY = 200;  // 200ms debounce delay

// Function Prototypes
void handleFireButtonPress(uint8_t i); // Handle button press for firing
void displayAdafruitLogo();            // Display the Adafruit logo on OLED
void animateNeoTrellis();              // Animation for NeoTrellis
void drawScreen();                     // Draw the current menu on the OLED
void menuUp();                         // Navigate menu up
void menuDown();                       // Navigate menu down
void menuLeft();                       // Navigate menu left
void menuRight();                      // Navigate menu right
void selectMenuItem();                 // Select the currently highlighted menu item
void handleConnectionMenu();           // Handle the "Connection" menu
void handleSetupMenu();                // Handle the "Set Up" menu
void drawSetupScreen();                // Draw the setup screen on the OLED
void updateNeoTrellisLEDs();           // Update the LEDs on the NeoTrellis
void initializeFireControl();          // Initialize fire control menu
void handleFireControl();              // Handle fire control logic
void exitFireControl();                // Exit fire control mode
void setButtonReady(uint8_t i);        // Set button ready to fire
void fireTarget(int target);           // Fire at the selected target
void drawFireReadyScreen(int button);  // Display ready-to-fire screen on OLED
void checkRadioConnection();           // Check the radio connection
void drawConnectionScreen();           // Draw the connection status on the OLED
void drawFireControlScreen();          // Draw the fire control screen
void drawFiringScreen(int target);     // Draw the screen when firing at a target
void drawErrorScreen(const char* message); // Display an error message
uint32_t Wheel(byte WheelPos);         // Generate rainbow colors for LEDs
void resetProgram();                   // Reset the entire program

// Trellis callback function: handles button presses on the NeoTrellis
TrellisCallback trellisCallback(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    handleFireButtonPress(evt.bit.NUM);  // Handle button press when rising
  }
  return 0;
}

// Another Trellis callback: handles LED blinking when buttons are pressed
TrellisCallback blink(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    trellis.pixels.setPixelColor(evt.bit.NUM, 0xFFFFFF); // Turn on LED
  } else if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_FALLING) {
    trellis.pixels.setPixelColor(evt.bit.NUM, 0); // Turn off LED
  }
  trellis.pixels.show(); // Show LED changes
  return 0;
}

// The setup function runs once when the system starts
void setup() {
  Serial.begin(115200);   // Start the serial monitor
  while (!Serial) delay(10);

  Wire.begin();           // Initialize I2C

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  
  displayAdafruitLogo(); // Show Adafruit logo

  // Initialize the NeoTrellis grid
  if (!trellis.begin()) {
    Serial.println("Could not start NeoTrellis");
    while (1);
  }

  // Enable NeoTrellis keys and register callbacks
  for (int i = 0; i < Y_DIM * X_DIM; i++) {
    trellis.activateKey(i, SEESAW_KEYPAD_EDGE_RISING); 
    trellis.registerCallback(i, trellisCallback);  
  }

  // Initialize the buttons on NeoTrellis to green
  for (int i = 0; i < num_fire_targets; i++) {
    trellis.pixels.setPixelColor(i, 0x00FF00);  // Green color
  }
  trellis.pixels.show(); // Show initial LED setup

  // Initialize Seesaw chip for rotary encoder and buttons
  if (!ss.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while (1);
  }

  ss.pinMode(SS_SWITCH_UP, INPUT_PULLUP);    // Configure buttons as inputs with pullups
  ss.pinMode(SS_SWITCH_DOWN, INPUT_PULLUP);
  ss.pinMode(SS_SWITCH_LEFT, INPUT_PULLUP);
  ss.pinMode(SS_SWITCH_RIGHT, INPUT_PULLUP);
  ss.pinMode(SS_SWITCH_SELECT, INPUT_PULLUP);

  ss.pinMode(24, INPUT_PULLUP);   // Pin 24 for additional input (encoder or button)
  encoder_position = ss.getEncoderPosition(); // Get the starting position of the encoder

  // Initialize the RFM69 radio
  pinMode(LED, OUTPUT);            // LED for visual feedback
  pinMode(RFM69_RST, OUTPUT);      // Set the reset pin for the radio
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");  // Test message for serial monitor
  Serial.println();

  // Manually reset the RFM69 module
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");  // Success message for radio initialization
  
  // Set radio frequency and transmission power
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true);  // Set transmission power (14-20), true for high-power module

  // Set encryption key for secure communication
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @"); Serial.print((int)RF69_FREQ); Serial.println(" MHz");

  // Animation and draw the initial screen
  animateNeoTrellis();
  drawScreen();
}

void loop() {
  trellis.read();  // Poll NeoTrellis for any button presses

  // Handle rotary encoder position changes
  int32_t new_position = ss.getEncoderPosition();
  if (encoder_position != new_position) {
    if (new_position > encoder_position) {
      menuDown();
    } else {
      menuUp();
    }
    encoder_position = new_position;
  }

  // Handle button presses using Seesaw
  if (!ss.digitalRead(SS_SWITCH_UP) && !button_pressed) {
    menuUp();          // Navigate up in the menu
    button_pressed = true;
  }
  else if (!ss.digitalRead(SS_SWITCH_DOWN) && !button_pressed) {
    menuDown();        // Navigate down in the menu
    button_pressed = true;
  }
  else if (!ss.digitalRead(SS_SWITCH_LEFT) && !button_pressed) {
    menuLeft();        // Navigate left
    button_pressed = true;
  }
  else if (!ss.digitalRead(SS_SWITCH_RIGHT) && !button_pressed) {
    menuRight();       // Navigate right
    button_pressed = true;
  }
  else if (!ss.digitalRead(SS_SWITCH_SELECT) && !button_pressed) {
    selectMenuItem();  // Select the current menu item
    button_pressed = true;
  }
  else if (ss.digitalRead(SS_SWITCH_UP) && ss.digitalRead(SS_SWITCH_DOWN) &&
           ss.digitalRead(SS_SWITCH_LEFT) && ss.digitalRead(SS_SWITCH_RIGHT) &&
           ss.digitalRead(SS_SWITCH_SELECT)) {
    button_pressed = false;  // Reset the button press state when all are released
  }

  // Check whether we are in the setup or fire control menus and handle accordingly
  if (in_setup_menu) {
    handleSetupMenu();
  }
  if (in_fire_control) {
    handleFireControl();
  }

  delay(10); // Small delay to prevent tight looping
}

// Displays the Adafruit logo on the OLED screen for 4 seconds
void displayAdafruitLogo() {
  display.clearDisplay();  // Clear the OLED screen
  display.drawBitmap(      // Draw the bitmap at the center of the screen
    (SCREEN_WIDTH  - LOGO_WIDTH ) / 2,
    (SCREEN_HEIGHT - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1); // logo_bmp is the predefined bitmap
  display.display();       // Display the drawn bitmap
  delay(4000);             // Wait for 4 seconds
}

// Handles the "up" menu navigation
void menuUp() {
  if (in_fixed_menu) {      // If currently in the fixed menu
    in_fixed_menu = false;  // Switch to the main menu
    current_menu_item = NUM_MENU_ITEMS - 1;  // Go to the last item in the main menu
  } else if (current_menu_item > 0) {
    current_menu_item--;    // Decrease the menu item index
  } else {
    in_fixed_menu = true;   // If at the top, go back to the fixed menu
    current_menu_item = NUM_FIXED_ITEMS - 1;
  }
  drawScreen();  // Redraw the screen to reflect changes
}

// Handles the "down" menu navigation
void menuDown() {
  if (in_fixed_menu) {  // If in the fixed menu
    if (current_menu_item < NUM_FIXED_ITEMS - 1) {
      current_menu_item++;  // Go to the next fixed item
    } else {
      in_fixed_menu = false;  // Switch to the main menu
      current_menu_item = 0;
    }
  } else {
    if (current_menu_item < NUM_MENU_ITEMS - 1) {
      current_menu_item++;  // Move to the next menu item
    } else {
      in_fixed_menu = true;  // If at the bottom, go to fixed menu
      current_menu_item = 0;
    }
  }
  drawScreen();  // Redraw the screen
}

// Handles left navigation in the fixed menu
void menuLeft() {
  if (in_fixed_menu) {
    if (current_menu_item > 0) {
      current_menu_item--;  // Go to the previous fixed menu item
    } else {
      current_menu_item = NUM_FIXED_ITEMS - 1;  // Wrap around to the last item
    }
  } else {
    in_fixed_menu = true;  // Switch to the fixed menu
    current_menu_item = 0;
  }
  drawScreen();  // Redraw the screen
}

// Handles right navigation in the fixed menu
void menuRight() {
  if (in_fixed_menu) {
    if (current_menu_item < NUM_FIXED_ITEMS - 1) {
      current_menu_item++;  // Go to the next fixed menu item
    } else {
      in_fixed_menu = false;  // Return to the main menu
      current_menu_item = 0;
    }
  } else {
    in_fixed_menu = true;  // Enter the fixed menu
    current_menu_item = 0;
  }
  drawScreen();  // Redraw the screen
}

// Handles selection of a menu item
// Modify the selectMenuItem() function to include the new reset option
void selectMenuItem() {
  if (in_fixed_menu) {
    // ... (existing code for fixed menu) ...
  } else {
    switch (current_menu_item) {
      case 0:  // Connection
        handleConnectionMenu();
        break;
      case 1:  // Set Up
        in_setup_menu = true;
        drawSetupScreen();
        break;
      case 2:  // Fire Control
        if (num_fire_targets > 0) {
          in_fire_control = true;
          initializeFireControl();
        } else {
          drawErrorScreen("Set targets first");
        }
        break;
      case 3:  // New option: Reset to Default
        resetToDefaultTargets();
        break;
    }
  }
  drawScreen();
}

// Handles the "Connection" menu
void handleConnectionMenu() {
  bool exit_menu = false;
  unsigned long last_check_time = 0;
  const unsigned long check_interval = 2000;  // Check connection every 2 seconds

  while (!exit_menu) {
    unsigned long current_time = millis();  // Get the current time

    // Periodically check the connection status
    if (current_time - last_check_time >= check_interval) {
      checkRadioConnection();  // Check the radio connection
      drawConnectionScreen();  // Display connection status on the OLED
      last_check_time = current_time;  // Update the last check time
    }

    // Check if the "Select" button is pressed to exit
    if (!ss.digitalRead(SS_SWITCH_SELECT)) {
      while (!ss.digitalRead(SS_SWITCH_SELECT)) {
        delay(10);  // Wait for the button to be released
      }
      exit_menu = true;  // Exit the menu
    }

    delay(10);  // Add a small delay to prevent tight looping
  }
}

// Handles the "Set Up" menu
void handleSetupMenu() {
  unsigned long current_time = millis();
  
  if (current_time - last_button_press > DEBOUNCE_DELAY) {
    if (!ss.digitalRead(SS_SWITCH_UP)) {
      if (num_fire_targets < 16) num_fire_targets++;
      last_button_press = current_time;
      drawSetupScreen();
      updateNeoTrellisLEDs();
    }
    else if (!ss.digitalRead(SS_SWITCH_DOWN)) {
      if (num_fire_targets > 1) num_fire_targets--;
      last_button_press = current_time;
      drawSetupScreen();
      updateNeoTrellisLEDs();
    }
    else if (!ss.digitalRead(SS_SWITCH_SELECT)) {
      in_setup_menu = false;
      drawScreen();
      return;
    }
  }
  
  // Check if encoder has been rotated
  int32_t new_position = ss.getEncoderPosition();
  if (encoder_position != new_position) {
    if (new_position > encoder_position) {
      if (num_fire_targets < 16) num_fire_targets++;
    } else {
      if (num_fire_targets > 1) num_fire_targets--;
    }
    encoder_position = new_position;
    drawSetupScreen();
    updateNeoTrellisLEDs();
  }
}

// Add a new function to reset to default targets
void resetToDefaultTargets() {
  num_fire_targets = DEFAULT_FIRE_TARGETS;
  updateNeoTrellisLEDs();
  drawSetupScreen();
}

// Draws the "Set Up" screen on the OLED display
void drawSetupScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Set Fire Targets");
  display.println();
  display.setTextSize(2);
  display.println(num_fire_targets);  // Display the current number of fire targets
  display.setTextSize(1);
  display.println();
  display.println("Up/Down to change");
  display.println("Select to confirm");
  display.display();  // Show the updated screen
}

// Updates the LEDs on the NeoTrellis grid
void updateNeoTrellisLEDs() {
  for (int i = 0; i < 16; i++) {
    if (i < num_fire_targets) {
      trellis.pixels.setPixelColor(i, 0x00FF00); // Green for active targets
    } else {
      trellis.pixels.setPixelColor(i, 0x000000); // Off for inactive targets
    }
  }
  trellis.pixels.show();  // Display the updated LED states
}

// Initializes fire control mode
void initializeFireControl() {
  for (int i = 0; i < 16; i++) {
    if (i < num_fire_targets) {
      trellis.pixels.setPixelColor(i, 0x00FF00); // Green for active targets
    } else {
      trellis.pixels.setPixelColor(i, 0x000000); // Off for inactive targets
    }
  }
  trellis.pixels.show();
  ready_to_fire_button = -1;  // Reset ready-to-fire button
  drawFireControlScreen();    // Draw the fire control screen
}

// Handles the fire control logic and state
void handleFireControl() {
  unsigned long current_time = millis();  // Get the current time

  // Handle LED blinking for the "ready to fire" button
  if (ready_to_fire_button != -1) {
    if (current_time - last_blink_time > 250) {  // Blink every 250ms
      last_blink_time = current_time;
      blink_state = !blink_state;
      if (blink_state) {
        trellis.pixels.setPixelColor(ready_to_fire_button, 0x00FF00);  // Green
      } else {
        trellis.pixels.setPixelColor(ready_to_fire_button, 0x000000);  // Off
      }
      trellis.pixels.show();
    }
  }

  // Check for SELECT button press to exit fire control
  if (!ss.digitalRead(SS_SWITCH_SELECT) && !button_pressed) {
    exitFireControl();  // Exit fire control mode
    button_pressed = true;
  } else if (ss.digitalRead(SS_SWITCH_SELECT)) {
    button_pressed = false;
  }
}

// Handles the firing button press event on the NeoTrellis
void handleFireButtonPress(uint8_t i) {
  if (i >= num_fire_targets) {
    return;  // Ignore presses on buttons that aren't fire targets
  }

  uint32_t buttonColor = trellis.pixels.getPixelColor(i);

  if (buttonColor == 0xFF0000) {  // Red - already fired
    return;  // Do nothing for already fired buttons
  } else if (i == ready_to_fire_button) {  // Blinking - ready to fire
    fireTarget(i);  // Fire the target
  } else {  // Green or off - set button to ready
    // Cancel the previous ready button if it exists
    if (ready_to_fire_button != -1) {
      trellis.pixels.setPixelColor(ready_to_fire_button, 0x00FF00);  // Green
      trellis.pixels.show();
    }
    
    ready_to_fire_button = i;  // Set this button as ready to fire
    blink_state = true;        // Start blinking in ON state
    last_blink_time = millis();  // Reset the blink timer
    
    // Immediately start blinking by setting to ON
    trellis.pixels.setPixelColor(i, 0x00FF00);  // Green
    trellis.pixels.show();
    
    drawFireReadyScreen(i);  // Display the ready-to-fire screen
  }
}

// Exits the fire control mode
void exitFireControl() {
  in_fire_control = false;  // Exit the fire control mode
  
  // Turn off all LEDs
  for (int i = 0; i < 16; i++) {
    trellis.pixels.setPixelColor(i, 0x000000);  // Turn off LEDs
  }
  trellis.pixels.show();
  ready_to_fire_button = -1;  // Reset the ready button
  drawScreen();  // Redraw the main screen
}

// Sets a button as ready to fire and manages the blinking effect
void setButtonReady(uint8_t i) {
  // Cancel previous ready button if it exists
  if (ready_to_fire_button != -1) {
    trellis.pixels.setPixelColor(ready_to_fire_button, 0x00FF00);  // Green
    trellis.pixels.show();
  }
  
  ready_to_fire_button = i;  // Set this button as the ready button
  blink_state = true;        // Set the blink state to ON
  last_blink_time = millis();  // Reset blink timer
  
  // Start blinking immediately
  trellis.pixels.setPixelColor(i, 0x00FF00);  // Set LED to green
  trellis.pixels.show();
  
  drawFireReadyScreen(i);  // Display the ready-to-fire screen
}

// Fires the selected target
void fireTarget(int target) {
  // Stop the blinking effect
  ready_to_fire_button = -1;
  
  // Set button to red to indicate it has been fired
  trellis.pixels.setPixelColor(target, 0xFF0000);  // Red for fired target
  trellis.pixels.show();
  
  // Display a firing message on the OLED
  drawFiringScreen(target);
  
  // Send the fire command via radio
  char radiopacket[20];
  sprintf(radiopacket, "FIRE %d", target);  // Create a message with the target number
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));  // Send the radio packet
  rf69.waitPacketSent();  // Wait until the packet is sent
  
  delay(1000);  // Display the firing message for 1 second
  
  drawFireControlScreen();  // Return to the fire control screen
}

// Draws the ready-to-fire screen on the OLED
void drawFireReadyScreen(int button) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Fire Control");
  display.println();
  display.setTextSize(1);
  display.print("Fire Button #");
  display.println(button + 1);  // Display the button number
  display.setTextSize(1);
  display.println("READY");
  display.println();
  display.println("Press again to fire");
  display.println("Or press another");
  display.println("button to cancel");
  display.display();  // Show the ready-to-fire message
}

// Checks the radio connection to the fire unit
void checkRadioConnection() {
  char radiopacket[20] = "Hello World #";  // Prepare a test message
  itoa(packetnum++, radiopacket + 13, 10);  // Add the packet number
  Serial.print("Sending "); 
  Serial.println(radiopacket);  // Log the message being sent
  
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));  // Send the message
  rf69.waitPacketSent();  // Wait until the message is sent

  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];  // Buffer for incoming messages
  uint8_t len = sizeof(buf);  // Length of the buffer
  
  if (rf69.waitAvailableTimeout(1000)) {  // Wait for a reply (1 second timeout)
    if (rf69.recv(buf, &len)) {  // If a reply is received
      Serial.print("Got a reply: ");
      Serial.println((char *)buf);  // Log the received message
      last_rssi = rf69.lastRssi();  // Record the signal strength
      fire_unit_connected = true;   // Mark the fire unit as connected
    } else {
      Serial.println("Receive failed");  // Log a receive failure
      fire_unit_connected = false;
    }
  } else {
    Serial.println("No reply, is fire unit connected?");  // No reply received
    fire_unit_connected = false;
  }
}

// Draws the connection status screen on the OLED
void drawConnectionScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connection Status");
  display.println();
  
  // Display the connection status
  if (fire_unit_connected) {
    display.println("Fire Unit: ONLINE");
    display.print("RSSI: ");
    display.print(last_rssi);  // Display the signal strength (RSSI)
    display.println(" dBm");
  } else {
    display.println("Fire Unit: OFFLINE");
    display.println("Check connection");
  }
  
  display.println();
  display.println("Press SELECT to exit");
  display.display();  // Show the connection status
}

// Draws the fire control screen on the OLED
void drawFireControlScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Fire Control");
  display.println();
  display.println("Press button to fire");
  display.println("Select to exit");
  display.display();  // Show the fire control screen
}

// Draws the firing screen on the OLED when a target is fired
void drawFiringScreen(int target) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Fire Control");
  display.println();
  display.setTextSize(2);
  display.print("Fired #");
  display.println(target + 1);  // Display the target number
  display.display();  // Show the firing screen
}

// Draws an error message on the OLED
void drawErrorScreen(const char* message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Error");
  display.println();
  display.println(message);  // Display the error message
  display.display();
  delay(2000);  // Show the error message for 2 seconds
  drawScreen();  // Return to the main screen
}

// Animates the NeoTrellis grid with a rainbow pattern
void animateNeoTrellis() {
  for (int i = 0; i < Y_DIM * X_DIM; i++) {
    trellis.pixels.setPixelColor(i, Wheel(map(i, 0, Y_DIM * X_DIM - 1, 0, 255)));  // Map the color
    trellis.pixels.show();  // Show the colors
    delay(50);  // Delay between LED changes
  }
  
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < Y_DIM * X_DIM; j++) {
      trellis.pixels.setPixelColor(j, Wheel(((j * 256 / (Y_DIM * X_DIM)) + i) & 255));  // Create a rainbow effect
    }
    trellis.pixels.show();  // Show the colors
    delay(10);  // Delay for smooth transition
  }

  for (int i = 0; i < Y_DIM * X_DIM; i++) {
    trellis.pixels.setPixelColor(i, 0x000000);  // Turn off all LEDs
    trellis.pixels.show();
    delay(50);  // Delay between LED changes
  }
}

// Draws the main screen and updates the menu
void drawScreen() {
  display.clearDisplay();

  // Draw the top status bar with connection status
  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor((SCREEN_WIDTH - (fire_unit_connected ? 19 : 22) * 6) / 2, 1);
  display.print(fire_unit_connected ? "Fire Unit Online" : "Fire Unit OFFLINE");

  // Draw the main menu
  display.setTextColor(SSD1306_WHITE);
  for (int i = 0; i < NUM_MENU_ITEMS; i++) {
    if (!in_fixed_menu && i == current_menu_item) {
      display.fillRect(0, 12 + i * 10, SCREEN_WIDTH, 10, SSD1306_WHITE);  // Highlight selected item
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(2, 13 + i * 10);
    display.print(menu_items[i]);  // Display menu items
  }

  // Draw the bottom menu
  display.drawFastHLine(0, 54, SCREEN_WIDTH, SSD1306_WHITE);
  display.setTextColor(SSD1306_WHITE);
  for (int i = 0; i < NUM_FIXED_ITEMS; i++) {
    if (in_fixed_menu && i == current_menu_item) {
      display.fillRect(i * 42, 55, 42, 9, SSD1306_WHITE);  // Highlight selected fixed item
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(i * 42 + 2, 56);
    display.print(fixed_items[i]);  // Display fixed menu items
  }

  display.display();  // Show the updated screen
}

// Generates rainbow colors based on the position on the wheel
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return trellis.pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);  // Red to Blue
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return trellis.pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);  // Blue to Green
  }
  WheelPos -= 170;
  return trellis.pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);  // Green to Red
}

// Resets the entire program, including variables and menu states
void resetProgram() {
  num_fire_targets = 0;        // Reset fire targets
  in_setup_menu = false;       // Exit setup menu
  in_fire_control = false;     // Exit fire control mode
  current_menu_item = 0;       // Reset menu selection
  in_fixed_menu = false;       // Exit fixed menu mode
  for (int i = 0; i < 16; i++) {
    trellis.pixels.setPixelColor(i, 0x000000);  // Turn off all LEDs
  }
  trellis.pixels.show();       // Show the updated LED state
  drawScreen();                // Redraw the main screen
}
