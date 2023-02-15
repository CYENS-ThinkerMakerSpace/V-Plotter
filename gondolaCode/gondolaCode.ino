#include <BluetoothSerial.h> 
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) 
#error Bluetooth is not enabled!
#endif 

BluetoothSerial SerialBT; 


const int servo_pin = 27; // GPIO 27 
String bluetoothData = ""; 
char prev = '0'; 
int msServoActive = 70; 

// Setting PWM properties 
const int freq = 50; 
const int servoChannel = 0; 
const int resolution = 8; 
  
void setup(){ 
  // Configure LED PWM functionalitites 
  Serial.begin(115200); 
  ledcSetup(servoChannel, freq, resolution); 
  
  // Attach the channel to the GPIO to be controlled 
  ledcAttachPin(servo_pin, servoChannel); 
  
  SerialBT.begin("ESP32_Gondola"); 
  Serial.println("Bluetooth Started! Ready to pair..."); 
  startWithPenUp(); 
} 


void loop() { 
  if (SerialBT.available()) 
  { 
    String bluetoothData = SerialBT.readString(); // Each time this is read, it empties the buffer 

    if ((bluetoothData.charAt(0) != '0' && bluetoothData.charAt(0) != '1') || bluetoothData.charAt(0) == prev) 
    { 
      //Serial.write("Same as the last character, or non supported character: "); 
      //Serial.print(bluetoothData.charAt(0)); 
      //Serial.write(".\n"); 
      // skip loop 
      return; 
    } 

  if (bluetoothData.charAt(0) == '1') 
  { 
    Serial.print("\nPEN DOWN.\n"); 
    ledcWrite(servoChannel, 35); // Start the servo 
    delay(msServoActive); // Wait for the pen to touch the "canvas" 
    ledcWrite(servoChannel, 0); // Stop the servo 
    prev = '1'; 
  } 
  if (bluetoothData.charAt(0) == '0') 
  { 
    Serial.print("\nPEN UP.\n"); 
    ledcWrite(servoChannel, 1); // Start the servo 
    delay(msServoActive); // Wait for the pen to not touch the "canvas" 
    ledcWrite(servoChannel, 0); // Stop the servo  
    prev = '0';  
  } 
    
  } 
} 

void startWithPenUp() 
{ 
    Serial.print("\nStarting with Pen Up.\n"); 
    ledcWrite(servoChannel, 1); // Start the servo 
    delay(msServoActive); // Wait for the pen to not touch the "canvas" 
    ledcWrite(servoChannel, 0); // Stop the servo 
    prev = '0'; 
}
