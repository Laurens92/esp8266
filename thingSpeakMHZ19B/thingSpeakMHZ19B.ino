#include <ESP8266WiFi.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>		// Remove if using HardwareSerial or Arduino package without SoftwareSerial support
#include "ThingSpeak.h"			// Thingspeak is used to write the data to a cloud environment
#include "secrets.h"

// Pin lay-out for the ESP8266 NodeMCU V2
#define LED_YELLOW 	D1   // Connect yellow LED to D1
#define LED_GREEN 	D2   // Connect green LED to D2
#define LED_RED 	D5   // Connect red LED to D5
#define MH_Z19_RX 	D7   // Set pin D7 to UART RX
#define MH_Z19_TX 	D6   // Set pin D6 to UART TX

#define BAUDRATE 9600    // Device to MH-Z19 Serial baudrate (do not change!)

/* 
This program controls the MHZ19B from a ESP8266 NodeMCU V2. The MHZ19B is a CO2 sensor using non-dispersive infrared principle (NDIR).
The ESP8266 NodeMCU V2 is a microcontroller with an embedded WiFi module.

The program reads the CO2 ppm level and controls three LEDS to show the air quality, the following rules are used:
<800  			green
>800 && <1200 	orange
>1200 			red

When an internet connection is available the CO2 level can be written towards the cloud using thingspeak
*/

//************ BEGIN PROGRAM ************//
unsigned long ts_channel_number = SECRET_CH_ID;			// Thingspeak channel number
const char * ts_write_API_key = SECRET_WRITE_APIKEY; 	// Thingspeak API key

char ssid[] = SECRET_SSID;   							// your network SSID (name)
char pass[] = SECRET_PASS;   							// your network password

WiFiClient client;										// Setup WiFi object
MHZ19 myMHZ19;                                          // Create object to control MHZ19B
SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX);         // Setup a software serial object to communicate with MH-Z19B

unsigned long TimerLedBlink_ms = 0; 			// Timer to blink the buildin LED to notify user the microcontroller is not hanging
unsigned long periodLED_On_ms = 5000;			// 5 seconds blinking interval LED

bool warmingup_mhz19 = true;
unsigned long periodWarmup_mhz19_ms = 180000; 	// Start-up time of the MH-Z19B is 3 min. Otherwise wrong results are read
unsigned long TimerWarmup_mhz19 = 0;			// Timer for the MH-Z19B to warm-up

unsigned long TimerRetrieveData_ms = 0;			// Timer to retrieve data and send it to thingspeak
unsigned long periodRetrieveData_ms = 60000; 	// How often a new CO2 sample is read

// Define the CO2 levels for green and red. Yellow is inbetween
int level_green_ppm = 800;
int level_red_ppm = 1200;

void setup()
{
  Serial.begin(9600);              	// Start a serial connection for debugging purposes
  delay(100);						//
  
  co2Serial.begin(BAUDRATE);        // Start the software serial to communicate with the MH-Z19B   
  myMHZ19.begin(co2Serial);         // Pass communication stream 
  myMHZ19.autoCalibration(false);   // Turn auto calibration OFF
  delay(100);
  
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(LED_YELLOW, OUTPUT);     	// Initialize the LED_BUILTIN pin as an output
  pinMode(LED_GREEN, OUTPUT);     	// Initialize the LED_BUILTIN pin as an output
  pinMode(LED_RED, OUTPUT);     	// Initialize the LED_BUILTIN pin as an output
  
  WiFi.mode(WIFI_STA);				// Station Mode: the ESP8266 connects to an access point
  ThingSpeak.begin(client);			// Initialize the ThingSpeak library and network settings using the ThingSpeak.com service
  
  // Turn all LEDS on to notify user the program is booting
  digitalWrite(LED_BUILTIN, HIGH);  // LED off
  digitalWrite(LED_YELLOW, HIGH);  	// LED on
  digitalWrite(LED_GREEN, HIGH);  	// LED on
  digitalWrite(LED_RED, HIGH);  	// LED on
  delay(100);
}

void loop()
{
	// Connect or reconnect to WiFi
	if(WiFi.status() != WL_CONNECTED)
  {
		Serial.print("Attempting to connect to SSID: ");
		Serial.println(SECRET_SSID);
		while (WiFi.status() != WL_CONNECTED)
    {
			WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network. Change this line if using open or WEP network
			Serial.print(".");
			delay(5000);
		}
    Serial.println("\nConnected.");
	}
 
	// Blink the built in LED to notify user that the microcontroller is not hanging during the warm-up stage
	if(millis() - TimerLedBlink_ms > periodLED_On_ms && warmingup_mhz19)
  {
		TimerLedBlink_ms = millis();
		digitalWrite(LED_BUILTIN, LOW); 	// LED on
		delay(500);
		digitalWrite(LED_BUILTIN, HIGH);  	// LED off
	}
	else
  {
    digitalWrite(LED_BUILTIN, HIGH);  	// LED off after warming up
  }
	
	// Check if MH-Z19B is still warming-up
	if(warmingup_mhz19)
  {
		TimerWarmup_mhz19 = millis();
		if(TimerWarmup_mhz19 > periodWarmup_mhz19_ms)
    {
			warmingup_mhz19 = false;
		}
	}
  else // This code is only reached after warm-up!
  { 
    if (millis() - TimerRetrieveData_ms >= periodRetrieveData_ms) // Execute these commands every 60 seconds
      {
        TimerRetrieveData_ms = millis();
        
        // Retrieve CO2_level_ppm level
        int CO2_level_ppm = myMHZ19.getCO2();          // Request CO2_level_ppm (as ppm)
        int httpCode = ThingSpeak.writeField(ts_channel_number, 1, CO2_level_ppm, ts_write_API_key);
    
        Serial.print("CO2 (ppm): ");                      
        Serial.println(CO2_level_ppm); 

      // Change led color depending on PPM level
      if(CO2_level_ppm < level_green_ppm) // Green
      {
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_GREEN, HIGH); 
        digitalWrite(LED_RED, LOW); 
      }
      else if(CO2_level_ppm >= level_green_ppm && CO2_level_ppm < level_red_ppm) // Yellow
      {
        digitalWrite(LED_YELLOW, HIGH);
        digitalWrite(LED_GREEN, LOW); 
        digitalWrite(LED_RED, LOW);    
      }
      else // Red
      {
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, HIGH); 
      }
    } 
  }
	
  // Allow ESP8266 to do WIFI stuff
  delay(100); 
}