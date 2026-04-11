#ifdef ESP32
  #include <WiFi.h>
#elif defined(ARDUINO_UNOWIFIR4)
  #include <WiFiS3.h>
#else
  #error "please have an esp32 or arduino r4 wifi board"
#endif


#include <string.h>

/*This file contains the code for the arduino wifi. This will eventually be put in the main file for the arduino code*/
#define MAX_CONNECT_ATTEMPTS 20

// not using c strings because c++ strings are easier to deal with
String ROUTER_NAME = "HerbyIOT";
String ROUTER_PASSWORD = "herbinator";

String getString(String);
void connect(String, String);



void setup(){ // in the futire, this code may be moved to its own function so that we can do more with it on startup
  Serial.begin(9600); // so that terminal can be read
  
  /*
  while (!Serial){
    ; // waits for arduino to connect
  }

  ROUTER_NAME = getString("Enter router name.");
  ROUTER_NAME.trim();
  
  ROUTER_PASSWORD = getString("Enter router password.");\
  ROUTER_PASSWORD.trim();
  */
  connect(ROUTER_NAME, ROUTER_PASSWORD);


}

void loop(){
  // will add connection code and probably put it in a separate file.
}


// this function enables the boards to connect to a wifi router
void connect(String name, String password){

  WiFi.begin(ROUTER_NAME.c_str(), ROUTER_PASSWORD.c_str());
 
  for (int attempts = 0; attempts < MAX_CONNECT_ATTEMPTS && WiFi.status() != WL_CONNECTED; attempts++){   // delay for connetion to the 
    delay(500); // pauses
  }

  if (WiFi.status() == WL_CONNECTED){
    Serial.println("Connected");
  } else Serial.println("Failed to connect. Please try again.");
}

String getString(String prompt){
  Serial.readString(); // flushes whatever is in the input
  Serial.println(prompt);

  while(Serial.available() <= 0); // BLANK--buffer for user to enter input. Will be exited once input provided

  return Serial.readStringUntil('\n');

}

