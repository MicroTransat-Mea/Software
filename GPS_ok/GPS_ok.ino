
#include "dGPS.h"
#include <SD.h>

File myFile;
// Software serial TX & RX Pins for the GPS module
// Initiate the software serial connection

int ledPin = 13;                  // LED test pin
float London[]={51.5171,-0.1062};

dGPS dgps = dGPS();               // Construct dGPS class

void setup() {
  pinMode(13, OUTPUT);       // Initialize LED pin
  pinMode(10, INPUT);
  pinMode(4, OUTPUT);
  Serial.begin(115200);
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization carte SD : failed");
  }
  else {
    Serial.println("initialization carte SD : OK"); 
  }

  // préparation du fichier txt
  if (SD.exists("log.txt")) {    // fichier existant, rien à faire
  } else {
      myFile = SD.open("log.txt", FILE_WRITE); // creation du fichier
      myFile.close();
  }
  
  Serial.end();                  // Close any previously established connections
  Serial.begin(9600);            // Serial output back to computer.  On.
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.println("Initialisation GPS.");
  dgps.init();                   // Run initialization routine for dGPS.
  digitalWrite(13, HIGH);
  delay(1000); 
  digitalWrite(13, LOW); 
}

void loop() { 
  dgps.update(London[0],London[1]);  
  dgps.updategga();
  Serial.print("satellites trouvés : ");
  Serial.println(dgps.SatView());
  if ( dgps.SatView() >= 3){
    digitalWrite(13, HIGH);
  }
  else {
      digitalWrite(13, LOW);
  }
  
  Serial.print("Hdop : ");
  Serial.println(dgps.Hdop());
  Serial.print("Vitesse: ");
  Serial.println(dgps.Vel());
  Serial.print("Cap: ");
  Serial.println(dgps.Head());  

}
