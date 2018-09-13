
#include "dGPS.h"
#include <SD.h>

File myFile;
// Software serial TX & RX Pins for the GPS module
// Initiate the software serial connection

int ledPin = 13;                  // LED test pin
float London[]={51.5171,-0.1062};

dGPS dgps = dGPS();               // Construct dGPS class
long buf_time[100];
float buf_speed[100];
float buf_cap_reel[100];
int buf_regu[100];
float buf_hdop[100];
int buf_nb_sat[100];
float buf_lat[100];
float buf_lon[100];
int i = 0;

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
  buf_time[i] = dgps.Time();
  buf_speed[i] = dgps.Vel();
  buf_cap_reel[i] = dgps.Head();
  buf_regu[i] = 5;
  buf_hdop[i] = dgps.Hdop();
  buf_nb_sat[i] = dgps.SatView();
  buf_lat[i] = dgps.Lat();
  buf_lon[i] = dgps.Lon();
  i++;
  if (i >= 10){
    myFile = SD.open("log.txt", FILE_WRITE);
    String line; 
    for (int a = 0; a < i; a++){
      line = "$";
      line += "t:";
      line += buf_time[a];
      line += " nb_sat:";
      line += buf_nb_sat[a];
      line += " hdop:";
      line += buf_hdop[a];
      line += " v:";
      line += buf_speed[a];
      line += " lat:";
      line += buf_lat[a];
      line += " lon:";
      line += buf_lon[a];
      line += " cap:";
      line += buf_cap_reel[a];
      myFile.println(line);
    }  
    myFile.close();
    i = 0;
  }
  
   
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
  Serial.println(dgps.Date());       
}
