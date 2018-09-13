
#include "dGPS.h"
#include <SD.h>
#include <Servo.h>

File myFile;
Servo barre;
Servo aile;

int ledPin = 13;                  // LED test pin
float cible[]={48.3611,-4.5672};

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
int ch3;
int ch6;
int ch5;
int pos;


void setup() {
  pinMode(12, INPUT);   // input for ch3
  pinMode(7, INPUT);    // input for rc switch
  pinMode(5, INPUT);    // input for rc switch
  pinMode(13, OUTPUT);       // Initialize LED pin
  barre.attach(8);
  aile.attach(6);
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
  Serial.println("Initialisation GPS.");
  dgps.init();                   // Run initialization routine for dGPS.
  digitalWrite(13, HIGH);
  delay(1000); 
  digitalWrite(13, LOW); 
}

void loop() {
  ch3 = pulseIn(12, HIGH, 24000); // Read the pulse width of radio_commande
  pos = map(ch3, 990,1990,0,180);   // 90 = face au vent
  if (pos >= 0 && pos <= 180){
     barre.write(pos);
  }

  ch6 = pulseIn(7, HIGH, 24000);
  ch5 = pulseIn(5, HIGH, 24000);
  
  if (ch5 < 1500){   // switch -> 1) radio commandé / 2) semi autonome     
    // ici le switch est sur "radio commandé"
    /*
    ////////////////////////////////////
    //           Voiles               //
    ////////////////////////////////////
    int reglage = 2*pos;    // On calcule la position de la voile à partir de l'angle 
     if (reglage > 180){    // du régulateur d'allure
       reglage = 360 - reglage; 
     }
     if (reglage >= 0 && reglage <= 180){
        aile.write(reglage);
     }
    */
    ////////////////////////////////////
    //            Aile                //
    ////////////////////////////////////
    
    if (ch6 >= 900 && ch6 <= 1100){   // les trois positions du switch 'C' donnent la position de l'aile
      aile.write(50);  // pos 1 -> aileron à droite
    }
    else if (ch6 >= 1900 && ch6 <= 2100){
      aile.write(120);  // pos 3 -> aileron à gauche
    }
    else {
      aile.write(90); // pos 2 -> aileron au centre
    }
  }
  else {
    // ici le switch est sur "semi autonome"
    if (pos >= 90){   // La position de l'aile dépend de l'angle du régulateur
      aile.write(50);
    }
    else {
      aile.write(120);
    }
  }

  dgps.update(cible[0],cible[1]);  
  dgps.updategga();
  Serial.print("satellites trouvés : ");
  Serial.println(dgps.SatView());
  Serial.print("cap vers cible : ");
  Serial.print(dgps.Azim());
  Serial.print("   cap : ");
  Serial.print(dgps.Head());
  Serial.print("  dst vers cible : ");
  Serial.println(dgps.Dist());
  
  if ( dgps.SatView() >= 3){
    digitalWrite(13, HIGH);
  }
  else {
      digitalWrite(13, LOW);
  }
  
  buf_time[i] = dgps.Time();
  buf_speed[i] = dgps.Vel();
  buf_cap_reel[i] = dgps.Head();
  buf_regu[i] = 5;
  buf_hdop[i] = dgps.Hdop();
  buf_nb_sat[i] = dgps.SatView();
  buf_lat[i] = dgps.Lat();
  buf_lon[i] = dgps.Lon();
  i++;
  if (i >= 90){
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
      buf_regu[i] = 5;
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
}
