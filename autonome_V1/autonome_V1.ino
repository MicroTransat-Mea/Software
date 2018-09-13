#include "dGPS.h"
#include <SD.h>
#include <Servo.h>

File myFile;
Servo barre;
Servo aile;
int a = 16;  //map(analogRead(A14),0,1024,0,90);
int pos1 = 90 - a;
int pos2 = 90;
int pos3 = 90 + a;


int ledPin = 13;                  // LED test pin

dGPS dgps = dGPS();               // Construct dGPS class
long buf_time[100];
float buf_speed[100];
float buf_cap_reel[100];
int buf_regu[100];
float vent[100];
int buf_nb_sat[100];
float buf_lat[100];
float buf_lon[100];
float buf_dir_vent[100];
int i = 0;
int ch3;
int ch6;
int ch5;
int pos;

// liste points GPS
float wp_lat[] = {48.428198, 48.428839};  // 48.428198  -4.608139
float wp_lon[] = {-4.608139, -4.607959};

unsigned long last_change = 0;  // date du dernier changement d'allure
int estim_vent = -1;            // angle estimé du vent. Si -1 incertitude trop importante
int tab_vent[10];
int index_tab_vent = 0;
int index_wpt = 0;              // index dans la liste des wpt
int route_vent;                 // Consigne du régulateur pour atteindre le wpt
bool mode_autonome = false;     // mode
unsigned long timer1;           // timer
unsigned long timer2;           // interval de calcul cible
unsigned long interval_calcul = 10000; 
int vent_moyen;

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
  
  Serial.println("Initialisation GPS.");
  dgps.init();                   // Run initialization routine for dGPS.
  digitalWrite(13, HIGH);
  delay(1000); 
  digitalWrite(13, LOW);
}


int analyse_vent(int regulateur, int cap){
  int estim = cap - regulateur;
  if (estim < 0){
    estim += 360;
  }
  return estim;
}

void next_point(float dist){  // vérifier échelle : mètre, km ?
  if (dist < 0.02){
    if (index_wpt < sizeof(wp_lat)){
      index_wpt ++;
    }
    else {
      index_wpt = 0;
    }
    // on charge les nouvelles coordonnées   
    dgps.update(wp_lat[index_wpt],wp_lon[index_wpt]);
  }
}

void commande_barre( int angle){
  if (angle <= 180){
    angle += 180;
  }
  else {
    angle -= 180;
  }
  angle = angle/2;
  barre.write(angle);

  // commande aile

  if (angle <= 180){   // La position de l'aile dépend de l'angle du régulateur
    aile.write(pos1);
  }
  else {
    aile.write(pos3);
  }
  
}

void loop() {
  
  dgps.update(wp_lat[0],wp_lon[0]);  
  dgps.updategga();
  ch3 = pulseIn(12, HIGH, 24000); // Read the pulse width of radio_commande
  pos = map(ch3, 990,1990,0,180);   // 90 = face au vent
  if (pos >= 0 && pos <= 180 && mode_autonome == false){
     barre.write(pos);
  }

  ch6 = pulseIn(7, HIGH, 24000);
  ch5 = pulseIn(5, HIGH, 24000);
  
  if (ch5 < 1500){   // switch -> 1) radio commandé / 2) semi autonome     
    // ici le switch est sur "radio commandé"
    mode_autonome = false;
    estim_vent = -1;
    ////////////////////////////////////
    //            Aile                //
    ////////////////////////////////////
    
    if (ch6 >= 900 && ch6 <= 1100){   // les trois positions du switch 'C' donnent la position de l'aile
      aile.write(pos1);  // pos 1 -> aileron à droite
    }
    else if (ch6 >= 1900 && ch6 <= 2100){
      aile.write(pos3);  // pos 3 -> aileron à gauche
    }
    else {
      aile.write(pos2); // pos 2 -> aileron au centre
    }
  }
  else {
    // ici le switch 'D' est sur "semi autonome"
    if (ch6 >= 1100){   // le switch 'C' est en pos1 -> mode semi auto
      /////////////////////////////////////
      //        Mode full auto           //
      /////////////////////////////////////
      mode_autonome = true;
 
      if (estim_vent == -1){  // le vent estimé n'est pas suffisement précis ou inexistant
        commande_barre(90);
        timer1 = millis();
        estim_vent = -2;
        Serial.println(" début du travers ");
      }
      else if (estim_vent == -2) { // phase d'attente de stabilisation vent de travers
        if (millis() - timer1 > 10000){   // attente de 10 secondes
          estim_vent = 0;
          timer2 = millis();
          Serial.println(" Fin du travers ");
        }
      }
      else if (dgps.SatView() >= 3) {
        
        estim_vent = analyse_vent(2*pos ,dgps.Head());
        tab_vent[index_tab_vent] = estim_vent;
        index_tab_vent ++;
        vent_moyen = (tab_vent[0]+tab_vent[1]+tab_vent[2]+tab_vent[3]+tab_vent[4]+tab_vent[5]+tab_vent[6]+tab_vent[7]+tab_vent[8]+tab_vent[9])/10; // Oui oui je sais c'est sale ... 

        if (millis() - timer2 > interval_calcul){
          Serial.println("calcul route");
          timer2 = millis();
          next_point(dgps.Dist());
          route_vent = analyse_vent(dgps.Azim(), vent_moyen);
          if (route_vent < 45 || route_vent > 315){   // !! vérifier sens de rotation !!
            commande_barre(45);    // on louvoie en privilégiant babord amure (arbitraire)
          }
          else {  // On est en route direct
            commande_barre(route_vent);
          } 
        }
      }
    }
    else {
      /////////////////////////////////////
      //        Mode semi auto           //
      /////////////////////////////////////
      mode_autonome = false;
      estim_vent = -1;
    
      if (pos >= 90){   // La position de l'aile dépend de l'angle du régulateur
        aile.write(pos1);
      }
      else {
        aile.write(pos3);
      }
    }
  }
  
  if ( dgps.SatView() >= 3){
    digitalWrite(13, HIGH);
  }
  else {
      digitalWrite(13, LOW);
  }
  
  buf_time[i] = dgps.Time();
  buf_speed[i] = dgps.Vel();
  buf_cap_reel[i] = dgps.Head();
  buf_regu[i] = pos;
  vent[i] = vent_moyen;
  buf_nb_sat[i] = dgps.SatView();
  buf_lat[i] = dgps.Lat();
  buf_lon[i] = dgps.Lon();
  buf_dir_vent[i] = vent_moyen;
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
      line += vent[a];
      line += " v:";
      line += buf_speed[a];
      line += " lat:";
      line += String(buf_lat[a],8);
      line += " lon:";
      line += String(buf_lon[a],8);
      line += " cap:";
      line += buf_cap_reel[a];
      line += " regu:";
      line += buf_regu[a];
      line += " vent:";
      line += buf_dir_vent[a];
      Serial.print("ecriture carte sd : ");
      Serial.println(line);
      myFile.println(line);
    }  
    myFile.close();
    i = 0;
  }       
}
