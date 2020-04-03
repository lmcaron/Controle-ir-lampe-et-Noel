/****Controle de lumieres de Noel et d'une lampe
 création: 2019-12-11
 Dynamique d'un ressort oscillant avec condition ininitale fixes

 Trouver solution pour executer 2 interrupts en même temps. > Fonction ISR appelle
 une autre fonction dans code. Tous les signaux sont en phase, donc un seul interrupt.

Connect RobotDyn AC Light Dimmer as follows:
Dimmer pin  --  Arduino pin
    VCC     --    5V
  GND     --    GND
  Z-C     --    D2
  PWM     --    D3

Glow sketch for AC Voltage dimmer with Zero cross detection
Based on the sketch by Charith Fernanado 
Adapted from RobotDyn: http://www.robotdyn.com
License: Creative Commons Attribution Share-Alike 3.0 License.
Attach the Zero cross pin of the module to Arduino External Interrupt pin
Select the correct Interrupt # from the below table (the Pin numbers are digital pins, NOT physical pins: digital pin 2 [INT0]=physical pin 4 and digital pin 3 [INT1]= physical pin 5)

Pin    |  Interrrupt # | Arduino Platform
---------------------------------------
2      |  0            |  All
3      |  1            |  All
18     |  5            |  Arduino Mega Only
19     |  4            |  Arduino Mega Only
20     |  3            |  Arduino Mega Only
21     |  2            |  Arduino Mega Only

In the program pin 2 is chosen
*/
const int AC_LOAD1 = 3;    // Output to Opto Triac pin
const int AC_LOAD2 = 5;
const int AC_LOAD3 = 6;
const int analogInPin = A0;  // Analog input pin
int dimming1 = 50;  // Dimming level (0-128)  0 = ON, 128 = OFF
int dimming2 = 50;
int dimming3 = 50;
int dimtime1 = 0;
int dimtime2 = 0;
int dimtime3 = 0;
int intensite_max = 5;
volatile int t_interrupt = 0; //moment ou se déclenche l'interrupt
volatile int ut_actuel = 0; //temps actuel en microseconde


const float pi = 3.1416;
int valeur_seuil = 115; //anciennement 150
int valeur_min = 600; //valeur du sensor à distance min
long periode1 = 6000; //en millisecondes
long periode2 = 6300; //en millisecondes
byte mode_nuit = 0;

int sensorValue = 0;        // value read from the pot 50 à 500
int outputValue = 0;        // value output to the PWM (analog out)
int sensorPresence = 2;
int sensorValue_avant = 0;
int sensorVariation = 0;
long tActuel = 0;
long t1 = 0;                //temps auquel intensité incrémente
long t2 = 0;
long t3 = 0;
long t4 = 0;
long deltaT1 = 0;
long deltaT2 = 0;
long periodeTemp = 1000;
int theta1 = 200;           //dephasage ms
byte flag = 0;
byte etat = LOW;
byte etat_deco = LOW;
byte lecture = LOW;
byte decision_lue = HIGH;

float thetax =  0;  //selon signe valeur initiale
int X1 = 0;        //variable fonction de t
int X2 = 0;        //variable fonction de t
int delta_x = 0;
int valeur_seuil_x = (int) pow((5*valeur_seuil)/(11.955*1023),-1.29);
int valeur_max_x = valeur_seuil_x - 5;
int valeur_min_x = (int) pow((5*valeur_min)/(11.955*1023),-1.29);
int x_neutre = (valeur_seuil_x + valeur_min_x)/2; //valeur neutre du ressort 
int ampx_max = valeur_seuil_x - x_neutre;
int ampx = ampx_max;    //de 0 à 35 environ. en cm
int x_avant = 0;

void setup()
{
  pinMode(AC_LOAD1, OUTPUT); // Set AC Load pin as output
  pinMode(AC_LOAD2, OUTPUT);
  pinMode(AC_LOAD3, OUTPUT);
  attachInterrupt(0, zero_crosss_int, RISING);  // Choose the zero cross interrupt # from the table above
  Serial.begin(9600);
  
  Serial.println(valeur_seuil_x);
  Serial.println(valeur_min_x);
  Serial.println(x_neutre);
  Serial.println(ampx_max);
}

// the interrupt function must take no parameters and return nothing
void zero_crosss_int()  // function to be fired at the zero crossing to dim the light
{
  t_interrupt = micros();
}

void triac_control()
{
  // Firing angle calculation : 1 full 50Hz wave =1/50=20ms 
  // Every zerocrossing thus: (50Hz)-> 10ms (1/2 Cycle) For 60Hz => 8.33ms (10.000/120)
  // 10ms=10000us
  // (10000us - 10us) / 128 = 75 (Approx) For 60Hz =>65
  ut_actuel = micros();
  dimming1 = intensite_max;
  dimtime1 = 65*dimming1;    // For 60Hz =>65
  if(ut_actuel - t_interrupt >= dimtime1){
    digitalWrite(AC_LOAD1, HIGH);   // triac firing
    delayMicroseconds(8);         // triac On propogation delay (for 60Hz use 8.33)
    digitalWrite(AC_LOAD1, LOW); 
  }
}

void affichage(int x1, int x2)
{
  // Firing angle calculation : 1 full 50Hz wave =1/50=20ms 
  // Every zerocrossing thus: (50Hz)-> 10ms (1/2 Cycle) For 60Hz => 8.33ms (10.000/120)
  // 10ms=10000us
  // (10000us - 10us) / 128 = 75 (Approx) For 60Hz =>65
  ut_actuel = micros();  
  dimming2 = map(x1, valeur_min_x, valeur_seuil_x, 100, intensite_max); 
  dimming3 = map(x2, valeur_min_x, valeur_seuil_x, 100, intensite_max); 
  dimtime2 = 65*dimming2;   // For 60Hz =>65
  dimtime3 = 65*dimming3;
  
  if(ut_actuel - t_interrupt >= dimtime2){
    digitalWrite(AC_LOAD2, HIGH);   // triac firing
    delayMicroseconds(8);         // triac On propogation delay (for 60Hz use 8.33)
    digitalWrite(AC_LOAD2, LOW); 
  }
  if(ut_actuel - t_interrupt >= dimtime3){
    digitalWrite(AC_LOAD3, HIGH);   // triac firing
    delayMicroseconds(8);         // triac On propogation delay (for 60Hz use 8.33)
    digitalWrite(AC_LOAD3, LOW); 
  }
}

//************************************

void loop()  { 
  // read the analog in value:
  sensorValue = analogRead(analogInPin);

  //initialisation du compteur lors d'une nouvelle détection
  if(sensorValue > valeur_seuil && lecture == LOW){
    t4 = millis();
    lecture = HIGH;
    decision_lue = LOW;
  }
  
  //mesure du temps de détection
  if(sensorValue > valeur_seuil){
    tActuel = millis();
    deltaT2 = tActuel - t4;
  }
  else{
    lecture = LOW;
  }

  //changement état lampe selon le temps de détection
  if(deltaT2 > 100 && deltaT2 < 800 && lecture == LOW && decision_lue == LOW){
    t1 = millis();
    decision_lue = HIGH;
    if(etat == LOW){
      etat = HIGH;
      Serial.println("lampe1:HIGH");
    }
    else{
      etat = LOW;
      Serial.println("lampe1:LOW");
    }
  }

  //changement état déco selon temps de détection
  if(deltaT2 >= 800 && lecture == LOW && decision_lue == LOW){
    t1 = millis();
    decision_lue = HIGH;
    if(etat_deco == LOW){
      etat_deco = HIGH;
      Serial.println("deco:HIGH");
    }
    else{
      etat_deco = LOW;
      Serial.println("deco:LOW");
    }    
  }
  
  //Activation de la lampe
  if(etat == HIGH){
    triac_control();
  }

  //Activation de la déco (affichage)
  if(etat_deco == HIGH){
    X1 = (int) ampx*cos(2*pi*deltaT1/periode1 + thetax) + x_neutre;
    X2 = (int) ampx*cos(2*pi*deltaT1/periode2 + thetax) + x_neutre;
    affichage(X1,X2);
  }
  
  tActuel = millis();
  deltaT1 = tActuel - t1;
}
