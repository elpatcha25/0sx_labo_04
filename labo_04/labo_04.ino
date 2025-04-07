#include <HCSR04.h>
#include <AccelStepper.h>
#include <LCD_I2C.h>
#include <string.h>


LCD_I2C lcd(0x27, 20, 21);
HCSR04 hc(6, 7);

#define MOTOR_INTERFACE_TYPE 4
#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11

enum EtatPorte { FERMEE,
                 OUVERTURE,
                 OUVERTE,
                 FERMETURE };
EtatPorte currentState = FERMEE;

const String NUM_ETUDIANT = "2407822";
const String cm = " cm";
const float tour = 2038.0;
const int MAX_DEGREE = 170;
const int MIN_DEGREE = 10;
const int POSITION_OUVERTE = (tour * MAX_DEGREE) / 360.0;
const int POSITION_FERMEE = (tour * MIN_DEGREE) / 360.0;
const int MAX_SPEED = 1000;
const int SET_SPEED = 255;
const int ACCELERATION = 200;
const int TEMPS_DISTANCE = 50;
const int MAX_CENTIMETRE = 60;
const int MIN_CENTIMETRE = 30;
const unsigned long TEMPS_OUVERTE = 3000;

int deg = 0;
int dist = 0;
int oldDistance = -1;
unsigned long tempsDistance = 0;
unsigned long tempsOuverte = 0;

AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();

  myStepper.setMaxSpeed(MAX_SPEED);
  myStepper.setAcceleration(ACCELERATION);
  myStepper.setSpeed(SET_SPEED);
  myStepper.setCurrentPosition(0);

  afficherDemarrage();
  delay(2000);
  lcd.clear();
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - tempsDistance > TEMPS_DISTANCE) {
    tempsDistance = currentTime;
    dist = hc.dist();
  }

  afficherLCD();
  gestionEtat(currentTime);
  affichageSerie(currentTime);

  myStepper.run();
}


void afficherLCD() {
  
  lcd.setCursor(0, 0);
  lcd.print("Dist  : ");
  lcd.print(String(dist) + "cm ");

  lcd.setCursor(0, 1);
  if (myStepper.distanceToGo() == 0) {
    if (abs(myStepper.currentPosition() - POSITION_FERMEE) < 5) {
      lcd.print("Ferme           ");
    } else if (abs(myStepper.currentPosition() - POSITION_OUVERTE) < 5) {
      lcd.print("Ouverte         ");
    }
  } else {
    deg = map(abs(myStepper.currentPosition()), 0, POSITION_OUVERTE, MIN_DEGREE, MAX_DEGREE);
    lcd.print("Angle : ");
    lcd.print(deg);
    lcd.print((char)223);
    lcd.print("      ");
  }
}

void affichageSerie(unsigned long now) {

  static unsigned long lastTime = 0;
  int rate = 100;

  if (now - lastTime < rate) return;

  lastTime = now;

  Serial.println("etd:" + NUM_ETUDIANT + ",dist:" + String(dist) + ",deg:" + String(deg));
}

void gestionEtat(unsigned long currentTime) {

  switch (currentState) {
    case FERMEE:
      if (dist < MIN_CENTIMETRE && dist > 0) {
        currentState = OUVERTURE;
        myStepper.enableOutputs();
        myStepper.moveTo(POSITION_OUVERTE);
      }
      break;

    case OUVERTURE:
      if (myStepper.distanceToGo() == 0) {
        currentState = OUVERTE;
        tempsOuverte = currentTime;
        myStepper.disableOutputs();
      }
      break;

    case OUVERTE:
      if (dist > MAX_CENTIMETRE) {
        currentState = FERMETURE;
        myStepper.enableOutputs();
        myStepper.moveTo(POSITION_FERMEE);
      }
      break;

    case FERMETURE:
      if (myStepper.distanceToGo() == 0) {
        currentState = FERMEE;
        myStepper.disableOutputs();
      }
      break;
  }
}


void afficherDemarrage() {
  lcd.setCursor(0, 0);
  lcd.print(NUM_ETUDIANT);
  lcd.setCursor(0, 1);
  lcd.print("LAB 4B");
}