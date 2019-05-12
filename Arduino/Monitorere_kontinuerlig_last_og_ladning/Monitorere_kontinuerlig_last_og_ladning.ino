
// Et program for å logge data fra Arduinoens analoge pins, armere og kjøre drivsystemet

#include <Servo.h>

// Den analoge pinnen for batteriets spenningsdeler
const int sensorPin = A0;
double voltage = 0.0;

// Den analoge pinnen for batteriets strømmåler
const int Strommaaler = A1;
int sensitivity = 100; //100 for ACS712 20A Module
int volt_read = 0;
int ACSoffset = 2500; //spenningsavik
double voltage_strom = 0;
double Amps = 0;

// De analoge pinnene for thrusternes strømmålere
const int thruster_1 = A2;
const int thruster_2 = A3;
int volt_thruster_1 = 0;
int volt_thruster_2 = 0;
double voltage_strom_T1 = 0;
double voltage_strom_T2 = 0;
double Amps_T1 = 0;
double Amps_T2 = 0;


double capasity_Bat = 0;
double capasity_used = 0;
unsigned long timer_1 = 0;
unsigned long timer_2 = 0;

// PWM pinnene for ESCene
byte servoPin = 11;
byte servo2Pin = 10;
Servo servo;
Servo servo2;


void setup(void)
{

  // Åpner serial port på 128000 og setter navn på kolonnene i Excel
  Serial.begin(128000);
  Serial.println("CLEARDATA");
  Serial.println("LABEL,PC_Time,Millisec_Time,Volt,Current,Current_T1,Current_T2,Capasity");
  Serial.println("RESETTIMER");


  servo.attach(servoPin);
  servo2.attach(servo2Pin);

  // Sender et "stop" signal til ESCene
  servo.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);

  // Delay slik at ESCene rekker å gjenkjenne stop signalet
  delay(7000);

}

void loop(void) {

  timer_1 = millis();

  // Setter en signal vaerdi, burde være mellom 1100 og 1900
  int signal1 = 1260;

  servo.writeMicroseconds(signal1); // Sender et signal til ESCen.

  int signal2 = 1260;

  servo2.writeMicroseconds(signal2);

  // Loop for gjennomsnittsverdier av alle målingene iløpet av 9 sekunder
  for (int j = 0; j < 30; j++) {


    for (int i = 0; i < 50; i++) {

      // Lagrer verdien på analog input A1
      volt_read = analogRead(Strommaaler);

      // Konverterer verdien fra inputten til amper
      voltage_strom = (volt_read / 1024.0) * 5000; //in mV
      Amps = ((voltage_strom - ACSoffset) / sensitivity) + Amps;


      volt_thruster_1 = analogRead(thruster_1);
      voltage_strom_T1 = (volt_thruster_1 / 1024.0) * 5000;
      Amps_T1 = ((voltage_strom_T1 - ACSoffset) / sensitivity) + Amps_T1;


      volt_thruster_2 = analogRead(thruster_2);
      voltage_strom_T2 = (volt_thruster_2 / 1024.0) * 5000;
      Amps_T2 = ((voltage_strom_T2 - ACSoffset) / sensitivity) + Amps_T2;


      // Lagrer verdien på analog input A0
      int sensorVal = analogRead(sensorPin);

      // Konverterer verdien fra inputten til volt
      voltage = (sensorVal / 1024.0) * 5.0 * 3.2795  + voltage; // 3.2795 forholdstall for motstanden i spenningsdeleren

    }

    delay(300);
  }

  voltage = voltage / 1500;
  Amps = Amps / 1500;
  Amps_T1 = Amps_T1 / 1500;
  Amps_T2 = Amps_T2 / 1500;

  // Holder timer_2 konstant
  timer_2 = millis();
  timer_2 = timer_2 - timer_1;

  // Kalkulerer kapasiteten
  capasity_used = (Amps * timer_2) / 3600000;
  capasity_Bat = capasity_Bat + capasity_used;

  // Printer tiden og de forskjellige verdiene.
  Serial.print("DATA,TIME,TIMER,");

  Serial.print(voltage, 3);
  Serial.print(",");
  Serial.print(Amps, 3);
  Serial.print(",");
  Serial.print(Amps_T1, 3);
  Serial.print(",");
  Serial.print(Amps_T2, 3);
  Serial.print(",");
  Serial.println(capasity_Bat, 3);

}
