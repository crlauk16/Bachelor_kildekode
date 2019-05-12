
// Et program for å logge data fra Arduinoens analoge pins


// Den analoge pinnen for batteriets spenningsdeler
#define sensorPin A0
double voltage = 0.0;


// Den analoge pinnen for batteriets strømmåler
const int Strommaaler = A1;
int sensitivity = 100; //100 for ACS712 20A Module
int volt_read = 0;
int ACSoffset = 2500;  //spenningsavik
double voltage_strom = 0;
double Amps = 0;

// Den analoge pinnen for solcellepanelets spenningsdeler
#define sensorPin_sol A2
double voltage_sol = 0.0;


double capasity_Bat = 0;
double capasity_used = 0;
unsigned long timer_1 = 0;
unsigned long timer_2 = 0;


void setup(void)
{
  // Åpner serial port på 128000 og setter navn på kolonnene i Excel
  Serial.begin(128000);
  Serial.println("CLEARDATA");
  Serial.println("LABEL,PC_Time,Millisec_Time,Volt,Volt_sol,Current,Capasity");
  Serial.println("RESETTIMER");

}

void loop(void)
{

  timer_1 = millis();


  // Loop for gjennomsnittsverdier av alle målingene iløpet av 9 sekunder
  for (int j = 0; j < 30; j++) {


    for (int i = 0; i < 50; i++) {

      // Lagrer verdien på analog input A1
      volt_read = analogRead(Strommaaler);

      // Konverterer verdien fra inputten til amper
      voltage_strom = (volt_read / 1024.0) * 5000; //in mV
      Amps = ((voltage_strom - ACSoffset) / sensitivity) + Amps;


      // Lagrer verdien på analog input A0
      int sensorVal = analogRead(sensorPin);

      // Konverterer verdien fra inputten til volt
      voltage = (sensorVal / 1024.0) * 5.0 * 3.2795 + voltage; // 3.2795 forholdstall for motstanden i spenningsdeleren

      // Lagrer verdien på analog input A2
      int sensorVal_sol = analogRead(sensorPin_sol);

      // Konverterer verdien fra inputten til volt
      voltage_sol = (sensorVal_sol / 1024.0) * 5.0 * 8.52 + voltage_sol;  // 8.52 forholdstall for motstanden i spenningsdeleren

    }
    
    delay(300);
  }

  voltage = voltage / 1500;
  voltage_sol = voltage_sol / 1500;
  Amps = Amps / 1500;

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
  Serial.print(voltage_sol, 3);
  Serial.print(",");
  Serial.print(Amps, 3);
  Serial.print(",");
  Serial.println(capasity_Bat, 3);

}
