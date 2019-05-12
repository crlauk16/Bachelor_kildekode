
/* Dette programmet er for å anskaffe data fra analoge pins og anvende
   de i utregninger for et batteris kapasitet og effekten levert av et solcellepanel.

   Forså å bruke disse utregningene for å få ut en setting for et drivsystem.

   I tillegg finner programmet en lokalisjon som blir sendt via sattelitt.

   Programmet inneholder en funksjon som finner kompasskurs i form av radianer
   med roll og pitch kompansasjon.

   Programmet kobler seg også til en I2C adresse for å definere seg som en slave
   og innholder en funksjon som sender 14 verdier til en master.
*/



// Definering av nødvendige biblioteker
#include <IridiumSBD.h>
#include <TinyGPS++.h> // NMEA parsing: http://arduiniana.org
#include <Wire.h>
#include <SPI.h>
#include <I2C_Anything.h>
#include <Filters.h>



#define Magnetometer_mX0 0x03
#define Magnetometer_mX1 0x04
#define Magnetometer_mZ0 0x05
#define Magnetometer_mZ1 0x06
#define Magnetometer_mY0 0x07
#define Magnetometer_mY1 0x08


#define IridiumSerial Serial3
#define GPSSerial Serial2
#define GPSBaud 9600
#define DIAGNOSTICS true // Byttes mellom true og false avnhengig av om man vil se diagnotiseringen av sattelitt-modulen

// Tidsbruk mellom sendinger fra sattelitt-modul i sekunder
#define BEACON_INTERVAL 3600

IridiumSBD modem(IridiumSerial);
TinyGPSPlus tinygps;


// Den analoge pinnen for batteriets spenningsdeler
const int sensorPin = A0;
float voltage = 0.0;

// Den analoge pinnen for batteriets strømmåler
const int Strommaaler = A1;
int sensitivity = 100; //100 for 20A Modulen ACS712
int volt_read = 0;
int ACSoffset = 2500;
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

#define interval1 60000
#define interval2 300000


unsigned long timer_i1 = 0;
unsigned long timer_i2 = 0;
unsigned long timer_i3 = 0;
unsigned long timer_1 = 0;
unsigned long timer_2 = 0;


float variable_BatCap = 0;

float voltage_OC_bat = 0;
float effect_sol = 0;
int setting = 1;
int i = 0;

long int lattitude = 0;
long int longitude = 0;
int lat1, lat2, lat3, lat4;
int lon1, lon2, lon3, lon4;
long int kurs = 0;
int head1, head2, head3;
int satelitt = 0;

int teller = 1;

const float pi = 3.141592;


// Filtrerer ut forandringer raskere enn 5 Hz.
float filterFrequency = 1.0;

// Lager et lavpassfilter
FilterOnePole lowpassFilterX( LOWPASS, filterFrequency );
FilterOnePole lowpassFilterY( LOWPASS, filterFrequency );
FilterOnePole lowpassFilterZ( LOWPASS, filterFrequency );

const int MPU_addr = 0x68; // I2C adresse for MPU-6050 chippen
int16_t AcX, AcY, AcZ;
const float accRange = 4.0 * 9.81; //ms^-2
const float radiansToDegrees = 180 / 3.1415926535;
typedef struct
{
  float x = 0;
  float y = 0;
  float z = 0;
} vect3d;
vect3d acc;

float Roll , Pitch, Xtilt , Ytilt;
int mX0, mX1, mX_out;
int mY0, mY1, mY_out;
int mZ0, mZ1, mZ_out;
float heading, headingDegrees, headingFiltered, declination;
float Xm, Ym, Zm;
#define Magnetometer 0x1E //I2C 7bit adresse for HMC5883





void setup()
{


  // Starter serial portene
  Serial.begin(115200);
  while (!Serial);
  IridiumSerial.begin(19200);
  GPSSerial.begin(GPSBaud);

  Wire.begin();
  delay(100);

  // Skru på satelitt-modul
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);

  // Setup for Iridium modem
  if (modem.begin() != ISBD_SUCCESS)
  {
    Serial.println("Couldn't begin modem operations.");
    exit(0);
  }

  Wire.beginTransmission(Magnetometer);
  Wire.write(0x02); // Select mode register
  Wire.write(0x00); // Kontinuerlig målings modus
  Wire.endTransmission();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(1);     // (vekker MPU-6050)
  Wire.endTransmission(true);


  Wire.begin(0x48);              // Definerer denne Arduinoen som en I2C slave med adresse 0x48
  Wire.onRequest(requestEvent);

}

void loop()
{
  timer_1 = millis();



  // Leter etter GPS signal og henter ut bredd- og lengdegrad
  while (GPSSerial.available() > 0) {
    if (tinygps.encode(GPSSerial.read())) {
      displayInfo();
    }
  }

  // Henter ut kompasskursen
  kompass();


  i++;


  // Lagrer verdien på analog input A1
  volt_read = analogRead(Strommaaler);

  // Konverterer verdien fra inputten til amper
  voltage_strom = (volt_read / 1024.0) * 5000; //in mV
  Amps = ((voltage_strom - ACSoffset) / sensitivity); // For batteriet


  volt_thruster_1 = analogRead(thruster_1);
  voltage_strom_T1 = (volt_thruster_1 / 1024.0) * 5000;
  Amps_T1 = ((voltage_strom_T1 - ACSoffset) / sensitivity); // For thruster 1


  volt_thruster_2 = analogRead(thruster_2);
  voltage_strom_T2 = (volt_thruster_2 / 1024.0) * 5000;
  Amps_T2 = ((voltage_strom_T2 - ACSoffset) / sensitivity); // For thruster 2


  // Lagrer verdien på analog input A0
  int sensorVal = analogRead(sensorPin);

  // Konverterer verdien fra inputten til volt
  voltage = (sensorVal / 1024.0) * 5.0 * 3.2795; // 3.2795 forholdstall for motstanden i spenningsdeleren

  delay(5);

  // Kalkulerer forskjellen mellom timer 1 og timer 2 for å få en verdi på målesyklussen til loopen
  timer_2 = millis();
  timer_2 = timer_2 - timer_1;


  voltage_OC_bat = voltage;
  effect_sol = (Amps_T1 + Amps_T2) * 12 + (Amps * voltage) + effect_sol;

  capasity_used = (Amps * timer_2) / 3600000; // I ampertimer (Ah)
  capasity_Bat = capasity_Bat + capasity_used;


  // Intervallet (hvert minutt) som sender ut en setting basert på verdiene til batterits kapasitet og effekten solcellepanelet utgir.
  if (millis() > timer_i1 + interval1) {
    timer_i1 = millis();

    effect_sol = effect_sol / (i) ;


    if ( voltage_OC_bat > 13.65) {
      capasity_Bat = 20;
    }


    if ( voltage_OC_bat < 11.8) {
      capasity_Bat = 2.5;
    }


    if (capasity_Bat < 8.75 || effect_sol < 30) {
      setting = 1;
    }

    if (capasity_Bat > 8.75 && capasity_Bat < 17.5 ) {
      setting = 2;
    }

    if (capasity_Bat > 17.5 && effect_sol > 40) {
      setting = 3;
    }

    effect_sol = 0;
    i = 0;
  }


  // Intervallet (hvert 30 minutt) for når satelitt-modulen skal sende posisjon
  if (millis() > timer_i2 + interval2) {
    timer_i2 = millis();



    // Begynner å lete etter GPS signal
    Serial.println("Beginning to listen for GPS traffic...");

    // Leter etter GPS signal for opp til 7 minuter
    while ((!tinygps.location.isValid() || !tinygps.date.isValid()) &&
           millis() - timer_1 < 7UL * 60UL * 1000UL)
    {
      if (GPSSerial.available())
        tinygps.encode(GPSSerial.read());
    }

    // Fikk vi GPS signal?
    if (!tinygps.location.isValid())
    {
      Serial.println(F("Could not get GPS fix."));
      Serial.print(F("GPS characters seen = "));
      Serial.println(tinygps.charsProcessed());
      Serial.print(F("Checksum errors = "));
      Serial.println(tinygps.failedChecksum());
      return;
    }

    Serial.println(F("A GPS fix was found!"));

    // Starter å kommunisere med satelitt-modulen og skrur den på
    Serial.println("Beginning to talk to the RockBLOCK...");
    char outBuffer[60]; // Prøv å hold meldingen liten
    sprintf(outBuffer, "%d%02d%02d%02d%02d%02d,%s%u.%09lu,%s%u.%09lu,%lu,%ld",
            tinygps.date.year(),
            tinygps.date.month(),
            tinygps.date.day(),
            tinygps.time.hour(),
            tinygps.time.minute(),
            tinygps.time.second(),
            tinygps.location.rawLat().negative ? "-" : "",
            tinygps.location.rawLat().deg,
            tinygps.location.rawLat().billionths,
            tinygps.location.rawLng().negative ? "-" : "",
            tinygps.location.rawLng().deg,
            tinygps.location.rawLng().billionths,
            tinygps.speed.value() / 100,
            tinygps.course.value() / 100);

    Serial.print("Transmitting message '");
    Serial.print(outBuffer);
    Serial.println("'");

    int err = modem.sendSBDText(outBuffer);
    if (err != ISBD_SUCCESS)
    {
      Serial.print("Transmission failed with error code ");
      Serial.println(err);
    }


  }

  // Deler opp alle variablene som skal sendes over I2C slik at verdiene ikke oversiteg 255.
  lat1 = lattitude % 100;
  lat2 = (lattitude / 100) % 100;
  lat3 = (lattitude / 10000) % 100;
  lat4 = (lattitude / 1000000) % 100;

  lon1 = longitude % 100;
  lon2 = (longitude / 100) % 100;
  lon3 = (longitude / 10000) % 100;
  lon4 = (longitude / 1000000) % 100;

  head1 = kurs % 100;
  head2 = (kurs / 100) % 100;


}


// Funksjon for å sende alle 14 variabler over til master.
void requestEvent()
{
  I2C_writeAnything (lat4);
  I2C_writeAnything (lat3);
  I2C_writeAnything (lat2);
  I2C_writeAnything (lat1);

  I2C_writeAnything (lon4);
  I2C_writeAnything (lon3);
  I2C_writeAnything (lon2);
  I2C_writeAnything (lon1);

  I2C_writeAnything (head2);
  I2C_writeAnything (head1);

  I2C_writeAnything (setting);

  I2C_writeAnything (satelitt);

  // Verifikasjon for at koden kontinuerlig sender verdier.
  if (teller == 1) {
    teller = 2;
  }
  else {
    teller = 1;
  }


  I2C_writeAnything (teller);


}



// Funksjon for å skaffe posisjon

void displayInfo () {
  if (tinygps.location.isValid())
  {
    lattitude = tinygps.location.lat() * 1000000; // Ganger med 1000000 for å ikke få et desimaltall
    longitude = tinygps.location.lng() * 1000000;

  }

}

// Funksjon for å skaffe kurs med kompansasjon for roll og pitch.

void kompass () {


  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // Starter med register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // Ber om et total av 14 registrer
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  lowpassFilterX.input(AcX);
  lowpassFilterY.input(AcY);
  lowpassFilterZ.input(AcZ);

  // Skalerer akselerasjons verdiene
  acc.x = (float)lowpassFilterX.Y / 65535.0 * accRange;
  acc.y = (float)lowpassFilterY.Y / 65535.0 * accRange;
  acc.z = (float)lowpassFilterZ.Y / 65535.0 * accRange;

  // Kalkulerer rotasjon fra graviatsjons vektor

  float roll  = atan2(acc.y, sqrt(acc.x * acc.x + acc.z * acc.z));
  float pitch = atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z));
  float roll_print = roll - 0.17;
  float pitch_print = pitch + 0.45;

  //---- X-Axis
  Wire.beginTransmission(Magnetometer); // Sender til enhet
  Wire.write(Magnetometer_mX1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mX0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // Sender til enhet
  Wire.write(Magnetometer_mX0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mX1 = Wire.read();
  }
  //---- Y-Axis
  Wire.beginTransmission(Magnetometer); // Sender til enhet
  Wire.write(Magnetometer_mY1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mY0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // Sender til enhet
  Wire.write(Magnetometer_mY0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mY1 = Wire.read();
  }

  //---- Z-Axis
  Wire.beginTransmission(Magnetometer); // Sender til enhet
  Wire.write(Magnetometer_mZ1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mZ0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // Sender til enhet
  Wire.write(Magnetometer_mZ0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer, 1);
  if (Wire.available() <= 1)
  {
    mZ1 = Wire.read();
  }




  //---- X-Axis
  mX1 = mX1 << 8;
  mX_out = mX0 + mX1; // Rå data
  // Fra datasheet: 0.92 mG/digit
  Xm = mX_out * 0.00092; // Gauss enhet
  //* Jorden magnetiske felt varierer fra 0.25 til 0.65 Gauss, så dette er de omtrentelige verdiene vi trenger.
  //---- Y-Axis
  mY1 = mY1 << 8;
  mY_out = mY0 + mY1;
  Ym = mY_out * 0.00092;
  //---- Z-Axis
  mZ1 = mZ1 << 8;
  mZ_out = mZ0 + mZ1;
  Zm = mZ_out * 0.00092;
  // ==============================

  Xtilt = ((Xm * cos(pitch_print)) + (Zm * sin(pitch_print)));
  Ytilt = ((Xm * sin(roll_print) * sin(pitch_print)) + ((Ym * cos(roll_print)) - (Zm * sin(roll_print) * cos(pitch_print))));

  // Kalkulerer kurs
  heading = atan2(Ytilt, Xtilt);

  // Korregerer kursen med deklinasjons vinkelen som varierer på posisjon.
  // I Grimstad er den 2 grader => 0.03490658503 rad
  declination = 0.03490658503;
  heading += declination;

  // Korregerer for negative verdier
  if (heading < 0) heading += 2 * PI;
  
  headingDegrees = heading * 180 / PI; // kurs i grader
  // Filtrer bort ujevne verdier
  headingFiltered = headingFiltered * 0.85 + headingDegrees * 0.15;
  // Gjør kurs om til radianer
  kurs = headingFiltered * (pi / 180) * 100;

  delay(100);


}




#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif
