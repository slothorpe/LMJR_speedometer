/* Ansteuerung fuer Futura Classico LI Tacho
 * Basis Sabvoton- (oder ungetestet Kelly-) Controller
 * Holger Lesch, August 2019
 * holger.lesch@gmx.net
 * letzte Aenderung 27.10.2019
 */

#include <FreqCount.h>

// Arduino PINs
// Frequenzmessung am NANO und UNO: PIN 5
// Wird automatisch ueber die FreqCount Bibliothek gesetzt
const byte ledPin = 13;  // Aktivitaetsanzeige
const byte outputPin = 4; // Ausgang Rtg. Tacho
const byte highBrakePin = A0;
const byte ThrottleAnalogPin = A4; // Holger 4, Karl 5
const byte gear3Pin = A3;
const byte gear1Pin = A2;

// Aufbau des Datenprotokoll zum Tacho
const int basis1 = 0x00;
const int basis2 = 0x00;
const int timePause = 49; //in ms
const bool msbFirst = true; // Reihenfolge der Bits.
const int timeLow = 498; //in us

// Berechnung von Geschwindigkeit / U/min
const int freqCountTime = 250; // Messinterval in ms fuer Freq.Messung, je laenger, je genauer, dafuer aber selteren Update der Geschw.
const int freqFactor = 1000/freqCountTime; // fuer die Umrechnung des Zaehlerwerts in die tats.- Frequenz
const unsigned long radUmfang = 1350l; //Radumfang in mm lt. Tabelle
//const unsigned long hallDivider = 27l;    // Original Motor - Hall-Geber-Impulse / Umdrehung = Anzahl Pole Motor ?
const unsigned long hallDivider = 16l;    // QS 205 V3 Hall-Geber-Impulse / Umdrehung = Anzahl Pole Motor ?
const unsigned long maxSpeed = 75l;
const float maxAccelaration = 0.5*8.91; // max. Beschleunigungdie möglich ist in m/s^^2 = ca. 0.5G
const unsigned long maxSpeedChange = (unsigned long)((((maxAccelaration*3.6)/(float)freqFactor))+0.5);

// Schwellwerte fuer Throttle, Bremse usw.
const int throtNotaus =  50;      // wenn der Notaus gedrueckt ist -> Throttle auf Masse
const int throtInactive = 240;    // knapp oberhalb des Wertes fuer Gasgriff ganz losgelassen, 1,14V = 233
const int brakeActive = 512;      // 12V high-brake
const byte gearActive = 128;      // Schalter fuer Fahrstufen <=high 
const byte minRekuSpeed = 2;      // Minimale Geschwindigkeit bei der rekupiert wird

// Ausgabepuffer
char buf[80];
byte outbuf[11];


// primitve Ein- und Ausschaltung der Debug-Ausgabe ueber USB
#undef DEBUG
#ifdef DEBUG
#define DEBUGPLN Serial.println
#define DEBUGP Serial.print
#define SPRINTF sprintf
#else
#define DEBUGPLN(...)
#define DEBUGP(...)
#define SPRINTF(...)
#endif

void setup() {                                                                                                                                                                                                                                                                                      
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Frequenzzaehler initialisieren
  FreqCount.begin(freqCountTime);    

  // Ausgabe-Pins initialisieren
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin,0);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin,0);

  // Mess-Pins initialisieren
  pinMode(highBrakePin,INPUT);
  pinMode(gear3Pin,INPUT_PULLUP);
  pinMode(gear1Pin, INPUT_PULLUP);
  pinMode(ThrottleAnalogPin,INPUT);
  DEBUGPLN(maxAccelaration);
  DEBUGPLN(maxSpeedChange);
}

 void writeStart()
{
  digitalWrite(outputPin,1);
  delayMicroseconds(timeLow * 2);
  digitalWrite(outputPin,0);
}

void write0()
{
  digitalWrite(outputPin,0);
  delayMicroseconds(timeLow * 2);
  digitalWrite(outputPin,1);
  delayMicroseconds(timeLow);
}

void write1()
{
  digitalWrite(outputPin,0);
  delayMicroseconds(timeLow);
  digitalWrite(outputPin,1);
  delayMicroseconds(timeLow * 2);
}


void writeSequence(byte* seq, int len) {
  int i,j;
  byte out;

  for (i=0;i<len; i++) {
    out = seq[i];
    for (j=0;j<8;j++) {
      if (msbFirst) {
        if (out & 0x80) write1 (); else write0();
        out=out<<1;
      } else {
        if (out & 1) write1 (); else write0();
        out=out>>1;
      }
    }
  }
  digitalWrite(outputPin,0);
} 


void loop() {
  int i;
  static byte out; 
  bool handbrake=false;
  bool notaus=false;
  bool motorbrake = false;
  int gear = 2;
  unsigned long tint; //tachowert intern fuer Berechnung
  static byte tacho=0;
  int checksum = 0;
  static int count =0;
  unsigned long freq;
  int rounds;
  int throttle;

  // Geschwindigkeit bestimmen
  if (FreqCount.available()) {
    freq = FreqCount.read()*freqFactor ; // Anzahl Hall-Impulse / Sekunde
    rounds = (int)(freq*60l)/hallDivider;  // U/min
    tint = (freq*radUmfang*36l)/(hallDivider*10000l);  // km/h
    //SPRINTF(buf,"Frequenz: %4ld - Tacho: %2d - U/min: %4d - ",freq,(int)tint, rounds);
    if (tint <= maxSpeed) {
      // Messfehler abschwächen, beim V-Max gibt es manchmal Stoerungen
      if (tint < (unsigned long)tacho || abs(tint-(unsigned long int)tacho) <= maxSpeedChange) {
        tacho = (byte)tint;
      } else {
          tacho += (byte)(maxSpeedChange/2l);
      } 
    }
    SPRINTF(buf,"Frequenz: %4ld - Tacho: %2d (%2d) - U/min: %4d - ",freq,(int)tint,tacho, rounds);
  } else {
    SPRINTF(buf,"                                                ");
  }
  DEBUGP(buf);
    
  // restliche Anzeigen bestimmen
  throttle=analogRead(ThrottleAnalogPin);  // Stellung des Gasgriffs
  notaus = throttle < throtNotaus; // Notaus legt Throttle auf Masse
  handbrake = analogRead(highBrakePin) > brakeActive;   // high-brake 12V


  // Fahrstufen auswerten
  if (analogRead(gear1Pin) < gearActive) {
    gear=1;
  }
  if (analogRead(gear3Pin) < gearActive) {
    gear=3;
  }

  // Wenn Gas auf Null und ueber bestimmter Drehzahl -> Slide Recharge
  // Wenn Handbremse gezogen und ueber Stillstand -> Motorbremse
  // In beiden Faellen Symbol Reku/Motorbremse im Tacho an
  if ((throttle > throtNotaus && throttle <= throtInactive && tacho >= minRekuSpeed) ||
      (handbrake && tacho >= minRekuSpeed) ) {
      motorbrake = true;
   }
  
  SPRINTF(buf,"Throttle: %3d, Notaus: %d - Handbremse: %d - Motorbremse/Reku: %d - Gang: %d - ",throttle, notaus,handbrake,motorbrake,gear);
  DEBUGP(buf);


  // datenpaket zusammenbauen
  outbuf[0]=0x0a;
  outbuf[1] = count++;
  outbuf[2]= basis1;
  outbuf[3] = basis1;
  outbuf[4] = outbuf[3] | gear;
  if (handbrake) outbuf[4] ^=0x20;
  if (notaus) outbuf[3] ^=0x20;
  outbuf[5] = outbuf[3];
  if (motorbrake) outbuf[3] ^=0x01;
  outbuf[6]= basis1;
  outbuf[7]= basis2 + tacho/16;
  outbuf[8]= basis2 + (tacho%16)*0x10;
  // Korrekturen der Tachoanzeige vom Classico LI, nach Geschmack ein oder auskommentieren
  // Unterhalb von 10km/h zeigt der Tacho zu wenig an
  if (tacho<10 && tacho>0) outbuf[8]+=8;
  // Oberhalb von 40km/h zeigt der Tacho zu viel an
  if (tacho>39) outbuf[8]-=8;
  if (tacho>45) outbuf[8]-=8;
  //if (tacho>55) outbuf[8]-=8;
  outbuf[9]= basis2;
  outbuf[10] = basis2;

  // Senden beginnt
  digitalWrite(ledPin,1);
  writeStart();
  checksum = 0;  
  for (i=0;i<11; i++) {
    // XOR Checksumme mitfuehren
    checksum=checksum ^ outbuf[i];
    
    SPRINTF (buf," %02x",outbuf[i]);
    DEBUGP(buf);
    writeSequence(&outbuf[i],1);
  }

  // XOR Checksumme senden
  out = checksum;
  
  SPRINTF (buf," %02x",out);
  DEBUGP(buf);
  writeSequence(&out,1);
  DEBUGPLN ();
  
  // Senden fertig
  digitalWrite(ledPin,0);  
  
  delay(timePause);
}
