#define PI 3.1415926535897932384626433832795
double Pt = 2700, d0 = 400, PLE = 4.5;
long freq;

int window = 10;
int Lsum[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, Rsum[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, Tsum[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int Ltime[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, Rtime[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, Ttime[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int Li[10], Ri[10], Ti[10], delTime = 800;

char tag[2];
int rssi, p, q;
double tdis;
double CdB;
long timeStamp;
int index = 0;


#include "SparkFun_UHF_RFID_Reader.h"
RFID rfidModule;

#include <SoftwareSerial.h>
SoftwareSerial softSerial(2, 3);  //RX, TX

#include "MINMAX.h"
MINMAX mmL1, mmL2;

#include "MegunoLink.h"
#include "Filter.h"


ExponentialFilter<long> xFilter(5, 0);
ExponentialFilter<long> yFilter(5, 0);

#define rfidSerial softSerial

#define rfidBaud 38400

#define moduleType ThingMagic_M6E_NANO

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  if (setupRfidModule(rfidBaud) == false) {
    Serial.println(F("Module failed to respond. Please check wiring."));
    while (1)
      ;
  }

  rfidModule.setRegion(REGION_INDIA);

  rfidModule.setReadPower(2700);  //5.00 dBm(500) to 27.00 dBm(2700)
  //rfidModule.setWritePower(2700);

  Serial.println(F("Press a key to begin scanning for tags."));
  while (!Serial.available())
    ;
  Serial.read();

  rfidModule.startReading();
}


void loop() {
  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;

  if (rfidModule.check() == true) {
    byte responseType = rfidModule.parseResponse();

    if (responseType == RESPONSE_IS_KEEPALIVE) {
      Serial.println(F("Scanning"));
    } else if (responseType == RESPONSE_IS_TAGFOUND) {

      rssi = rfidModule.getTagRSSI();
      freq = rfidModule.getTagFreq();
      timeStamp = rfidModule.getTagTimestamp();

      String tx = "", ty = "";

      for (byte x = 0; x < 2; x++) {
        char temp = char(rfidModule.msg[31 + x]);
        if (isAlphaNumeric(temp))
          tag[x] = temp;
      }

      for (byte x = 2; x < 6; x++) {
        char temp = char(rfidModule.msg[31 + x]);
        tx += temp;
      }

      for (byte x = 8; x < 12; x++) {
        char temp = char(rfidModule.msg[31 + x]);
        ty += temp;
      }

      CdB = 20 * log10((30000000 / freq) / (4 * PI * d0));
      tdis = d0 * pow(10, ((rssi - Pt / 100 + CdB) / (-10 * PLE)));
      tdis /= 2000;

      index = tag[1] - '0';
      if (tag[0] == 'L') {
        if (timeStamp - Ltime[index] > delTime || Li[index] == 10) {
          Lsum[index] = 0;
          Li[index] = 0;
        }

        Ltime[index] = timeStamp;
        Lsum[index] += rssi;
        Li[index]++;
        Serial.print(tag[0]);
        Serial.print(index);
        Serial.print(",");
        Serial.print(tx);
        Serial.print(",");
        Serial.print(ty);
        Serial.print(",");
        Serial.print(Lsum[index] / Li[index]);
        //Serial.print(rssi);
        Serial.println();
      } else if (tag[0] == 'R') {
        if (timeStamp - Rtime[index] > delTime || Ri[index] == 10) {
          Rsum[index] = 0;
          Ri[index] = 0;
        }

        Rtime[index] = timeStamp;
        Rsum[index] += rssi;
        Ri[index]++;
        Serial.print(tag[0]);
        Serial.print(index);
        Serial.print(",");
        Serial.print(tx);
        Serial.print(",");
        Serial.print(ty);
        Serial.print(",");
        Serial.print(Rsum[index] / Ri[index]);
        Serial.println();
      } else if (tag[0] == 'T') {
        if (timeStamp - Ttime[index] > delTime || Ti[index] == 10) {
          Tsum[index] = 0;
          Ti[index] = 0;
        }
        Ttime[index] = timeStamp;
        Tsum[index] += rssi;
        Ti[index]++;
        Serial.print(tag[0]);
        Serial.print(index);
        Serial.print(",");
        Serial.print(tx);
        Serial.print(",");
        Serial.print(ty);
        Serial.print(",");
        Serial.print(Tsum[index] / Ti[index]);
        Serial.println();
      }

    } else if (responseType == ERROR_CORRUPT_RESPONSE) {
      Serial.println("Bad CRC");
    } else if (responseType == RESPONSE_IS_HIGHRETURNLOSS) {
      Serial.println("High return loss, check antenna!");
    } else {
      Serial.println("Unknown error");
    }
  }
}

boolean setupRfidModule(long baudRate) {
  rfidModule.begin(rfidSerial, moduleType);
  rfidSerial.begin(baudRate);
  while (rfidSerial.available())
    rfidSerial.read();
  rfidModule.getVersion();

  if (rfidModule.msg[0] == ERROR_WRONG_OPCODE_RESPONSE) {
    rfidModule.stopReading();
    //Serial.println(F("Module continuously reading. Asking it to stop..."));
    delay(1500);
  } else {
    rfidSerial.begin(115200);
    rfidModule.setBaud(baudRate);
    rfidSerial.begin(baudRate);
    delay(250);
  }
  rfidModule.getVersion();
  if (rfidModule.msg[0] != ALL_GOOD)
    return false;
  rfidModule.setTagProtocol();
  rfidModule.setAntennaPort();
  return true;
}
