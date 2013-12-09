/**
 * GumboNode for the Arduino Uno.
 * GumboNodes are simple extremely low-power sensor networks consisting of a Microcontroller and a CC2500 wireless transceiver.
 * Since the Uno has more pins and serial output, we can use it to get more feedback from the CC2500.
 * There is also no power optimization, as this is a 'debugging' node. This is only useful if it's plugged into a computer.
 * For maximum size and power optimization, use GumboTiny.
 * Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 * SS - > 10
 *
 * Configurable:
 * CSN -> 7
 */
#include "cc2500_REG.h"
#include "cc2500_VAL.h"

#include <SPI.h>

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_SWOR    0x38
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F

#define GUMBO_ID 25
#define GUMBO_SIZE 100
#define TX_TIMEOUT 10 // in milliseconds

typedef struct {
  byte id;
  int8_t sensorReading;
  int8_t sensorReading2;
  int8_t rssi;
  byte hops;
  boolean staleData;
  long packetArrivalTime;
} GumboNode;

GumboNode gumboData[GUMBO_SIZE];
long sleepTime = 1000;
long wakeLength = 1000;
long staleDataTime = 20*wakeLength;
long lastSync = 0;
long syncDataLossInterval = 3*wakeLength;
long overWriteDataInterval = 5*wakeLength;
long previousWakeMillis = 0; 
long previousPrintMillis = 0; 
long previousTXTimeoutMillis = 0;        // will store last time data wa
byte cyclesSinceLastData, GDO0_State, sendSync, gumboListIndex;

void setup(){
  Serial.begin(115200);
  
  // Setup 
  pinMode(SS,OUTPUT);
  SPI.begin();
  digitalWrite(SS,HIGH);
  Serial.println("Initializing Wireless..");
  init_CC2500();
  Read_Config_Regs();
  initGumboList();
  sendSync = true;
  getTemp();    
  averageTemperature();
  Serial.println("GumboNode Uno 0.9");  
}

void loop(){
  if(sendSync) {
     gumboSendSync();
     sendSync = false;
  }
  // Immediately listen
  listenForPacket();
  unsigned long currentMillis = millis();
  gumboSendData(0);
  //gumboListIndex++;
  listenForPacket();
  if(currentMillis - lastSync > syncDataLossInterval) {
      // SYNC LOST
      lastSync = 0;
      //waitForSync();
  }
  if(currentMillis - previousWakeMillis > wakeLength) {
    previousWakeMillis = currentMillis;
    gumboSleep();
  }
  if(currentMillis - previousPrintMillis > staleDataTime) {
    previousPrintMillis = currentMillis;
    printAllData();
    averageTemperature();
  }
}

void listenForPacket() {
  SendStrobe(CC2500_RX);
  WriteReg(REG_IOCFG1,0x01);
  delay(20);
  unsigned long currentMillis = millis();
  if (digitalRead(MISO)) {      
    char PacketLength = ReadReg(CC2500_RXFIFO);
    char recvPacket[PacketLength];
    if(PacketLength == 8) {
      Serial.println("Packet Received!");
      //Serial.print("Packet Length: ");
      //Serial.println(PacketLength, DEC);
      //Serial.print("Data: ");
      for(int i = 1; i < PacketLength; i++){
        recvPacket[i] = ReadReg(CC2500_RXFIFO);
        //Serial.print(recvPacket[i], DEC);
        //Serial.print(" ");
      }
      //Serial.println(" ");
      byte rssi = ReadReg(CC2500_RXFIFO);
      byte lqi = ReadReg(CC2500_RXFIFO);
      byte PQTReached = isPQTReached();
      if(recvPacket[1] == 'd' && PQTReached) {
        if (addIfHigherQuality(recvPacket[3], recvPacket[4], recvPacket[5], lqi, recvPacket[7])) {
          Serial.println("Data updated!");
        } else { 
          Serial.println("Data discarded");
        }
      } else if (recvPacket[1] == 'w' && PQTReached) {
        lastSync = millis();
        Serial.println("Sync Received!");
      } else {
        
      }
    }
    
    // Make sure that the radio is in IDLE state before flushing the FIFO
    // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
    SendStrobe(CC2500_IDLE);
    // Flush RX FIFO
    SendStrobe(CC2500_FRX);
  } else {
    
  }
}

void gumboSendSync() {
  gumboSend(0, true);
}

void gumboSendData(byte gumboDataID) {
  gumboSend(gumboDataID, false);
}

void gumboSend(byte gumboDataID, boolean sync) {
  // Not valid data
  if(gumboData[gumboDataID].id == 0) return;
  
  WriteReg(REG_IOCFG1,0x06);
  // Make sure that the radio is in IDLE state before flushing the FIFO
  SendStrobe(CC2500_IDLE);
  // Flush TX FIFO
  SendStrobe(CC2500_FTX);
  // prepare Packet
  int length = 8;

  unsigned char packet[length];
  if(sync) {
    // First Byte = Length Of Packet
    packet[0] = length;
    packet[1] = 'w';
    packet[2] = GUMBO_ID;
    packet[3] = 0;
    packet[4] = 0;
    packet[5] = 0;
    packet[6] = 0;
    packet[7] = 0;
  }
  else {
    // First Byte = Length Of Packet
    packet[0] = length;
    packet[1] = 'd';
    packet[2] = GUMBO_ID;
    packet[3] = gumboData[gumboDataID].id;
    packet[4] = gumboData[gumboDataID].sensorReading;
    packet[5] = gumboData[gumboDataID].sensorReading2;
    packet[6] = gumboData[gumboDataID].rssi;
    if (gumboData[gumboDataID].id == GUMBO_ID) {
      packet[7] = 0;
    } else {
      packet[7] = gumboData[gumboDataID].hops + 1;
    }
  }
  
  // SIDLE: exit RX/TX
  SendStrobe(CC2500_IDLE);
  
  //Serial.println("Transmitting ");
  for(int i = 0; i < length; i++)
  {	  
      WriteReg(CC2500_TXFIFO,packet[i]);
  }
  // STX: enable TX
  SendStrobe(CC2500_TX);
  // Wait for GDO0 to be set -> sync transmitted
  previousTXTimeoutMillis = millis();
  while (!digitalRead(MISO) && (millis() - previousTXTimeoutMillis) <= TX_TIMEOUT) {
  }
  previousTXTimeoutMillis = millis();
  // Wait for GDO0 to be cleared -> end of packet
  while (digitalRead(MISO) && (millis() - previousTXTimeoutMillis) <= TX_TIMEOUT) {
  }
  //Serial.println("Finished sending");
  SendStrobe(CC2500_IDLE);
}

byte addIfHigherQuality(byte id, byte sensorReading, byte sensorReading2, byte lqi, byte hops) {
  byte listLocation = getListLocation(id);
  if (gumboData[listLocation].id == 0) {
    gumboData[listLocation].id = id;
    gumboData[listLocation].sensorReading = sensorReading;
    gumboData[listLocation].sensorReading2 = sensorReading2;
    gumboData[listLocation].rssi = lqi;
    gumboData[listLocation].hops = hops;
    Serial.print("Found no data. adding at ");
    Serial.println(listLocation);
    printNode(listLocation);
    return 1;
  } else if (hops <= gumboData[listLocation].hops){
    gumboData[listLocation].id = id;
    gumboData[listLocation].sensorReading = sensorReading;
    gumboData[listLocation].sensorReading2 = sensorReading2;
    gumboData[listLocation].rssi = lqi;
    gumboData[listLocation].hops = hops;
    Serial.print("Found data at ");
    Serial.println(listLocation);
    printNode(listLocation);
    return 1;
  } else {
    Serial.print("No Good!");
    Serial.print(listLocation);
    Serial.print(hops);
    Serial.println(gumboData[listLocation].hops);
    return 0; 
  }
  return 0;
}

void waitForSync() {
  unsigned long startMillis = millis();
  unsigned long currentMillis;
  while(lastSync == 0) {
    currentMillis = millis();
    listenForPacket();
    if(currentMillis - startMillis> syncDataLossInterval) {
      // Make this system a slave.
      startMillis = millis();
      Serial.println("Still waiting for sync!");
      ///gumboSend(GUMBO_ID, true);
      //lastSync = millis();
    }
  }
}

byte getListLocation(byte id) {
  byte i;
  byte zeroLocation = 0;
  for(i=0; i<GUMBO_SIZE; i++) {
    if (id == gumboData[i].id) return i;
    if ((gumboData[i].id == 0 || gumboData[i].staleData == true) && zeroLocation == 0) { 
      zeroLocation = i;
    }
  }
  return zeroLocation;
}

void printNode(byte node) {
  Serial.print("Point ");
  Serial.print(node);
  Serial.print(": ");
  Serial.print(gumboData[node].id);
  Serial.print(" ");
  Serial.print(gumboData[node].sensorReading);
  Serial.print(" ");
  Serial.print(gumboData[node].sensorReading2);
  Serial.print(" ");
  Serial.print(gumboData[node].rssi);
  Serial.print(" ");
  Serial.print(gumboData[node].hops);
  Serial.println();
}

void printAllData() {
  int i;
  Serial.println("Data");
  for(i=0; i<GUMBO_SIZE; i++) {
    Serial.print("Point ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(gumboData[i].id);
    Serial.print(" ");
    Serial.print(gumboData[i].sensorReading);
    Serial.print(" ");
    Serial.print(gumboData[i].sensorReading2);
    Serial.print(" ");
    Serial.print(gumboData[i].rssi);
    Serial.print(" ");
    Serial.print(gumboData[i].hops);
    Serial.println();
  } 
}

void initGumboList() {
  byte i;
  gumboData[0].id = GUMBO_ID;
  gumboData[0].sensorReading = getTemp();
  for(i=1; i<GUMBO_SIZE; i++) {
    gumboData[i].id = 0;
    gumboData[i].rssi = 0;
    gumboData[i].hops = 255;
    gumboData[i].staleData = false;
  }
}

double getTemp(void) {
  unsigned int wADC;
  double t;
  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.
  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC
  delay(20);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;
  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;
  // The returned temperature is in degrees Celcius.
  return (t);
}

void averageTemperature() {
  int i, averageTemperature;
  int validNodes = 0;
  for (i = 0; i<GUMBO_SIZE; i++) {
    if(gumboData[i].id != 0 && !gumboData[i].staleData) {
      validNodes++;
      //averageTemperature += (gumboData[i].sensorReading);
      averageTemperature += (gumboData[i].sensorReading / qualityFactor(i));
    }
  }
  averageTemperature = averageTemperature/validNodes;
  gumboData[0].sensorReading2 = averageTemperature;
  Serial.print("Average Temperature: ");
  Serial.println(averageTemperature);
}

byte qualityFactor(byte gumboListLocation) {
  byte qualityFactor = 4 * gumboData[gumboListLocation].hops;
  if(gumboData[gumboListLocation].rssi >= 30) {
    qualityFactor = qualityFactor + 1;
  } else if (gumboData[gumboListLocation].rssi >= 15) {
    qualityFactor = qualityFactor + 2;
  } else if (gumboData[gumboListLocation].rssi >= 0) {
    qualityFactor = qualityFactor + 3;
  } else if (gumboData[gumboListLocation].rssi >= -15) {
    qualityFactor = qualityFactor + 4;
  } else {
    qualityFactor = qualityFactor + 5;
  }
  return 1;
}

void gumboSleep() {
  delay(sleepTime);
  sendSync = true;
}

boolean isPQTReached() {
  return ReadReg(REG_PKTSTATUS) && B00100000;
}

void WriteReg(char addr, char value){
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
    };
    
  SPI.transfer(addr);
  delay(10);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
}

char ReadReg(char addr){
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);
  delay(10);
  char y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

char SendStrobe(char strobe){
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
  };
    
  char result =  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(10);
  return result;
}

void init_CC2500(){
  WriteReg(REG_IOCFG2,0x06);
  WriteReg(REG_IOCFG0,0x01);
  //WriteReg(REG_IOCFG1,0x0E);
  WriteReg(REG_IOCFG1,0x06);

  //WriteReg(REG_FIFOTHR,VAL_FIFOTHR);
  WriteReg(REG_FIFOTHR, 0x02);
  WriteReg(REG_SYNC1,VAL_SYNC1);
  WriteReg(REG_SYNC0,VAL_SYNC0);
  WriteReg(REG_PKTLEN,VAL_PKTLEN);
  //WriteReg(REG_PKTLEN, 0x06);
  WriteReg(REG_PKTCTRL1,0x8C);
  //WriteReg(REG_PKTCTRL0,VAL_PKTCTRL0);
  WriteReg(REG_PKTCTRL0, 0x0D);
  
  WriteReg(REG_ADDR,VAL_ADDR);
  WriteReg(REG_CHANNR,VAL_CHANNR);
  WriteReg(REG_FSCTRL1,VAL_FSCTRL1);
  WriteReg(REG_FSCTRL0,VAL_FSCTRL0);
  WriteReg(REG_FREQ2,VAL_FREQ2);
  WriteReg(REG_FREQ1,VAL_FREQ1);
  WriteReg(REG_FREQ0,VAL_FREQ0);
  /*WriteReg(REG_MDMCFG4,0x8C);
  WriteReg(REG_MDMCFG3,0x32);
  WriteReg(REG_MDMCFG1,0x72); */
  WriteReg(REG_MDMCFG4,VAL_MDMCFG4);
  WriteReg(REG_MDMCFG3,VAL_MDMCFG3);
  WriteReg(REG_MDMCFG2,VAL_MDMCFG2);
  WriteReg(REG_MDMCFG1,VAL_MDMCFG1);
  WriteReg(REG_MDMCFG0,VAL_MDMCFG0);
  WriteReg(REG_DEVIATN,VAL_DEVIATN);
  WriteReg(REG_MCSM2,VAL_MCSM2);
  WriteReg(REG_MCSM1,VAL_MCSM1);
  WriteReg(REG_MCSM0,VAL_MCSM0);
  WriteReg(REG_FOCCFG,VAL_FOCCFG);

  WriteReg(REG_BSCFG,VAL_BSCFG);
  WriteReg(REG_AGCCTRL2,0x00);
  WriteReg(REG_AGCCTRL1,0x40);
  //WriteReg(REG_AGCCTRL2,VAL_AGCCTRL2);
  //WriteReg(REG_AGCCTRL1,VAL_AGCCTRL1);
  WriteReg(REG_AGCCTRL0,VAL_AGCCTRL0);
  WriteReg(REG_WOREVT1,VAL_WOREVT1);
  WriteReg(REG_WOREVT0,VAL_WOREVT0);
  WriteReg(REG_WORCTRL,0x78);
  WriteReg(REG_FREND1,VAL_FREND1);
  WriteReg(REG_FREND0,VAL_FREND0);
  WriteReg(REG_FSCAL3,VAL_FSCAL3);
  WriteReg(REG_FSCAL2,VAL_FSCAL2);
  WriteReg(REG_FSCAL1,VAL_FSCAL1);
  WriteReg(REG_FSCAL0,VAL_FSCAL0);
  WriteReg(REG_RCCTRL1,VAL_RCCTRL1);
  WriteReg(REG_RCCTRL0,VAL_RCCTRL0);
  WriteReg(REG_FSTEST,VAL_FSTEST);
  WriteReg(REG_PTEST,VAL_PTEST);
  WriteReg(REG_AGCTEST,VAL_AGCTEST);
  WriteReg(REG_TEST2,VAL_TEST2);
  WriteReg(REG_TEST1,VAL_TEST1);
  WriteReg(REG_TEST0,VAL_TEST0);
/*  
  WriteReg(REG_PARTNUM,VAL_PARTNUM);
  WriteReg(REG_VERSION,VAL_VERSION);
  WriteReg(REG_FREQEST,VAL_FREQEST);
  WriteReg(REG_LQI,VAL_LQI);
  WriteReg(REG_RSSI,VAL_RSSI);
  WriteReg(REG_MARCSTATE,VAL_MARCSTATE);
  WriteReg(REG_WORTIME1,VAL_WORTIME1);
  WriteReg(REG_WORTIME0,VAL_WORTIME0);
  WriteReg(REG_PKTSTATUS,VAL_PKTSTATUS);
  WriteReg(REG_VCO_VC_DAC,VAL_VCO_VC_DAC);
  WriteReg(REG_TXBYTES,VAL_TXBYTES);
  WriteReg(REG_RXBYTES,VAL_RXBYTES);
  WriteReg(REG_RCCTRL1_STATUS,VAL_RCCTRL1_STATUS);
  WriteReg(REG_RCCTRL0_STATUS,VAL_RCCTRL0_STATUS);
  */
}

void Read_Config_Regs(void){ 
  Serial.println(ReadReg(REG_IOCFG2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
   delay(10);
/* Serial.println(ReadReg(REG_FIFOTHR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_SYNC1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_SYNC0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTLEN),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_ADDR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_CHANNR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG4),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG3),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_DEVIATN),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FOCCFG),HEX);
   delay(10);

  Serial.println(ReadReg(REG_BSCFG),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WOREVT1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WOREVT0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WORCTRL),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREND1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREND0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL3),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_RCCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_RCCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST0),HEX);
   delay(10);
 /*
  Serial.println(ReadReg(REG_PARTNUM),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_VERSION),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_FREQEST),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_LQI),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RSSI),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_MARCSTATE),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_WORTIME1),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_WORTIME0),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_PKTSTATUS),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_VCO_VC_DAC),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_TXBYTES),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RXBYTES),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RCCTRL1_STATUS),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RCCTRL0_STATUS),HEX);
   delay(1000);
*/  
}
  

