/**
 */
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "cc2500_REG.h"
#include "cc2500_VAL.h"
#include "SPI85.h"
#include "InternalTemperatureSensor.h"

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_SPWD    0x39
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F
#define GUMBO_ID 2
#define GUMBO_SIZE 25

#define TX_TIMEOUT 50 // in milliseconds

const static uint8_t MISO = PB0;
const static uint8_t MOSI = PB1;
const static uint8_t SCK  = PB2;
const static uint8_t SS   = PB4;

typedef struct {
  byte id;
  byte sensorReading;
  byte sensorReading2;
  byte rssi;
  byte hops;
  boolean staleData;
} GumboNode;

GumboNode gumboData[GUMBO_SIZE];
InternalTemperatureSensor temperature(1.0, TEMPERATURE_ADJUSTMENT);
long wakeLength = 1000;
long syncDataLossInterval = 5*wakeLength; // 5 * WatchDog Sleep Timer
long staleDataTime = 20*wakeLength;
long lastSync = 0;   
long previousTXTimeoutMillis = 0;  
long previousWakeMillis = 0;       
byte cyclesSinceLastData, GDO0_State, gumboDataIndex;
boolean sendSync;
boolean f_wdt = true;

void setup(){
  // Setup 
  pinMode(SS,OUTPUT);
  SPI85.begin();
  SPI85.setDataMode(SPI_MODE0);
  SPI85.setClockDivider(SPI_2XCLOCK_MASK);
  digitalWrite(SS,HIGH);
  sendSync = false;
  gumboDataIndex = 1;
  setup_watchdog(WDTO_1S); // approximately 1 seconds sleep
  init_CC2500();
  temperature.init();
  initGumboList();
  waitForSync();
}

void loop(){
  if(f_wdt) {
    sendSync = true;
    f_wdt = false;
  }
  if(sendSync) {
     gumboSendSync();
     sendSync = false;
     // Immediately listen and then send out own data
     listenForPacket();
     gumboSendData(0);
  }
  // Get the time now that sync has been sent.
  unsigned long currentMillis = millis();
  
  // Send out other GumboNodes data
  if(gumboData[gumboDataIndex].id > 0) {
    gumboSendData(gumboDataIndex);
    gumboDataIndex++;
  } else {
    gumboDataIndex = 1;
  }
  
  listenForPacket();
  if(currentMillis - lastSync > syncDataLossInterval) {
      // SYNC LOST
      lastSync = 0;
      waitForSync();
  }
  if(currentMillis - previousWakeMillis > wakeLength) {
    previousWakeMillis = currentMillis;
    gumboSleep();
  }
}

void listenForPacket() {
  SendStrobe(CC2500_RX);
  unsigned long currentMillis = millis();
  if (digitalRead(MISO)) {
    char PacketLength = ReadReg(CC2500_RXFIFO);
    char recvPacket[PacketLength];
    if(PacketLength == 8) {
      for(int i = 1; i < PacketLength; i++){
        recvPacket[i] = ReadReg(CC2500_RXFIFO);
      }
      byte rssi = ReadReg(CC2500_RXFIFO);
      byte lqi = ReadReg(CC2500_RXFIFO);
      if(recvPacket[1] == 'd') {
        // Data packet received
        addIfHigherQuality(recvPacket[3], recvPacket[4], recvPacket[5], lqi, recvPacket[7]);
      } else if (recvPacket[1] == 'w') {
        // Wake packet received.
        lastSync = millis();
      } else {
        // Bunk
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
    return 1;
  } else if (hops <= gumboData[listLocation].hops){
    gumboData[listLocation].id = id;
    gumboData[listLocation].sensorReading = sensorReading;
    gumboData[listLocation].sensorReading2 = sensorReading2;
    gumboData[listLocation].rssi = lqi;
    gumboData[listLocation].hops = hops;
    return 1;
  } else {
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
      gumboSendSync();
      lastSync = millis();
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

void initGumboList() {
  byte i;
  gumboData[0].id = GUMBO_ID;
  gumboData[0].sensorReading = getTemp();
  for(i=1; i<GUMBO_SIZE; i++) {
    gumboData[i].id = 0;
    gumboData[i].rssi = 0;
    gumboData[i].hops = 255;
  }
}

int getTemp() {
  return temperature.in_c();
}

void averageTemperature() {
  int i, averageTemperature;
  int validNodes = 0;
  for (i = 0; i<GUMBO_SIZE; i++) {
    if(gumboData[i].id != 0) {
      validNodes++;
      averageTemperature += gumboData[i].sensorReading;
    }
  }
  averageTemperature = averageTemperature/validNodes;
  gumboData[0].sensorReading2 = averageTemperature;
}

void gumboSleep() {
  SendStrobe(CC2500_SPWD);
  sleep_enable();
  sei();                         //ensure interrupts enabled so we can wake up again
  sleep_cpu();                   //go to sleep
  sleep_disable();
  sei();                         //enable interrupts again (but INT0 is disabled from above)
}

// Here be dragons...

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt=true;  // set global flag
}

void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

void WriteReg(char addr, char value){
  digitalWrite(SS,LOW);
  SPI85.transfer(addr);
  delay(10);
  SPI85.transfer(value);
  digitalWrite(SS,HIGH);
}

char ReadReg(char addr){
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  char x = SPI85.transfer(addr);
  delay(10);
  char y = SPI85.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

char SendStrobe(char strobe){
  digitalWrite(SS,LOW);
  char result =  SPI85.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(10);
  return result;
}

void init_CC2500(){
  WriteReg(REG_IOCFG2,0x06);
  WriteReg(REG_IOCFG1,0x06);
  WriteReg(REG_IOCFG0,0x01);
  WriteReg(REG_FIFOTHR, 0x02);
  WriteReg(REG_SYNC1,VAL_SYNC1);
  WriteReg(REG_SYNC0,VAL_SYNC0);
  WriteReg(REG_PKTLEN,VAL_PKTLEN);
  WriteReg(REG_PKTCTRL1,0x8C);
  WriteReg(REG_PKTCTRL0, 0x0D);
  
  WriteReg(REG_ADDR,VAL_ADDR);
  WriteReg(REG_CHANNR,VAL_CHANNR);
  WriteReg(REG_FSCTRL1,VAL_FSCTRL1);
  WriteReg(REG_FSCTRL0,VAL_FSCTRL0);
  WriteReg(REG_FREQ2,VAL_FREQ2);
  WriteReg(REG_FREQ1,VAL_FREQ1);
  WriteReg(REG_FREQ0,VAL_FREQ0);
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
  WriteReg(REG_AGCCTRL0,VAL_AGCCTRL0);
  WriteReg(REG_WOREVT1,VAL_WOREVT1);
  WriteReg(REG_WOREVT0,VAL_WOREVT0);
  WriteReg(REG_WORCTRL,VAL_WORCTRL);
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
}
  
