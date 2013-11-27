/**
 */
#include "cc2500_REG.h"
#include "cc2500_VAL.h"
#include "SPI85.h"

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_SWOR    0x38
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F
#define GUMBO_ID 25
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
} GumboNode;

GumboNode gumboData[GUMBO_SIZE];
long sendInterval = 5000; // in milliseconds
long sleepTime = 1000;
long wakeTime = 1000;
long syncDataLossInterval = 3000;
long previousMillis = 0;        // will store last time data was sent
long previousSyncDataLossMillis = 0;        // will store last time data was sent
long previousTXTimeoutMillis = 0;        // will store last time data wa
byte cyclesSinceLastData, GDO0_State;

void setup(){
  // Setup 
  pinMode(SS,OUTPUT);
  SPI85.begin();
  SPI85.setDataMode(SPI_MODE0);
  SPI85.setClockDivider(SPI_2XCLOCK_MASK);
  digitalWrite(SS,HIGH);
  init_CC2500();
}

void loop(){
  gumboSend(GUMBO_ID, true);
  listenForPacket();
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > sendInterval) {
    previousMillis = currentMillis;   
    gumboSend(GUMBO_ID, false);
  }

  listenForPacket();
  gumboSleep();
}

void listenForPacket() {
  SendStrobe(CC2500_RX);
  unsigned long currentMillis = millis();
  if (digitalRead(MISO)) {
    char PacketLength = ReadReg(CC2500_RXFIFO);
    char recvPacket[PacketLength];
    if(PacketLength >= 6) {
      for(int i = 1; i < PacketLength; i++){
        recvPacket[i] = ReadReg(CC2500_RXFIFO);
      }
      if(recvPacket[1] == 'd') {
        
      } else if (recvPacket[1] == 'w') {
        
      } else {
        
      }
    }
    
    // Make sure that the radio is in IDLE state before flushing the FIFO
    // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
    SendStrobe(CC2500_IDLE);
    // Flush RX FIFO
    SendStrobe(CC2500_FRX);
  } else {
    if(currentMillis - previousSyncDataLossMillis > syncDataLossInterval) {
      previousSyncDataLossMillis = currentMillis;
    }
  }
}

void gumboSend(byte gumboDataID, boolean sync) {
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
    packet[7] = gumboData[gumboDataID].hops;
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

void checkQuality() {
  
}

void gumboSleep() {
 delay(sleepTime); 
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
  WriteReg(REG_PKTCTRL1,VAL_PKTCTRL1);
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
  
