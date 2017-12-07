/*
 * polling 3
 * lo slave risponde al master se interrogato da questo
 * oppure risponde al nodo intermedio se la richiesta è passata da questo
 */

#include <RFM69.h>
#include <EEPROM.h>
#include <Cmd.h>
#include "Antirimbalzo.h"
#include "TxPkt.h"
#include "ControlloUscita.h"
#include <Cmd.h>
#include <RFM69registers.h>


#define pinPULSANTE 3
#define PINBATTERIA 0 // per lettura tensione batteria 
// parametri radio
#define NETWORKID 27
#define FREQUENCY 868000000
#define RFM69_CS 10
#define RFM69_IRQ 2
#define RFM69_IRQN 0 
#define RFM69_RST 9

//#define MASTER 0

#define LEDPIN 4

Nodo::Nodo() { indirizzo=0; segnale=-127;}



unsigned long Tvoto;
byte indirizzo;
RFM69 radio=RFM69(RFM69_CS, RFM69_IRQ, true, RFM69_IRQN);

Nodo bestn[6];
Antirimbalzo swVoto;
bool votato=false;
bool modoVoto=false;
byte modifichealistabest=0;
bool StampaInfoRouting=false;
bool stampaInfo=false;
bool nodoRipetitore; // true se questo è un nodo ripetitore
int numeropoll; // contatore numero di poll (serve per decidere cosa trasmettere in modo non voto)
ControlloUscita led(LEDPIN,false,true);



void serialCmdStampaPacchettiRadio(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  if(args[1][0]=='0') radio._printpackets=false;
  if(args[1][0]=='1') radio._printpackets=true;
}

void serialCmdStampaInfoRouting(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  if(args[1][0]=='0') StampaInfoRouting=false;
  if(args[1][0]=='1') StampaInfoRouting=true;
}

void serialCmdStampaInfoStato(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  if(args[1][0]=='0') stampaInfo=false;
  if(args[1][0]=='1') stampaInfo=true;
}

void serialCmdImpostaNodoRipetitore(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  if(args[1][0]=='0') nodoRipetitore=false;
  if(args[1][0]=='1') nodoRipetitore=true;
  Serial.print(F("Nodo ripetitore:"));  
  Serial.println(nodoRipetitore);  
  EEPROM.write(1,nodoRipetitore);
}

void serialCmdMemorizzaIndirizzo(int arg_cnt, char **args)
{
  if(arg_cnt!=2) return;
  int ind=atoi(args[1]);
  if(ind<2 || ind>255) {
    Serial.println(F("parametro errato"));  
  } else {
    EEPROM.write(0,ind);
    radio.setAddress(ind);
    Serial.print(F("indirizzo memorizzato: "));
    Serial.println(ind);
  }
}
void serialCmdPrintRadioReg(int arg_cnt, char **args)
{
    radio.readAllRegs();
}
void serialCmdWriteRadioReg(int arg_cnt, char **args)
{
    if(arg_cnt!=3) return;
    byte reg=atoi((const char *)args[1]);
    byte val=atoi((const char *)args[2]);
    if(reg<0) return;
    if(reg>0x71) return;
    radio.writeReg(reg,val);
    Serial.print(reg,HEX);
    Serial.print(" ");
    Serial.println(val,HEX);
}

void setup() {
  // pin pulsante col pullup
  pinMode(pinPULSANTE, INPUT_PULLUP);
  // legge indirizzo slave dal byte 0 della eeprom
  indirizzo = EEPROM.read(0);
  nodoRipetitore= (EEPROM.read(1)==1);
  // info su seriale
  Serial.begin(250000);
  Serial.print(F("Slave 4.6 Indirizzo: "));
  Serial.print(indirizzo);
  Serial.print(F(" nodo ripetitore: "));
  Serial.println(nodoRipetitore);
  // imposta radio
  radioSetup();
  // stampa frequenza
  Serial.print(F("Frequenza: "));
  Serial.println(radio.getFrequency());
  //radio.readAllRegs();

  radio._printpackets=false;

  swVoto.cbInizioStatoOn=ElaboraPulsante;

  cmdInit(&Serial);
  cmdAdd("ip", serialCmdStampaPacchettiRadio);
  cmdAdd("ir", serialCmdStampaInfoRouting);
  cmdAdd("is", serialCmdStampaInfoStato);
  cmdAdd("w", serialCmdMemorizzaIndirizzo);
  cmdAdd("nr", serialCmdImpostaNodoRipetitore);
  cmdAdd("radioreg", serialCmdPrintRadioReg);
  cmdAdd("writeradioreg", serialCmdWriteRadioReg);
  led.OndaQuadra(20,2800);
  votato=false;
  modoVoto=false;
}

// algoritmo 1
void loop() {
  if(!nodoRipetitore) swVoto.Elabora(digitalRead(pinPULSANTE)==LOW);
  ElaboraRadio();
  led.Elabora();
  cmdPoll();
}

// algoritmo 2
void ElaboraRadio() {
  if(!radio.receiveDone()) return;
  if (radio._printpackets) stampapkt(radio.SENDERID,radio.TARGETID,radio.DATA,radio.DATALEN);
  CostruisciListaNodi(radio.SENDERID, radio.RSSI);
  if(radio.TARGETID!=indirizzo) return;
  byte destinatario=radio.DATA[1];
  delay(5);     
  if(destinatario==indirizzo) {
    if(!votato) led.OndaQuadra(20,2800);
    numeropoll++;
    if(!nodoRipetitore)
    {
      switch(radio.DATA[2]) {
        case 0xaa: // modo voto
            if(!modoVoto)
            {
              modoVoto=true;
              InizioVoto();
            }
            RispondiPollModoVoto(radio.SENDERID);
            break;
        case 0x55: // modo normale
            if(modoVoto)
            {
              modoVoto=false;
              FineVoto();
            }
            RispondiPollModoNonVoto(radio.SENDERID);
            break;
      }
    }
    else
    {
      RispondiPollModoRipetitore(radio.SENDERID);
    }
  } else {
      if(radio._printpackets) Serial.println(F("pkt da ritrasmettere"));
      radio.send(destinatario, (const byte *)radio.DATA, radio.DATALEN,false);
    }
  }
void RispondiPollModoRipetitore(byte rip) 
{
  TxPkt p(indirizzo,1);
  p.RispModoRipetitore();
  radio.send(rip, (const byte *)&p.dati, p.len,false);
  
}
void RispondiPollModoNonVoto(byte rip)
{
  TxPkt p(indirizzo,1);
  if(modifichealistabest>0 && (numeropoll & 1 == 1))
  {
    modifichealistabest--;
    p.ListaBest(bestn);
    if(stampaInfo) Serial.println("tx listabest");
  }
  else
  {
    unsigned long int t=micros();
    p.Sync(t,analogRead(PINBATTERIA)>>2);
    if(stampaInfo)
    {
      Serial.print("tx micros e batt\t");
      Serial.println(t);
    } 
  }
  radio.send(rip, (const byte *)&p.dati, p.len,false);
}

void RispondiPollModoVoto(byte rip)
{
  TxPkt p(indirizzo,1);
  if(votato)
  {
    p.SetOraVoto(Tvoto);
    if(stampaInfo)
    {
      Serial.print("tx oravoto\t");
      Serial.println(Tvoto);
    } 
    
  }
  else
  {
    p.NonVotato();
    if(stampaInfo) Serial.println("tx non votato");

  }
  radio.send(rip, (const byte *)&p.dati, p.len,false);
}

void InizioVoto() 
{
  votato=false;
  if(stampaInfo) Serial.println("iniziovoto");
  led.OndaQuadra(100,200);

}
void FineVoto() 
{
  led.OndaQuadra(20,2800);
  if(stampaInfo) Serial.println("finevoto");
  votato=false;
}
// algoritmo 3
void ElaboraPulsante() {
  if(modoVoto) 
  {
    votato=true;
    Tvoto=micros();
    led.OndaQuadra(1,1);
    if(stampaInfo)
    {
      Serial.print(F("VOTATO: tvoto="));      
      Serial.println(Tvoto,DEC);
    } 
  }
}





void CostruisciListaNodi(byte ind, int sign) {
    // se il nodo ricevuto è più forte del più debole lo sostituisco con questo
    if(ind==0) return;
    //if(dest!=0) return;
    //if(len<2) return;
    // controlla se il nuovo nodo è già in lista
    bool giainlista=false;
    for (int i=0;i<5;i++) if(bestn[i].indirizzo==ind) {bestn[i].segnale=sign; bestn[i].tultimopkt=millis(); giainlista=true; break;};
    for (int i=0;i<5;i++) if((millis() - bestn[i].tultimopkt) > 10000) {bestn[i].segnale=-127; bestn[i].indirizzo=0;};
    // se non c'è calcola trova l'indirizzo del più scarso
    if(!giainlista) 
    {
      bestn[5].segnale=sign;
      bestn[5].indirizzo=ind;
      bestn[5].tultimopkt=millis();
    }
    // poi li ordina
    Nodo tmp;
    for (int i=0;i<5;i++) 
      for (int k=i+1;k<6;k++) 
        if(bestn[k].segnale>bestn[i].segnale) 
        {
          tmp.indirizzo=bestn[k].indirizzo; 
          tmp.segnale=bestn[k].segnale; 
          tmp.tultimopkt=bestn[k].tultimopkt;
          bestn[k].segnale=bestn[i].segnale; 
          bestn[k].indirizzo=bestn[i].indirizzo;
          bestn[k].tultimopkt=bestn[i].tultimopkt;
          bestn[i].indirizzo=tmp.indirizzo;
          bestn[i].segnale=tmp.segnale; 
          bestn[i].tultimopkt=tmp.tultimopkt; 
          modifichealistabest=3;
        };
    
    if(StampaInfoRouting) {
      Serial.print(F("best: i/s "));
      Serial.print(ind);
      Serial.print("\t");
      Serial.print(sign);
      Serial.print("\t");
      for (int i=0;i<5;i++) 
      {
        Serial.print(bestn[i].indirizzo,DEC);
        Serial.print("\t");
        Serial.print(bestn[i].segnale,DEC);
        Serial.print("\t");
      }
      Serial.println();
    }
}

void stampapkt(byte SENDERID,byte TARGETID, byte *DATA,int DATALEN) {
Serial.print(F("rxFrame: time/sender/target/dati: "));
	  Serial.print(micros());
	  Serial.print("/");
	  Serial.print(SENDERID);
	  Serial.print("/");
	  Serial.print(TARGETID);
	  Serial.print("/D:");
	  for (uint8_t i = 0; i < DATALEN; i++){
	      Serial.print(DATA[i],HEX);
		  Serial.print("/");
  	
	  }
	  Serial.println("");    
}

void radioSetup() {
  // Hard Reset the RFM module 
  Serial.println("radioSetup");
  pinMode(RFM69_RST, OUTPUT); 
  digitalWrite(RFM69_RST, HIGH); 
  delay(100);
  digitalWrite(RFM69_RST, LOW); 
  delay(100);
	radio.initialize(RF69_868MHZ,indirizzo,NETWORKID);
  radio.writeReg(REG_BITRATEMSB,RF_BITRATEMSB_50000);
  radio.writeReg(REG_BITRATELSB,RF_BITRATELSB_50000);
  radio.writeReg(REG_FDEVMSB,RF_FDEVMSB_50000);
  radio.writeReg(REG_FDEVLSB,RF_FDEVLSB_50000);
  radio.writeReg(REG_PACKETCONFIG1,RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF); // data whitening e no address filter
  radio.setFrequency(FREQUENCY);
	radio.setHighPower(); 
  radio.setPowerLevel(31);
  radio.promiscuous(true);
}