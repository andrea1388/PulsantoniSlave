/*
 * polling 3
 * lo slave risponde al master se interrogato da questo
 * oppure risponde al nodo intermedio se la richiesta è passata da questo
 */
#define MAXBESTNEIGHBOURS 6
#include <RFM69.h>
#include <EEPROM.h>
#include <Cmd.h>
#include "Antirimbalzo.h"
#include "TxPkt.h"
#include "ControlloUscita.h"
#include <Cmd.h>


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

Nodo bestn[MAXBESTNEIGHBOURS];
Antirimbalzo swVoto;
bool votato=false;
bool modoVoto=false;
byte modifichealistabest=0;
bool StampaInfoRouting=false;
bool stampaInfo=false;
bool nodoRipetitore; // true se questo è un nodo ripetitore
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

void setup() {
  // pin pulsante col pullup
  pinMode(pinPULSANTE, INPUT_PULLUP);
  // legge indirizzo slave dal byte 0 della eeprom
  indirizzo = EEPROM.read(0);
  nodoRipetitore= (EEPROM.read(1)==1);
  // info su seriale
  Serial.begin(9600);
  Serial.print(F("Slave 4.1 Indirizzo: "));
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
  FineVoto();
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
  CostruisciListaNodi(radio.SENDERID, radio.RSSI);
  if(radio.TARGETID!=indirizzo) return;
  byte destinatario=radio.DATA[1];
  delay(5);     
  if(destinatario==indirizzo) {
    led.OndaQuadra(200,2800);
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
  if(modifichealistabest>0)
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

}
void FineVoto() 
{
  led.OndaQuadra(20,2800);
  if(stampaInfo) Serial.println("finevoto");
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


void radioSetup() {
  // Hard Reset the RFM module 
  Serial.println("radioSetup");
  pinMode(RFM69_RST, OUTPUT); 
  digitalWrite(RFM69_RST, HIGH); 
  delay(100);
  digitalWrite(RFM69_RST, LOW); 
  delay(100);
	radio.initialize(RF69_868MHZ,indirizzo,NETWORKID);
 /*
  radio.writeReg(0x03,0x0D); // 9k6
  radio.writeReg(0x04,0x05);
  */
  radio.writeReg(0x03,0x00); // 153k6
  radio.writeReg(0x04,0xD0);
  radio.writeReg(0x37,radio.readReg(0x37) | 0b01010000); // data whitening e no address filter
  radio.setFrequency(FREQUENCY);
	radio.setHighPower(); 
  radio.setPowerLevel(31);
  radio.promiscuous(true);
}


void CostruisciListaNodi(byte ind, int sign) {
    // se il nodo ricevuto è più forte del più debole lo sostituisco con questo
    if(ind==0) return;
    //if(dest!=0) return;
    //if(len<2) return;
    // controlla se il nuovo nodo è già in lista
    bool giainlista=false;
    for (int i=0;i<MAXBESTNEIGHBOURS;i++) if(bestn[i].indirizzo==ind) {bestn[i].segnale=sign; giainlista=true;};
    // se non c'è calcola trova l'indirizzo del più scarso
    if(!giainlista) 
    {
      bestn[MAXBESTNEIGHBOURS-1].segnale=sign;
      bestn[MAXBESTNEIGHBOURS-1].indirizzo=ind;
    }
    // poi li ordina
    Nodo tmp;
    for (int i=0;i<MAXBESTNEIGHBOURS-1;i++) 
      for (int k=i+1;k<MAXBESTNEIGHBOURS;k++) 
        if(bestn[k].segnale>bestn[i].segnale) 
        {
          tmp.indirizzo=bestn[k].indirizzo; 
          tmp.segnale=bestn[k].segnale; 
          bestn[k].segnale=bestn[i].segnale; 
          bestn[k].indirizzo=bestn[i].indirizzo;
          bestn[i].indirizzo=tmp.indirizzo;
          bestn[i].segnale=tmp.segnale; 
          modifichealistabest=3;
        };
    
    if(StampaInfoRouting) {
      Serial.print(F("best: i/s "));
      Serial.print(ind);
      Serial.print("\t");
      Serial.print(sign);
      Serial.print("\t");
      for (int i=0;i<MAXBESTNEIGHBOURS-1;i++) 
      {
        Serial.print(bestn[i].indirizzo),DEC;
        Serial.print("\t");
        Serial.print(bestn[i].segnale,DEC);
        Serial.print("\t");
      }
      Serial.println();
    }
}
