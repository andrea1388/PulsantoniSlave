#include <Arduino.h>
#include "TxPkt.h"


TxPkt::TxPkt(byte dest) {
    destinatario=dest;

}

void TxPkt::ListaBest(Nodo* best)
{
    len=12;
    dati=new byte[len];
    dati[0]=destinatario;
    dati[1]=2;
    for(int j=0;j<5;j++)
    {
        dati[j+2]=best[j].indirizzo;
        dati[2*j+3]=best[j].segnale;
    }
    
}
        
void TxPkt::Sync(unsigned long int micros,byte livbatt)
{
    len=7;
    dati=new byte[len];
    dati[0]=destinatario;
    dati[1]=1;
    dati[2]=micros >> 24;
    dati[3]=micros >> 16;
    dati[4]=micros >> 8;
    dati[5]=micros;
    dati[6]=livbatt;    
}
void TxPkt::NonVotato()
{
    len=2;
    dati=new byte[len];
    dati[0]=destinatario;
    dati[1]=4;
}
void TxPkt::SetOraVoto(unsigned long int micros)
{
    len=6;
    dati=new byte[len];
    dati[0]=destinatario;
    dati[1]=3;
    dati[2]=micros >> 24;
    dati[3]=micros >> 16;
    dati[4]=micros >> 8;
    dati[5]=micros;
    
}