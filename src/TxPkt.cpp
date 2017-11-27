#include <Arduino.h>
#include "TxPkt.h"


TxPkt::TxPkt(byte mitt, byte dest) {
    destinatario=dest;
    mittente=mitt;
}

void TxPkt::ListaBest(Nodo* best)
{
    len=13;
    dati=new byte[len];
    dati[0]=mittente;
    dati[1]=destinatario;
    dati[2]=2;
    for(int j=0;j<5;j++)
    {
        dati[j+3]=best[j].indirizzo;
        dati[2*j+4]=best[j].segnale;
    }
    
}
        
void TxPkt::Sync(unsigned long int micros,byte livbatt)
{
    len=8;
    dati=new byte[len];
    dati[0]=mittente;
    dati[1]=destinatario;
    dati[2]=1;
    dati[3]=micros >> 24;
    dati[4]=micros >> 16;
    dati[5]=micros >> 8;
    dati[6]=micros;
    dati[7]=livbatt;    
}
void TxPkt::NonVotato()
{
    len=3;
    dati=new byte[len];
    dati[0]=mittente;
    dati[1]=destinatario;
    dati[2]=4;
}
void TxPkt::SetOraVoto(unsigned long int micros)
{
    len=7;
    dati=new byte[len];
    dati[0]=mittente;
    dati[1]=destinatario;
    dati[2]=3;
    dati[3]=micros >> 24;
    dati[4]=micros >> 16;
    dati[5]=micros >> 8;
    dati[6]=micros;
    
}