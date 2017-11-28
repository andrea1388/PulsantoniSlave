#include <Arduino.h>
#include "Nodo.h"

class TxPkt {
    public:
        TxPkt(byte mittente, byte destinatario);
        void ListaBest(Nodo *best);
        void Sync(unsigned long int micros,byte livbatt);
        void NonVotato();
        void SetOraVoto(unsigned long int);
        byte dati[20];
        byte len;

    protected:
        byte destinatario;
        byte mittente;

};