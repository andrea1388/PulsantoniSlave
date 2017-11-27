#include <Arduino.h>
#include "Nodo.h"

class TxPkt {
    public:
        TxPkt(byte destinatario);
        void ListaBest(Nodo *best);
        void Sync(unsigned long int micros,byte livbatt);
        void NonVotato();
        void SetOraVoto(unsigned long int);
        byte *dati;
        byte len;

    protected:
        byte destinatario;

};