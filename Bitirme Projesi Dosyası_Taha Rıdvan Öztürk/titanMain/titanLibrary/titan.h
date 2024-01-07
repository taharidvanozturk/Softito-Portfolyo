#ifndef titan_H_
#define titan_H_
#include "Arduino.h"
#define fan D8
#define led D6
#define rol D5
#define buton_pin D0
#define DHTPIN D3      
#define DHTTYPE DHT11
#define sensor_pin A0

class titan{
    public:
    void init();
    int fanControl(int t);
    int butonOkuma();
    int suKontrol();
    

};

#endif 