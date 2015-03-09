#include <XBee.h>
#define sampleSize 50

XBee xbee = XBee();
int rssiArray[sampleSize];
int vectorArray[sampleSize];
int rssiDif = 0;
float rssiCompare = 0;
int vector = 0;


Rx16Response rx16 = Rx16Response();
void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
  xbee.setSerial(Serial1);
}

void loop(){
  int i = 0;
  while (i < sampleSize){
    xbee.readPacket(100);
    if (xbee.getResponse().isAvailable()){
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE){
        xbee.getResponse().getRx16Response(rx16);
        rssiArray[i] = rx16.getRssi();
        vectorArray[i] = rx16.getData(0);      
        i++;
      }
    }
  }
  int maxRssi = 0;
  int k = 0;
  while (k < sampleSize){
    if (rssiArray[k] > maxRssi){
     maxRssi = rssiArray[k];
    }
    k++;
  }
  int minRssi = 100;
  int j = 0;
  while (j < sampleSize){
    if (rssiArray[j] < minRssi){
      minRssi = rssiArray[j];
    }
    j++;
  }
  rssiDif = (maxRssi - minRssi);
  rssiCompare = (minRssi + (rssiDif*.1));
  int h = 0;
  int count = 0;
  while (h < sampleSize){
    if (rssiArray[h] <= rssiCompare && vectorArray[h] < 180){
      count++;
    }
    h++;
  }
  int topVector[count];
  int m = 0;
  int n = 0;
  while (m < sampleSize){
    if (rssiArray[m] <= rssiCompare && vectorArray[m] < 180){
      topVector[n] = vectorArray[m];
      n++;
    }
    m++;
  }
  int vectorSum = 0;
  int p = 0;
  while (p < count){
    vectorSum = vectorSum + topVector[p];
    p++;
  }
  vector = (vectorSum/count);
  Serial.print("\n");
  Serial.print("Vector: ");
  Serial.print(vector); 
}
