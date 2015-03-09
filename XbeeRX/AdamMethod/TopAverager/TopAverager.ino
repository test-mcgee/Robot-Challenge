#include <XBee.h>
XBee xbee = XBee();
const int Samples = 200;
int RSSIArray[Samples], HeadingArray[Samples];
Rx16Response rx16 = Rx16Response();

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  Serial1.begin(9600);
  xbee.setSerial(Serial1);
}

void loop() {
  //Retrieve Samples packets and store the Heading/RSSI Pairs in seperate arrays
  Retrieve();
  Serial.println("Current Heading: ");
  Serial.println(ProcessData(RSSIArray, HeadingArray));
}



/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

void Retrieve(){
    for (int i = 0; i< Samples; i ++)
  {
    xbee.readPacket(50);
    if (xbee.getResponse().isAvailable())
    {
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) xbee.getResponse().getRx16Response(rx16);
      RSSIArray[i] = abs(rx16.getRssi()-100);
      HeadingArray[i] = rx16.getData(0);
    }
  }  
}



int ProcessData(int * RSSIArray, int * HeadingArray){
  int RSSIMax = 0, RSSIMin = 100;
  int Average = 0, counter = 0;
  
  for(int i=1; i<Samples; i++){
    if(RSSIArray[RSSIMax] < RSSIArray[i]) RSSIMax = RSSIArray[i];
    else if(RSSIArray[RSSIMin] > RSSIArray[i]) RSSIMin = RSSIArray[i];
  }
  
  int Threshold = RSSIMax - (RSSIMax-RSSIMin)*0.1;
  
  for(int i=1; i<Samples; i++){
    if(RSSIArray[i] > Threshold){
      Average += HeadingArray[i];
      counter++;
    }
  }
  return Average/counter;   
}
  
  
  
  
  
  
  
  
  
  

