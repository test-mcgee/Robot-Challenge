
#include <XBee.h>

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();
int RSSIArray[200];
int DataArray[200];


void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  Serial1.begin(9600);
  xbee.setSerial(Serial1);
}

void loop() {
  for (int time = 0; time< 200; time ++)
  {
    xbee.readPacket(50);
    if (xbee.getResponse().isAvailable())
    {
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) xbee.getResponse().getRx16Response(rx16);
    }
    Serial.println(rx16.getData(0));
    Serial.println(abs(rx16.getRssi()-100));
  }
}


      




  
  
  
  
  
  
  
  
  
  

