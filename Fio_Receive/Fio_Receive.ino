/*
ReceiveBeaconwithLCD
 *LCD is optional*
 
 Purpose:  
 Receive beacons from a Base Station or BeaconSimulator and provides the Vector to the Base Station to the robot over SPI or Serial. 
 This is the code to be loaded to the Beacon Receiver module on your robot.  Your Robot will then interface with this Arduino and obtain 
 the heading/vector information either over Serial or SPI.
 
 Arduino Pin Usage:
 
 **Digital**
 -Serial-
 Pin 1  UART RX (Radio to Arduino) will monitor the Radio Communications
 Pin 2  UART TX (Arduino to Robot) will send vector heading to robot
 
 -SPI-
 Pin 10 SS_PIN   
 Pin 11 MOSI_PIN 
 Pin 12 MISO_PIN 
 Pin 13 SCK_PIN   
 
 -Radio to Arduino-
 RSSI   Analog Pin 0
 PDN    Digital Pin 8
 TX-RX  Digital Pin 9
 Data   RX - Digital Pin 0
 VIN    5V
 GND    GND
 
 -Settings-
 Voltage Reference for divider - Using GND and Digital Pin 7 and feeding divided voltage to AREF.  DPin 7 is set to logic High 
 LED      Digital Pin 6 (GoodPacketPin)
 BeaSim   Digital Pin 4 - Beacon Sim pin to allow checkout of SPI Interface
 
 -Status-
 LED      Digital Pin 6  Output  - to a 1k resister then and LED to Ground
 
 -Optional LCD display- 
 A soft "serial" port is added to provide a display functionality to the using Digital pins 2 and 3.
 
 -How it works- 
 
 -SPI and UART interfaces-
 The Beacon Receiver is a standard slave SPI interface.  Here are the Op codes
 200 - Ambigous Vector robot should wait or move until a valid vector is available 
 201 - SPI read has pulled last valid vector.  Poll again in a second and see if a valid vector is now available.  This will update
 0-179 - Valid vectors
 
 The Robot will listen on the TX line
 200 - Ambigous Vector robot should wait or move until a valid vector is available 
 0-179 - Valid vectors
 
 History:
 Initial Date: Sept 27 2010
 Author: Brian Sanders
 Project: Robotics Challenge
 
 10/27/2010  Versions: 1.0 Initial 
 02/08/2011  1.1 Cleaned up code  
 02/13/2011  1.2 Added SPI capabilities 
 02/15/2011  1.3 Added SPI multi-read support and 201 and 200 code definitions
 01/25/2012  1.4 Updated for Arudino 1.0 and to support a constant beacon simulator. Cleaned up Code.
 02/10/2012  1.5 Cleaned up code and fixed bugs pertaining to many invalid vector reports
 02/15/2012  1.6 Added LED status like feature for static vector receipt
 02/18/2014  1.7 Added comments and description for LCD support over Serial interface.
 ------------------------------------------------------------------------------------------------*/

#define SCK_PIN   13
#define MISO_PIN  12
#define MOSI_PIN  11
#define SS_PIN    10

#include <SoftwareSerial.h>
#include <XBee.h>
SoftwareSerial mySerial(2, 3); // RX, TX


//Pin Definitions
const int RSSIInPin = 0;        // Analog input pin that the RSSI is attached to
const int GoodPacketLEDPin = 6; // Flashing LED on this pin indicates a good direction
const int PDNPin = 8;           // Power on the transceiver 
const int TXRXPin = 9;          // Set the mode of the transceiver
const int BeaconSimPin = 4;    // Pulled high to allow simulated beacon and SPI checkout


// Defining sampling size
const int filterSamples = 15; //number of reading to average
int sensSmoothRSSI [filterSamples];   // array for holding raw sensor values for sensor1 

//Variables
int prevVector = 0;        // used to keep track of previous recoreded vector
int LEDStatus = 0;         // Flips on/off/on when a valid vectors are received
int maxRSSI = 0;           // highest RSSI on a particular scan
long lastReportTime = 0;  //millis function uses a long
int simValue = 0;          // Simulator reported value
int const historySize = 180;    // history for determining a 0-360 deg scan at 2 deg resolution

int historyIndex = 0;             // initialized to an invalid index  
int historyRSSI [historySize];    // array to record a full scan 
int historyRSSIMaxIndex = 0;      // index into historyRSSI where the RSSImax is observed during a scan
int lastGoodVector = 0;


//values for determining the current and previous headings
int previousReportVector = 0;

const float receiveThreshold = 0.98; // Percentage of max RSSI value to signal sholders of beam.

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();



void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  Serial1.begin(9600);
  xbee.setSerial(Serial1);
  
  //Define Interface Pins
  pinMode(TXRXPin, OUTPUT);
  pinMode(PDNPin, OUTPUT);
  pinMode(BeaconSimPin, INPUT);
  pinMode(GoodPacketLEDPin, OUTPUT);    
  analogReference(EXTERNAL);
  //Configure Radio for operation
  digitalWrite(TXRXPin, 0);    // Keep Transceiver in Receive
  digitalWrite(PDNPin, 1);     // Keep Radio powered ON

  //Setup this Arduino as a SPI device
  SlaveInit();

  mySerial.begin(115200);
  mySerial.println("Hello, world?");
}


void loop() {

  //Determine if the Beacon Sim Mode is selected.  This proves that communication to the Robot is possible.
  int beaconSim = digitalRead(BeaconSimPin);

  if (beaconSim == 1){
    ++simValue;
    if (simValue > 179) {
      simValue = 0; 
    }
    SPDR = simValue ;
    Serial.println(simValue);
    delay(3000);
  }

  else
  {
    int currentRSSI = 0;
    int RSSIRawData = 0;
    xbee.readPacket(100);
    
    //Check to see if a packet has been received
    if (xbee.getResponse().isAvailable()){
      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) xbee.getResponse().getRx16Response(rx16);
    }
    
    // Get the signal strength in anticipation of Vector
    currentRSSI = abs(rx16.getRssi()-100);

    //Grab the Vector
    int currentVector = rx16.getData(0);

    // Allow for a simulator transmitter which does not have a time varying vector output. 
    // if the last report was more than 3 seconds ago simply pass the last received vector as the current vector
    if((millis() - lastReportTime) > 3000){
      //Print out to the Serial port that the vector is static and briefly blink the LED for visual indication
      //Serial.println("Static Vector");
      lastReportTime = millis();
      printVector(currentVector);
      LEDStatus =1;
      digitalWrite(GoodPacketLEDPin, LEDStatus); 
      delay(750);
      LEDStatus =0;
      digitalWrite(GoodPacketLEDPin, LEDStatus);
    }   
    else {
      recordCurrentVector(currentRSSI, currentVector);
    } 
  }
}


/*-------------------------------------------------------------------------------------------------------------
 recordCurrentVector adds received RSSI and logs them according to their Vector
 
 -------------------------------------------------------------------------------------------------------------*/

void recordCurrentVector(int currentRSSI, int currentVector){

  // Populate the historyRSSI[0-179] to index and record corresponding RSSI in two deg incriments
  // enter the currentRSSI into the approriate location in the vector
  historyRSSI[currentVector] = currentRSSI; 
  
  //int candidateCurrentVector;
  /*
  Serial.print("\t\t");
   Serial.println(currentVector);
   Serial.print("\t");
   Serial.println(currentRSSI);
   */

  // keep track of vector with highest RSSI value
  if (currentRSSI > maxRSSI){
    historyRSSIMaxIndex = currentVector;   // save the index (vector) of the max RSSI
    maxRSSI = currentRSSI ;                // save the max RSSI value for later comparison
  }

  // determine the completion of a scan cycle by looking for the vector roll over.  currentVector should be smaller than prev
  if ( (currentVector + 5) < prevVector  ){
    //do scan analysis

    //use the historyRSSImaxIndex and calculate the threshold above and below maxRSSI point
    int peakRSSIVector =0;
    peakRSSIVector = findPeak(historyRSSIMaxIndex);

    // Average the upper and lower thresholdsto find the center
    int vector = reasonableVector(peakRSSIVector);

    //Make vector available to SPI and Serial 

    printVector(vector);
    // Clear out buffers and prepare for new scan
    clearData();
  }
  else
  {
    prevVector= currentVector; 
  }
}



/*-------------------------------------------------------------------------------------------------------------
 printVector provides the current vector to the serial or SPI output
 
 -------------------------------------------------------------------------------------------------------------*/


void printVector(int vector)
{
  SPDR = vector;
  Serial.println(vector);
  mySerial.println(vector);  //soft serial
  mySerial.println("                       ");  //soft serial
  previousReportVector = vector;  
  prevVector = vector;

  // If SPI has been read then set to 201 indicating that SPI has been previously fetched
  if (SPDR == 0) { 
    SPDR = 201;
  }
}


/*-------------------------------------------------------------------------------------------------------------
 reasonableVector insures that the received vector has changed not "much" since the last valid vector before 
 passing vector on to be made available.
 
 -------------------------------------------------------------------------------------------------------------*/


int reasonableVector(int candidateCurrentVector)
{
  //broken down to account for abs funtion uniquness
  int deltaSinceLastUpdate = 0;
  deltaSinceLastUpdate = abs (candidateCurrentVector - lastGoodVector);
  int currentReportVector;

  /*
  Serial.println("Finding deltaSinceLastUpdate candidateCurrentVector");
   Serial.print(deltaSinceLastUpdate);
   Serial.print(":");
   Serial.print(candidateCurrentVector);
   Serial.print(":");
   Serial.print(previousReportVector);
   Serial.print(":");
   Serial.println(candidateCurrentVector);
   */

  // determine if this reading is within 20 steps of last report  
  if (abs(deltaSinceLastUpdate) < 20){
    currentReportVector = candidateCurrentVector;
    // Change the LED status to indicate a complete scan and vector is valid
    ++LEDStatus;
    if (LEDStatus >1) {
      LEDStatus =0; 
    }
    digitalWrite(GoodPacketLEDPin, LEDStatus); 
  }
  else
  {
    //Value of 200 means that a the vector is not currently valid
    currentReportVector = 200;


  }
  lastGoodVector = candidateCurrentVector;
  return currentReportVector;
}


/*-------------------------------------------------------------------------------------------------------------
 findPeak startes from the Vector which had the highest RSSI signal and serarches higher and lower from that 
 point finding the Vector at which the RSSI reaches a threshold value indicating the sides of the received
 signal peak.  The average of the higher and lower peaks is returned as the current vector.
 
 -------------------------------------------------------------------------------------------------------------*/

int findPeak(int RSSIMaxIndex){
  //Start from the max RSSI and find the upper and lower vectors just above the threshold value.  Average those two and return
  //the center vector.
  int lowerThresholdIndex = RSSIMaxIndex;        //index of max RSSI location
  int indexValueofhistoryRSSI = historyRSSI[lowerThresholdIndex];  // Start scanning at the Max RSSI Index
  int thresholdValue = int(maxRSSI*receiveThreshold);              // Calculate RSSI Value to stop search 

  //Scan lower in vectors until thresholdValue is found and skip zero values (no reports)

  while(thresholdValue <= indexValueofhistoryRSSI && lowerThresholdIndex > 1)
  {
    --lowerThresholdIndex;  // = lowerThresholdIndex - 1;
    indexValueofhistoryRSSI= historyRSSI[lowerThresholdIndex];
    //make sure to skip Zeros - seems to be a limitation in the while Arduino preventing the additional condition check
    while (indexValueofhistoryRSSI == 0  && lowerThresholdIndex > 1) {
      --lowerThresholdIndex;
      indexValueofhistoryRSSI= historyRSSI[lowerThresholdIndex];
    }      
  }

  // Scan higher in vectors until thresholdValue is found and skip zero value (no reports)
  int higherThresholdIndex = RSSIMaxIndex;        //index of max RSSI location
  indexValueofhistoryRSSI = historyRSSI[higherThresholdIndex];  // Start scanning at the Max RSSI Index

  while((thresholdValue <= indexValueofhistoryRSSI || indexValueofhistoryRSSI == 0 ) && higherThresholdIndex <  historySize - 1 )
  {
    ++higherThresholdIndex;  
    indexValueofhistoryRSSI= historyRSSI[higherThresholdIndex];

    //make sure to skip Zeros - seems to be a limitation in the while Arduino preventing the additional condition check i the while loop
    while (indexValueofhistoryRSSI == 0 && higherThresholdIndex <  historySize - 1) {
      ++higherThresholdIndex; 
      indexValueofhistoryRSSI= historyRSSI[higherThresholdIndex];
    }   
  }
  int candidateCurrentVector = int((higherThresholdIndex + lowerThresholdIndex)/2);  
  /*
  Serial.println("Finding the index higher lower max and then currentVector");
   Serial.print(higherThresholdIndex);
   Serial.print(":");
   Serial.print(lowerThresholdIndex);
   Serial.print(":");
   Serial.print(RSSIMaxIndex);
   Serial.print(":");
   Serial.println(candidateCurrentVector);
   */
  return candidateCurrentVector;
}


/*-------------------------------------------------------------------------------------------------------------
 clearData resets variables and data structures to start a new scan
 
 -------------------------------------------------------------------------------------------------------------*/

void clearData(){
  //End cycle and start recording a new cycle
  maxRSSI = 0;
  historyRSSIMaxIndex = 0;
  historyIndex = 0;   
  //currentVector = 0;
  prevVector = 0;

  //Update the last report timer
  lastReportTime = millis();

  //Reset the RSSI Vector history for the next scan
  for(int i = 0; i < historySize; i++){
    historyRSSI [i]=0;    
  }  
}


/*-------------------------------------------------------------------------------------------------------------
 digitalSmooth provides an averaged value when sampling the RSSI analog input pin.
 
 -------------------------------------------------------------------------------------------------------------*/

//Credit for digitalSmooth function http://www.arduino.cc/playground/Main/DigitalSmooth
int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
  // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;

  i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

  /*
  for (j = 0; j < (filterSamples); j++){    // print the array to debug
   Serial.print(sorted[j]); 
   Serial.print("   "); 
   }
   Serial.println();
   */

  // throw out top 15% and bottom 60% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 60)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
    // Serial.print(sorted[j]); 
    // Serial.print("   "); 
  }

  //  Serial.println();
  //  Serial.print("average = ");
  //  Serial.println(total/k);
  return total / k;    // divide by number of samples
}

/*-------------------------------------------------------------------------------------------------------------
 SlaveInit initializes the Arduino to act as a slave SPI device to a master SPI device requesting data
 
 -------------------------------------------------------------------------------------------------------------*/

void SlaveInit(void) {
  // Set MISO output, all others input
  pinMode(SCK_PIN, INPUT);
  pinMode(MOSI_PIN, INPUT);
  pinMode(MISO_PIN, OUTPUT);
  pinMode(SS_PIN, INPUT);

  // Enable SPI
  SPCR = B00000000;
  SPCR = (1<<SPE);
}









































