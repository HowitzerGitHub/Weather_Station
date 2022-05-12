#include <Adafruit_BMP280.h> // include the BMP Library
#include <Wire.h>
#include <MPU6050.h>
#include <dht.h>
#include<string>

#define dht_apin A0

dht DHT;
MPU6050 mpu;
Adafruit_BMP280 bmp; // I2C Interface


int pin2 = 3;
int pin1 = 2;
unsigned long duration1;
unsigned long duration2;

unsigned long starttime;
unsigned long sampletime_ms = 3000;//sampe 1s ;
unsigned long lowpulseoccupancy1 = 0;
unsigned long lowpulseoccupancy2 = 0;
float ratio1 = 0;
float ratio2 = 0;
float concentration1 = 0;
float concentration2 = 0;
String s="";

void setup() {
  Serial.begin(115200);
  Serial.println(F("BMP280 test"));

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G,0x68))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  checkSettings();

  
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  starttime = millis();//get the current time;

  
  
}




void checkSettings ()
{
  Serial.println ();

  Serial.print (" * Sleep Mode:        ");
  Serial.println (mpu.getSleepEnabled ()? "Enabled" : "Disabled");

  Serial.print (" * Clock Source:      ");
  switch (mpu.getClockSource ())
    {
    case MPU6050_CLOCK_KEEP_RESET:
      Serial.
  println ("Stops the clock and keeps the timing generator in reset");
      break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ:
      Serial.println ("PLL with external 19.2MHz reference");
      break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ:
      Serial.println ("PLL with external 32.768kHz reference");
      break;
    case MPU6050_CLOCK_PLL_ZGYRO:
      Serial.println ("PLL with Z axis gyroscope reference");
      break;
    case MPU6050_CLOCK_PLL_YGYRO:
      Serial.println ("PLL with Y axis gyroscope reference");
      break;
    case MPU6050_CLOCK_PLL_XGYRO:
      Serial.println ("PLL with X axis gyroscope reference");
      break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:
      Serial.println ("Internal 8MHz oscillator");
      break;
    }

  Serial.print (" * Gyroscope:         ");
  switch (mpu.getScale ())
    {
    case MPU6050_SCALE_2000DPS:
      Serial.println ("2000 dps");
      break;
    case MPU6050_SCALE_1000DPS:
      Serial.println ("1000 dps");
      break;
    case MPU6050_SCALE_500DPS:
      Serial.println ("500 dps");
      break;
    case MPU6050_SCALE_250DPS:
      Serial.println ("250 dps");
      break;
    }

  Serial.print (" * Gyroscope offsets: ");
  Serial.print (mpu.getGyroOffsetX ());
  Serial.print (" / ");
  Serial.print (mpu.getGyroOffsetY ());
  Serial.print (" / ");
  Serial.println (mpu.getGyroOffsetZ ());

  Serial.println ();
}

void loop() {

    duration1 = pulseIn(pin1, LOW);
    duration2 = pulseIn(pin2, LOW);
    lowpulseoccupancy1 = lowpulseoccupancy1+duration1;
    lowpulseoccupancy2 = lowpulseoccupancy2+duration2;
    s="";

    //Serial.print("*********************************Begin***************************************\n");
    
    DHT.read11(dht_apin);
    /*Serial.print("DHT readings");
    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature); 
    Serial.println("C  ");

    Serial.print("\nBMP readings");
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
    Serial.println(" hPa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1019.66)); //The "1019.66" is the pressure(hPa) at sea level in day in your region
    Serial.println(" m");                    //If you don't know it, modify it until you get your current altitude

    Serial.println();*/
    s=s+DHT.humidity+" "+DHT.temperature+" "+bmp.readTemperature()+" "+bmp.readPressure()/100+" "+bmp.readAltitude(1019.66)+" ";
    
    
    //Serial.print("MPU6050-readings");

  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();

  /*Serial.print(" Xraw = ");
  Serial.print(rawGyro.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawGyro.YAxis);
  Serial.print(" Zraw = ");
  Serial.println(rawGyro.ZAxis);

  Serial.print("\t         Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normGyro.ZAxis);
  Serial.print("\n\n");*/

  s=s+rawGyro.XAxis+" "+rawGyro.YAxis+" "+rawGyro.ZAxis+" "+normGyro.XAxis+" "+normGyro.YAxis+" "+normGyro.ZAxis+" ";


    if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
  {
    ratio1 = lowpulseoccupancy1/(sampletime_ms*10.0);  // Integer percentage 0=>100
    concentration1 = 1.1*pow(ratio1,3)-3.8*pow(ratio1,2)+520*ratio1+0.62; // using spec sheet curve

    ratio2 = lowpulseoccupancy2/(sampletime_ms*10.0);  // Integer percentage 0=>100
    concentration2 = 1.1*pow(ratio2,3)-3.8*pow(ratio2,2)+520*ratio2+0.62; // 
    
    /*Serial.print("PM10 ");

       
    Serial.print("concentration1 = ");
    Serial.print(concentration1);
    Serial.print(" pcs/0.01cf  -  ");

    Serial.print("concentration2 = ");
    Serial.print(concentration2);
    Serial.print(" pcs/0.01cf  -  ");*/

    s=s+" "+concentration1+" "+concentration2+" ";

    
    if (concentration1 < 1000) {

     //Serial.print("CLEAN\n");
     s=s+"CLEAN"+" ";
    
  }
    if (concentration1 > 1000 && concentration1 < 10000) {

     Serial.print("GOOD\n");

    }
    
    if (concentration1 > 10000 && concentration1 < 20000) {

     Serial.print("ACCEPTABLE\n");

    }
      if (concentration1 > 20000 && concentration1 < 50000) {

     Serial.print("HEAVY\n");
  }

    if (concentration1 > 50000 ) {

     Serial.print("HAZARD\n");

     
    } 
      
    lowpulseoccupancy1 = 0;
    lowpulseoccupancy2 = 0;
    starttime = millis();
  }



  Serial.println("\n");
  int sensorValue = analogRead(A1);

   Serial.println(sensorValue);
  if (sensorValue<900)
  {
      Serial.println("it is raining");
    }
   else
   {
       Serial.println("Weather is clear");
   }
  

  Serial.print("*********************************End***************************************\n\n\n\n");
  
  delay(3000);
}
