#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h> 
char auth[] = "wZj19yHb7gfJAhZkyf-IeZRL4nE1X54Q";   
char ssid[] = "athi";   // your network SSID (name)
char pass[] ="12345678";   // your network password 
SoftwareSerial mySerial(D7,D6);//RX TX
BLYNK_WRITE(V1)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  if(pinValue==1)
  {
    if(Serial.available()>0)
  { 
    char b=Serial.read();
    Serial.println(b);
    String str1=String(b);
    Blynk.notify("Seatavailable:"+str1);
    
  
  }

 
  }
  // process received value
}
BLYNK_WRITE(V2)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  if(pinValue==1)
  {
    if(mySerial.available()>0)
  {
    int i;
    String str;
    char a[12]={'\0'};
    for(i=0;i<12;i++)
    {
      a[i]=mySerial.read();
      delay(100);
      
    }
  Serial.print(a[12]);
   str=String(a);
  Serial.println(str);
  if(a[10]=='9')
  {
    Blynk.notify("Location 1");
  }
  else if(a[9]=='1')
  {
    Blynk.notify("Location 2");
  }
  else if(a[10]=='A')
  {
    Blynk.notify("Location 3");
  }
  }
  }
}
void setup() {
  // put your setup code here, to run once:
  Blynk.begin(auth, ssid, pass); 
  mySerial.begin(9600);   
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
   Blynk.run(); 
   
}
