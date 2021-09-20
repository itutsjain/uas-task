#include<Servo.h>
Servo Myservo;
int pos;
void setup()
{
Myservo.attach(3);
}

void loop()
{
data = Serial.read()
  
for(pos=0;pos<=data;pos++){
Myservo.write(pos);
delay(15);
}
cout<<"payload is dropped";
  delay(1000);
  
  for(pos=data;pos>=0;pos--){
Myservo.write(pos);
delay(15);
}
  delay(1000);
  
}
