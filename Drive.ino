void setup() {
  // put your setup code here, to run once:o
pinMode(7,OUTPUT);
pinMode(6,OUTPUT);
pinMode(3,OUTPUT);
pinMode(2,OUTPUT);
Serial.begin(9600);
}
void RF()
{
analogWrite(7,255);
digitalWrite(6,1);
}
void LF()
{
  analogWrite(3,255);
digitalWrite(2,1);
}
void loop() {
  // put your main code here, to run repeatedly:
  char a=Serial.read();
if(a=='W')
{
  RF();
  LF();
}
else if(a=='L')
{
  RF();
    analogWrite(3,255);
digitalWrite(2,0);
}
else if(a=='R')
{
  LF();
    analogWrite(7,255);
digitalWrite(6,0);
}
else if(a=='N')
{
  analogWrite(3,0);
  analogWrite(7,0);
}
}
