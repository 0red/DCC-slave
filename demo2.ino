#define CD4051_APIN A1
#define CD4051_BPIN A2
#define CD4051_CPIN A3
#define CD4051_KPIN A6
#define CD4051_LPIN A7

#define OUTPIN CD4051_LPIN
//JRcd4051 JRcd4051(OUTPIN,CD4051_APIN,CD4051_BPIN,CD4051_CPIN);


int tcrt2,tcrt1;
void setup(){
pinMode(13,OUTPUT);
Serial.begin(115200);
pinMode(CD4051_APIN,OUTPUT);
pinMode(CD4051_BPIN,OUTPUT);
pinMode(CD4051_CPIN,OUTPUT);
}

void loop(){

for (int i=0;i<8;i++) {
  digitalWrite(CD4051_APIN,bitRead(i,0));
  digitalWrite(CD4051_BPIN,bitRead(i,1));
  digitalWrite(CD4051_CPIN,bitRead(i,2));
  
  tcrt1 = analogRead(CD4051_KPIN);
  tcrt2 = analogRead(CD4051_LPIN);
  analogWrite(13,tcrt1/4);
  Serial.print(tcrt1);
  Serial.print("-");
  Serial.print(tcrt2);
  Serial.print(": ");
}  
  Serial.println(":");
  delay(250);
}
//https://showmeyourcode.org/how-to-use-tcrt5000-with-arduino/
