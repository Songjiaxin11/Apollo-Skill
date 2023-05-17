#define red 3
#define green 5
#define blue 6
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}

void loop() {
  for(int i=0;i<256;i+=10 )
  {
  analogWrite(red,i);
    delay(80);
  }
  for(int i=255;i>0;i-=10 )
  {
  analogWrite(red,i);
    delay(80);
  }
  for(int i=0;i<256;i+=10 )
  {
  analogWrite(blue,i);
    delay(80);
  }
for(int i=255;i>0;i-=10 )
  {
  analogWrite(blue,i);

    delay(80);
  }

  for(int i=0;i<256;i+=10 )
  {
  analogWrite(green,i);
    delay(80);
  }
  for(int i=255;i>0;i-=10 )
  {
  analogWrite(green,i);
    delay(80);
  }
}
