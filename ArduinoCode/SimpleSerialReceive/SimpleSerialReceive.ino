int IN1 = 7;
int IN2 = 8;
int PWMA = 10;

void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  digitalWrite(PWMA, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    String receivedString = Serial.readStringUntil('\n'); // Read until semicolon character
    delay(10);
    if(receivedString == "forward"){
      Serial.println("f");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PWMA,255);

    }else if(receivedString == "backwards"){
      Serial.println("b");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(PWMA,255);
    }else if(receivedString == "right"){
      Serial.println("r");
    }else if(receivedString == "left"){
      Serial.println("l");
    }else if(receivedString == "stop"){
      Serial.println("s");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }else{
      Serial.println("#");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  }
  delay(10);
}
