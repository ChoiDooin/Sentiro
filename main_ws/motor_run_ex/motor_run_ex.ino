const int pwmPinA = 13;  // PWM 제어 핀
const int dirPinA = 7;   // 방향 제어 핀

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == '0') {
      digitalWrite(dirPin, HIGH);  // CCW 방향
      analogWrite(pwmPin, 200);    // 속도 설정
    } else if (command == '1') {
      analogWrite(pwmPin, 0);      // 모터 정지
    }
  }
}
