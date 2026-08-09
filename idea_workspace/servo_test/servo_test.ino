
const int head       = 9;   


void setup() {
    Serial.begin(9600);
}

void loop() {

    Serial.println("Enter Value: ");
    while (Serial.available() == 0) {
        
    }
    set_servo();
}

void set_servo() {
    if (Serial.available() > 0) {
        int serVal = Serial.parseInt();

        analogWrite(head, serVal);
        Serial.print("Servo moved to: ");
        Serial.println(serVal);
    }
}