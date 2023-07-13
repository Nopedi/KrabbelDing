#include <Servo.h>

Servo servo_0Rh;
Servo servo_0Rv;

Servo servo_0Lh;
Servo servo_0Lv;

Servo servo_1Rh;
Servo servo_1Rv;

Servo servo_1Lh;
Servo servo_1Lv;

Servo servo_2Rh;
Servo servo_2Rv;

Servo servo_2Lh;
Servo servo_2Lv;

String vals = "";
int val_0Rh, val_0Rv;
int val_0Lh, val_0Lv;

int val_1Rh, val_1Rv;
int val_1Lh, val_1Lv;

int val_2Rh, val_2Rv;
int val_2Lh, val_2Lv;

int BASE_VALUE = 90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(37);
  servo_0Lh.attach(6);
  servo_0Lv.attach(7);

  servo_0Rh.attach(8);
  servo_0Rv.attach(9);

  servo_1Lh.attach(12);
  servo_1Lv.attach(13);

  servo_1Rh.attach(10);
  servo_1Rv.attach(11);

  servo_2Lh.attach(5);
  servo_2Lv.attach(4);

  servo_2Rh.attach(3);
  servo_2Rv.attach(2);

  servo_0Rh.write(BASE_VALUE);
  servo_0Rv.write(BASE_VALUE);

  servo_0Lh.write(BASE_VALUE);
  servo_0Lv.write(BASE_VALUE);

  servo_1Rh.write(BASE_VALUE);
  servo_1Rv.write(BASE_VALUE);

  servo_1Lh.write(BASE_VALUE);
  servo_1Lv.write(BASE_VALUE);

  servo_2Rh.write(BASE_VALUE);
  servo_2Rv.write(BASE_VALUE);

  servo_2Lh.write(BASE_VALUE);
  servo_2Lv.write(BASE_VALUE);
}


void loop() {
  // check if data is available
}

void serialEvent() {
  vals = Serial.readString();
  val_0Lh = vals.substring(0, 3).toInt();
  val_0Lv = vals.substring(3, 6).toInt();
  val_0Rh = vals.substring(6, 9).toInt();
  val_0Rv = vals.substring(9, 12).toInt();

  val_1Lh = vals.substring(12, 15).toInt();
  val_1Lv = vals.substring(15, 18).toInt();
  val_1Rh = vals.substring(18, 21).toInt();
  val_1Rv = vals.substring(21, 24).toInt();

  val_2Lh = vals.substring(24, 27).toInt();
  val_2Lv = vals.substring(27, 30).toInt();
  val_2Rh = vals.substring(30, 33).toInt();
  val_2Rv = vals.substring(33, 36).toInt();

  servo_0Rv.write(val_0Rv);
  servo_0Rh.write(val_0Rh);
  servo_0Lv.write(val_0Lv);
  servo_0Lh.write(val_0Lh);

  servo_1Rv.write(val_1Rv);
  servo_1Rh.write(val_1Rh);
  servo_1Lv.write(val_1Lv);
  servo_1Lh.write(val_1Lh);
  
  servo_2Rv.write(val_2Rv);
  servo_2Rh.write(val_2Rh);
  servo_2Lv.write(val_2Lv);
  servo_2Lh.write(val_2Lh);

  String out = "0Lh " + String(val_0Lh) + "; 0Lv" + String(val_0Lv) + "; ORh " + String(val_0Rh) + "; 0Rv " + String(val_0Rv)
  + "1Lh " + String(val_1Lh) + "; 1Lv " + String(val_1Lv) + "; 1Rh " + String(val_1Rh) + "; 1Rv " + String(val_1Rv)
  + "2Lh " + String(val_2Lh) + "; 2Lv " + String(val_2Lv) + "; 2Rh " + String(val_2Rh) + "; 2Rv " + String(val_2Rv);
  Serial.println(out);
}
