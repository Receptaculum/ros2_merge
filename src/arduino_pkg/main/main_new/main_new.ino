// 주의 : 코드의 정상 동작 여부를 검증할 수 없음!

// 지령 메시지 저장소 선언
char msg[3];

// 제어 명령값 변수 선언
int steer = 0;
int left_speed = 0;
int right_speed = 0;

// 제어 빈도
const unsigned long control_period = 50;
unsigned long time_mem = 0;

// 설정 핀
const int STEER_MTR_1 = 2;
const int STEER_MTR_2 = 3;
const int RIGHT_MTR_1 = 4;
const int RIGHT_MTR_2 = 5;
const int LEFT_MTR_1 = 6;
const int LEFT_MTR_2 =7;
const int POT = A2;

// 저항 값
const int R_MAX_LEFT = 460;
const int R_MAX_RIGHT = 352;

// 실제 회전 각도
const int REAL_ANGLE_LEFT = -60;
const int REAL_ANGLE_RIGHT = 60;

// 저항각 저장 변수
int steer_measured = 0;

void steer_mtr_set(int steer_command, int steer_measured);
void left_mtr_set(int speed);
void right_mtr_set(int speed);
void read_command_data();
void debug_print();

void setup() {
    // Serial 통신 설정
    Serial.begin(38400);

    // PinMode 설정
    pinMode(STEER_MTR_1, OUTPUT);
    pinMode(STEER_MTR_2, OUTPUT);
    pinMode(RIGHT_MTR_1, OUTPUT);
    pinMode(RIGHT_MTR_2, OUTPUT);
    pinMode(LEFT_MTR_1, OUTPUT);
    pinMode(LEFT_MTR_2, OUTPUT);
    pinMode(POT, INPUT);
}


void loop() {
    read_command_data();

    if(millis() - time_mem >= control_period) {
 
        steer = constrain(steer, REAL_ANGLE_LEFT, REAL_ANGLE_RIGHT);
        left_speed = constrain(map(left_speed, -128, 127, -255, 255), -255, 255);
        right_speed = constrain(map(right_speed, -128, 127, -255, 255), -255, 255);

        steer_measured = map(analogRead(POT), R_MAX_LEFT, R_MAX_RIGHT, REAL_ANGLE_LEFT, REAL_ANGLE_RIGHT);

        steer_mtr_set(steer, steer_measured);
        left_mtr_set(left_speed);
        right_mtr_set(right_speed);

        debug_print();

        time_mem = millis();
    }
}

void steer_mtr_set(int steer_command, int steer_measured) {

    //           0
    //           |
    //           |
    // -90 -------------- 90

    // err > 0 : 좌편향 -> 우로 조향
    // err < 0 : 우편향 -> 좌로 조향
    int err = steer_command - steer_measured;
    int steer_intensity = map(abs(err), 0, 100, 100, 255);

    // err > 0 : 좌편향 -> 우로 조향
    if(err > 0) {
        analogWrite(STEER_MTR_1, steer_intensity);
        analogWrite(STEER_MTR_2, 0);
    }

    // err < 0 : 우편향 -> 좌로 조향
    else if(err < 0) {
        analogWrite(STEER_MTR_1, 0);
        analogWrite(STEER_MTR_2, steer_intensity);
    }   

    else {
        analogWrite(STEER_MTR_1, 0);
        analogWrite(STEER_MTR_2, 0);
    }
}

void left_mtr_set(int speed) {
    if(speed >= 0) {
        // 정회전
        analogWrite(LEFT_MTR_1, speed);
        analogWrite(LEFT_MTR_2, 0);
    }

    else {
        // 역회전
        analogWrite(LEFT_MTR_1, 0);
        analogWrite(LEFT_MTR_2, -speed);
    }
}

void right_mtr_set(int speed) {
    if(speed >= 0) {
        // 정회전
        analogWrite(RIGHT_MTR_1, speed);
        analogWrite(RIGHT_MTR_2, 0);
    }

    else {
        // 역회전
        analogWrite(RIGHT_MTR_1, 0);
        analogWrite(RIGHT_MTR_2, -speed);
    }
}

void read_command_data() {
    // 수신받는 데이터는 항상 3 Byte임
    if(Serial.available() >= 3) {

        for(int i = 0; i < 3; i++) {
            msg[i] = Serial.read();
            }
    
            // 제어 명령 저장
            steer = msg[0];
            left_speed = msg[1];
            right_speed = msg[2];
      }
}

void debug_print() {
    Serial.print(steer);
    Serial.print(" ");
    Serial.print(left_speed);
    Serial.print(" ");
    Serial.print(right_speed);
    Serial.print("\n");
}