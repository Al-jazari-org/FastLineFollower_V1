#include <QTRSensors.h>
//#define DEBUG
#define LOG
enum States {NONE,CALIBRATION,HALT,RUNNING,FINISHED};
enum LineBias  {LEFTMOST,RIGHTMOST,CENTER};

QTRSensors qtr;

const uint8_t sensor_count = 8;
uint16_t sensor_data[sensor_count] ;

uint8_t bottom_pad_percnt = 90;

struct Line {
    uint8_t start,end;
    uint16_t center;
    uint16_t getDir(){
        if(start == end){
            return (uint16_t)start * 1000;
        }
        if(center > ((float)(sensor_count-1)/2) * 1000){
            return (uint16_t)end * 1000;
        }else{
            return (uint16_t)start * 1000;
        }
    };
};

struct LineArray {
    uint16_t size;
    Line data[sensor_count];
};

struct {
    const uint8_t ENA = 5;
    const uint8_t ENB = 6;
    const uint8_t IN4 = 10;
    const uint8_t IN3 = 9;
    const uint8_t IN2 = 8;
    const uint8_t IN1 = 7;

    uint8_t speed = 195;

    void init(){
        pinMode(ENA,OUTPUT);
        pinMode(ENB,OUTPUT);
        pinMode(IN4,OUTPUT);
        pinMode(IN3,OUTPUT);
        pinMode(IN2,OUTPUT);
        pinMode(IN1,OUTPUT);
        setSpeed(speed);
    }

    void setSpeed(uint8_t s){
        analogWrite(ENA,s);
        analogWrite(ENB,s);
    };

    void setSpeed(uint8_t l,uint8_t r){
        analogWrite(ENA,l);
        analogWrite(ENB,r);
    };

    void setSpeedByPd(float pd){
        bool side = (pd < 0.0f)? 1 : 0;
        if(pd == 0.0f){
            setSpeed(speed);
            return;
        }
        float norm = abs(pd) / (((float)(sensor_count-1))/2);
        float norm_cubed = pow(norm,3);
        uint8_t speed_offset = (uint8_t)(80.0f*norm_cubed);
        Serial.print("SPEED OFF: ");
        Serial.println(speed_offset);
        if(side){
            if(norm > 0.8){
                this->right();
                setSpeed(speed);
            }else{
                this->forward();
                setSpeed(speed-speed_offset,speed+speed_offset);
            }
        }else{
            if(norm > 0.8){
                this->left();
                setSpeed(speed);

            }else{
                this->forward();
                setSpeed(speed+speed_offset,speed-speed_offset);
            }
        }
    };

    void forward(){
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
    }
    void right(){
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
    };
    void left(){
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
    }
    void neutral(){
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,LOW);
    }
    void halt(){
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,HIGH);
    }
}vehicle;

struct {
    struct led{
        const uint8_t pin;

        led(uint8_t _pin) : pin((uint8_t)_pin){
        }
        void on(){
            digitalWrite(this->pin,HIGH);
        };
        void off(){
            digitalWrite(this->pin,LOW);
        };
        operator bool() const {
            return digitalRead(pin);
        };
    };

    led red = led(12);
    led yellow = led(11);
    led green = led(4);

    void init(){
        pinMode(red.pin,OUTPUT);
        pinMode(yellow.pin,OUTPUT);
        pinMode(green.pin,OUTPUT);
    }

}leds;

struct {
    const uint8_t pin = 3;

    void init(){
        pinMode(pin,INPUT_PULLUP);
    }

   // void(*onClick)();
    void onClickSet(void(*func)()){
        attachInterrupt(digitalPinToInterrupt(pin),func,FALLING);
    };
   // void(*onRelease)();
    void onReleaseSet(void(*func)()){
        attachInterrupt(digitalPinToInterrupt(pin),func,RISING);
    };

}button;

LineArray lines = {0};

volatile States state = NONE;

volatile bool running = false;

void onClick(){
    static volatile unsigned long debounce = 0;
    if(millis() - debounce < 200){
        debounce = millis();
        return;
    }
    debounce = millis();


    if(state != CALIBRATION){
        ChangeState(RUNNING);
    }
};



void setup(){

    Serial.begin(115200);
    leds.init();
    button.init();
    button.onReleaseSet(onClick);

    leds.green.on();
    delay(500);
    leds.green.off();

    leds.yellow.on();
    delay(500);
    leds.yellow.off();

    leds.red.on();
    delay(500);
    leds.red.off();

    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, sensor_count);
    qtr.setEmitterPin(2);
    qtr.setSamplesPerSensor(4);
    vehicle.init();
    pinMode(LED_BUILTIN, OUTPUT);

    vehicle.neutral();
    //Calibrate();

    Serial.println("--LOG--");
    Serial.println("MIN:");
    for(int i=0;i<sensor_count;i++){
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("MAX:");
    for(int i=0;i<sensor_count;i++){
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("Padding : P");
    Serial.print(bottom_pad_percnt);
    Serial.print(" R");
    Serial.println((float) bottom_pad_percnt / 100);


    Serial.println("PAD MIN:");
    for(int i=0;i<sensor_count;i++){
        Serial.print(GetPaddedMin(i));
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("--LOG--");

    ChangeState(HALT);

}

unsigned long t0 = 0;

void loop(){
    //t0 = millis() - t0;

    ClearLines();
    qtr.read(sensor_data);
#ifdef DEBUG
    for(int i=0;i<sensor_count;i++){
        Serial.print(GetSValue(i));
        Serial.print(" ");
    }
        Serial.println();
    for(int i=0;i<sensor_count;i++){
        if(IsSensorOnBlack(i)){
            Serial.print("-");
        }else{
            Serial.print("X");
        }

        Serial.print(" ");
    }
        Serial.println();
#endif

    if(state != RUNNING){
        vehicle.halt();
        return;
    }

    FindLines();//IMPORTANT

    static const float home = (float)(sensor_count-1) / 2 ;
    static float target = 4.5;
    static float last_error = 0;
    if(IsOnBlack()){
        if(IsOnAllBlack()){
            ChangeState(FINISHED);
            return;
        }
        target = (float)(GetLine(LEFTMOST).getDir()) / 1000;
        Serial.print("TARGET :");
        Serial.println(target);
    }
    float error = home - target;
    float pd = error + -(last_error - error) * 1.5;
    last_error = error;
    vehicle.setSpeedByPd(pd);
    /*
    if(pd > 0){
        vehicle.left();
    }else if(pd == 0){
        vehicle.neutral();
    }
    else {
        vehicle.right();
    }
    */




#ifdef LOG

    Serial.print("BlackLine :");
    if(IsOnBlack()){
        Serial.print("X");
    }
    Serial.print(" ");
    Serial.println((float)(GetLine(LEFTMOST).getDir()) / 1000);

    Serial.print("PD :");
    Serial.println(pd);

    float norm = abs(pd) / (((float)(sensor_count-1))/2);

    Serial.print("NORM :");
    Serial.println(norm);

#endif
}




void Calibrate(){
    //vehicle.left();
    ChangeState(CALIBRATION);
    vehicle.right();
    for(int i=0;i< 220;i++){
        qtr.calibrate();
    }
    vehicle.neutral();
    ChangeState(NONE);
}

uint16_t GetSValue(uint8_t idx){
    if(idx < 0 || idx > sensor_count-1)return 0;
    return  sensor_data[idx];
};

uint16_t GetMin(uint8_t idx){
    if(idx < 0 || idx > sensor_count-1)return 0;
    return qtr.calibrationOn.minimum[idx];
};


uint16_t GetMax(uint8_t idx){
    if(idx < 0 || idx > sensor_count-1)return 0;
    return qtr.calibrationOn.maximum[idx];
};

uint16_t GetPaddedMin(uint8_t idx){
    float ratio = (float)bottom_pad_percnt / 100;
    uint16_t min_v = GetMin(idx);
    uint16_t max_v = GetMax(idx);
    return min_v + (uint16_t)(ratio* (  max_v - min_v ) );
};

bool IsSensorOnBlack(uint8_t idx){
    uint16_t v = GetSValue(idx);
    if(v == 0) return false;
    //if(v > GetPaddedMin(idx)) return true;
    if(v > 900) return true;
    return false;
};

bool IsOnAllBlack(){
    bool ret = true;
    for(int i=0;i<sensor_count;i++){
        ret &= IsSensorOnBlack(i);
        if(!ret) return ret;
    }
    return true;
};

bool IsOnBlack(){
    bool ret = false;
    for(int i=0;i<sensor_count;i++){
        ret |= IsSensorOnBlack(i);
        if(ret)break;
    }
    return ret;
}

void ClearLines(){
    memset(&lines,0,sizeof(LineArray));
}

void AddLine(uint8_t start,uint8_t end){
    if(lines.size >= sensor_count)return ;
    Line* line = &(lines.data[lines.size]);
    line->start = start;
    line->end = end;
    lines.size ++;
}

void CalcLines(){
    for(int i=0;i<lines.size;i++){
        Line *l = &lines.data[i];
        uint16_t delta = l->end - l->start;
        l->center = delta * 1000;
        if(delta == 0){
            l->center = l->start * 1000;
            break;
        }
        l->center = l->start * 1000 + l->center / 2;
    }
};

Line GetLine(LineBias bias){
    if(lines.size == 0 ) return (const Line){0};
    if(lines.size == 1 ) return lines.data[0];
    switch(bias){
        case LEFTMOST:{
            Line *l = &(lines.data[0]);
            for(int i=1;i<lines.size;i++){
                if(lines.data[i].start == 0)return lines.data[i];
                if(lines.data[i].start < l->start) l = &(lines.data[i]);
            }
            break;
        }
        case RIGHTMOST:{
            Line *l = &(lines.data[0]);
            for(int i=1;i<lines.size;i++){
                if(lines.data[i].end == sensor_count-1)return lines.data[i];
                if(lines.data[i].end > l->end) l = &(lines.data[i]);
            }
            break;
        }

    }

};


uint16_t FindLines(){
    int16_t start = -1;
    for(int i=0;i<sensor_count;i++){
        if(IsSensorOnBlack(i)){
            if(start == -1){
                start = i;
            }else{

            }
        }else if(start != -1){
            AddLine(start,i-1);
            start = -1;
        }

    }
    if (start > 0){
        AddLine(start,sensor_count-1);
    }
    CalcLines();
    return lines.size;
};


void ChangeState(States nState){
    switch(state){
        case CALIBRATION:{
            leds.red.off();
        };
            break;
        case HALT:{
            leds.red.off();
            leds.yellow.off();
        };
            break;
        case RUNNING:{
            leds.yellow.off();
            vehicle.halt();
        };
            break;
        case FINISHED:{
            leds.green.off();
        };
            break;
    }
    switch(nState){
        case CALIBRATION:{
            leds.red.on();
        };
            break;
        case HALT:{
            leds.red.on();
            leds.yellow.on();
        };
            break;
        case RUNNING:{
            leds.yellow.on();

        };
            break;
        case FINISHED:{
            leds.green.on();
        };
            break;
        case NONE:{
            leds.green.off();
            leds.red.off();
            leds.yellow.off();
        }
    }
    state = nState;
}
