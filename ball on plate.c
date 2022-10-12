#include <Servo.h>
#include <PID_v1.h>
Servo myservo[2];

int xStable = 86;
int yStable = 85;
int xPosition1 = 0;
int yPosition1 = 0;
float xVelocity = 0;
float yVelocity = 0;
//212 159
double XSetpoint, XInput, XOutput, XServoOutput;
double YSetpoint, YInput, YOutput, YServoOutput;
/*

        작은공
        float Kp = 0.05;               //P게인 값
        float Ki = 0.008;                  //I게인 값
        float Kd = 0.02;                  //D게인 값
*/

//큰공
float Kp = 0.25;               //P게인 값
float Ki = 0.1;                  //I게인 값
float Kd = 0.1;                  //D게인 값
unsigned int noTouchCount = 0; //viariable for noTouch

#define Xresolution 848.0 //128  280 ~ 1000
#define Yresolution 640.0 //64   280 ~ 1000

PID xPID(&XInput, &XOutput, &XSetpoint, Kp, Ki, Kd, DIRECT);   //PID객체 생성
PID yPID(&YInput, &YOutput, &YSetpoint, Kp, Ki, Kd, DIRECT);   //PID객체 생성

void setup()
{
        Serial.begin(9600);
       

        myservo[0].attach(2);
        myservo[1].attach(3);
        myservo[0].write(xStable);
        myservo[1].write(yStable);
        xPID.SetMode(AUTOMATIC);               //PID모드를 AUTOMATIC으로 설정
        xPID.SetOutputLimits(-50, 50);         //PID의 값을 최소 -80부터 최대 80까지 설정
        yPID.SetMode(AUTOMATIC);               //PID모드를 AUTOMATIC으로 설정
        yPID.SetOutputLimits(-50, 50);         //PID의 값을 최소 -80부터 최대 80까지 설정
        xPID.SetSampleTime(50);
        yPID.SetSampleTime(50);


        for ( int i = 0 ; i  < 100 ; i ++)
        {
                myservo[0].write(xStable - 50 + i);
                myservo[1].write(yStable - 50 + i) ;
                delay(10);
        }
        for ( int i = 0 ; i  < 100 ; i ++)
        {
                myservo[0].write(xStable + 50 - i);
                myservo[1].write(yStable + 50 - i) ;
                delay(10);
        }
        myservo[0].write(xStable );
        myservo[1].write(yStable) ;
        Serial.println("start");

}
int xPosition0 = -1;
int yPosition0 = -1;
void loop()
{

        while (1)
        {
                pinMode(A1, OUTPUT);
                pinMode(A3, OUTPUT);
                pinMode(A0, INPUT);
                pinMode(A2, INPUT);
                digitalWrite(A1, LOW);
                digitalWrite(A3, HIGH);
                delay(2);
                xPosition1 = 1023-analogRead(A2) ;
                xPosition1 = 1023-analogRead(A2) ;

                pinMode(A0, OUTPUT);
                pinMode(A2, OUTPUT);
                pinMode(A1, INPUT);
                pinMode(A3, INPUT);
                digitalWrite(A0, LOW);
                digitalWrite(A2, HIGH);
                delay(2);
                yPosition1 = analogRead(A3);
                yPosition1 = analogRead(A3);

                if (xPosition1 > 0 && yPosition1 < 1023)
                {

                        noTouchCount = 0;
                        if (xPosition1 < 110)
                                xPosition1 = 0;
                        else
                                xPosition1 = xPosition1 - 110;
                        if (yPosition1 < 110)
                                yPosition1 = 0;
                        else
                                yPosition1 = yPosition1 - 110;

                        xPosition1 = (xPosition1) / (803.0 / Xresolution); //Reads X axis touch position
                        yPosition1 = (yPosition1) / (803.0 / Yresolution); //Reads Y axis touch position
                        /*      if ( xPosition0 - xPosition1 > 30 * 4 || xPosition0 - xPosition1 < -30 * 4 ||
                                        yPosition0 - yPosition1 > 30 * 4 || yPosition0 - yPosition1 < -30 * 4 )
                                {
                                if (xPosition0 != -1)
                                {
                                        continue;
                                }
                                xPosition0 = -1;

                                }*/

                        xPosition0 = xPosition1;
                        yPosition0 = yPosition1;

                        if (xPosition1 > (Xresolution / 2) - 30 && xPosition1 < (Xresolution / 2) + 30 &&
                                        yPosition1 > (Yresolution / 2) - 30 && yPosition1 < (Yresolution / 2) + 30)
                        {
                                //   myservo[0].write(xStable);
                                //   myservo[1].write(yStable);
                                //    xPosition0 = -1;
                                //   Serial.print("good ");
                                //   continue;

                        }

                        // xPosition1= 120;
                        // yPosition1 = 20;

                        //      Serial.print("ON : :");
                        XSetpoint = Xresolution / 2;                       //막대 중앙 위치(Set Point를 15cm로 설정)
                        XInput = xPosition1;                //공의 위치 측정

                        xPID.Compute();                       //PID계산

                        XServoOutput = xStable + (XOutput);            //서보모터의 각도 설정(100도는 서보모터가 수평을 이루었을 때 각도)


                        YSetpoint = Yresolution / 2;                       //막대 중앙 위치(Set Point를 15cm로 설정)
                        YInput = yPosition1;                //공의 위치 측정

                        yPID.Compute();                       //PID계산

                        YServoOutput = yStable + (YOutput);            //서보모터의 각도 설정(100도는 서보모터가 수평을 이루었을 때 각도)

                        Serial.print("     ");
                        myservo[0].write(XServoOutput);            //서보모터에게 값 전달
                        myservo[1].write(YServoOutput);            //서보모터에게 값 전달


                        Serial.print("    X = ");

                        Serial.print(xPosition1);

                        Serial.print(" Y = ");
                        Serial.print(yPosition1);

                        Serial.print("     X = ");
                        Serial.print(XOutput);

                        Serial.print(" Y = ");
                        Serial.println(YOutput);
                }
                else
                {
                        noTouchCount++;

                        if (noTouchCount == 400)
                        {
                                Serial.println("not touch");
                                xPosition0 = -1;
                                myservo[0].write(xStable);
                                myservo[1].write(yStable);
                        }
                }
                //  if (xPosition1 ==85 && yPosition1 ==50)
                //Serial.println("!");





                // delay(200);

        }
}