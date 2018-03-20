#ifndef MAIN_H
#define MAIN_H
#include <QObject>
#include <qdebug.h>
class Motion : public QObject
{
    Q_OBJECT
    //Q_PROPERTY(NOTIFY updatedPWM3)
public:
    Motion() {
        PWM1 = 0;
        PWM2 = 0;
        PWM3 = 0;
        PWM4 = 0;
        PWM5 = 0;
        PWM6 = 0;
    }
    void Forward1();
    void Sway1();
    void Forward2();
    void Sway2();
    void Upward1();
    void Upward2();


public Q_SLOTS:
    void updatePWM1(int tmp){
        PWM1 = tmp;
        //getPWM3();
        emit updatedPWM1();
        Sway1();
    }
    void updatePWM2(int tmp){
        PWM2 = tmp;
        //getPWM3();
        emit updatedPWM2();
        Sway2();
    }
    void updatePWM3(int tmp){
        PWM3 = tmp;
        //getPWM3();
        emit updatedPWM3();
        Forward1();
    }
    void updatePWM4(int tmp){
        PWM4 = tmp;
        //getPWM3();
        emit updatedPWM4();
        Forward2();
    }
    void updatePWM5(int tmp){
        PWM5 = tmp;
        //getPWM3();
        emit updatedPWM5();
        Upward1();
    }
    void updatePWM6(int tmp){
        PWM6 = tmp;
        //getPWM3();
        emit updatedPWM6();
        Upward2();
    }



    // Functions to return current PWM values
    int getPWM1(){
        return PWM1;
    }
    int getPWM2(){
        return PWM2;
    }
    int getPWM3(){
        return PWM3;
    }
    int getPWM4(){
        return PWM4;
    }
    int getPWM5(){
        return PWM5;
    }
    int getPWM6(){
        return PWM6;
    }


/*signals:
    void valueChanged(int newValue);*/
signals :
    void updatedPWM1();
    void updatedPWM2();
    void updatedPWM3();
    void updatedPWM4();
    void updatedPWM5();
    void updatedPWM6();

private:
    int PWM1;
    int PWM2;
    int PWM3;
    int PWM4;
    int PWM5;
    int PWM6;

};



#endif // MAIN_H
