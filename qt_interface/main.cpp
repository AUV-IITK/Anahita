#include <QApplication>
#include <bits/stdc++.h>
#include <QProcess>
#include <QObject>
//#include <QWidget>
#include <QWidgetList>
// #include <QQmlEngine>
#include <QQmlContext>
#include <QtQuickWidgets/QQuickWidget>
#include <QQuickView>
#include <QQuickWidget>
#include <QString>
#include "main.h"
#include "integrateros.h"
#include <QDebug>
#include <qdebug.h>
#include <QtGui>
#include <QString>
#include <QProcess>
#include <QQmlApplicationEngine>
#include <QStringList>
#include <qt5/QtQml/QtQml>
#include <qt5/QtQml/QQmlEngine>

void Motion :: Sway1(){
    // rostopic pub /ard/northsway std_msgs/Float64 "255"
    QString cmd = "rostopic pub /ard/northsway std_msgs/Int32 -- " ;
    cmd = cmd + QString::number(PWM1) ;
    // cmd = cmd + " " + QString::number(PWM2) ;
    std :: string cmd0 = cmd.toUtf8().constData();
    qDebug() << cmd0.c_str() ;
    qDebug() << "  \n" ;
    QString prog = "killall -9 roscore && killall -9 rosmaster";
    QObject *parent;
    QProcess *myProcess = new QProcess(parent);
       myProcess->start(cmd);
    // int a= std :: system(" roscore");
    // system("source /opt/ros/<Your DISTRO>/setup.bash")
    // std :: system(cmd0.c_str());   // To be uncommented
}
void Motion :: Forward1(){
    QString cmd = "rostopic pub /ard/west std_msgs/Int32 -- " ;
    cmd = cmd + QString::number(PWM3) ;
    std :: string cmd0 = cmd.toUtf8().constData();
    qDebug() << cmd0.c_str() ;
    qDebug() << "  \n" ;
    QObject *parent;
    QProcess *myProcess = new QProcess(parent);
       myProcess->start(cmd);

    //std :: system(cmd0.c_str());   // To be uncommented
}
void Motion :: Sway2(){
    // rostopic pub /ard/southsway std_msgs/Float64 "255"
    QString cmd = "rostopic pub /ard/southsway std_msgs/Int32 -- " ;
    cmd = cmd + QString::number(PWM2) ;
    // cmd = cmd + " " + QString::number(PWM2) ;
    std :: string cmd0 = cmd.toUtf8().constData();
    qDebug() << cmd0.c_str() ;
    qDebug() << "  \n" ;
    QString prog = "roscore";
    QObject *parent;
    QProcess *myProcess = new QProcess(parent);
       myProcess->start(cmd);
    // int a= std :: system(" roscore");
    // system("source /opt/ros/<Your DISTRO>/setup.bash")
    // std :: system(cmd0.c_str());   // To be uncommented
}
void Motion :: Forward2(){
    QString cmd = "rostopic pub /ard/west std_msgs/Int32 -- " ;
    cmd = cmd + QString::number(PWM4) ;
    // cmd = cmd + " " + QString::number(PWM4) ;
    std :: string cmd0 = cmd.toUtf8().constData();
    qDebug() << cmd0.c_str() ;
    qDebug() << "  \n" ;
    QObject *parent;
    QProcess *myProcess = new QProcess(parent);
       myProcess->start(cmd);
    //std :: system(cmd0.c_str());   // To be uncommented
}
void Motion :: Upward1(){
    QString cmd = "rostopic pub /ard/northup std_msgs/Int32 -- " ;
    cmd = cmd + QString::number(PWM4) ;
    // cmd = cmd + " " + QString::number(PWM4) ;
    std :: string cmd0 = cmd.toUtf8().constData();
    qDebug() << cmd0.c_str() ;
    qDebug() << "  \n" ;
    QObject *parent;
    QProcess *myProcess = new QProcess(parent);
       myProcess->start(cmd);
    //std :: system(cmd0.c_str());   // To be uncommented
}
void Motion :: Upward2(){
    QString cmd = "rostopic pub /ard/southup std_msgs/Int32 -- " ;
    cmd = cmd + QString::number(PWM4) ;
    // cmd = cmd + " " + QString::number(PWM4) ;
    std :: string cmd0 = cmd.toUtf8().constData();
    qDebug() << cmd0.c_str() ;
    qDebug() << "  \n" ;
    QObject *parent;
    QProcess *myProcess = new QProcess(parent);
       myProcess->start(cmd);
    //std :: system(cmd0.c_str());   // To be uncommented
}

void ROSfeatures :: ROScore(){
    QString cmd = "roscore" ;
    std :: string cmd0 = cmd.toUtf8().constData();
    qDebug() << cmd0.c_str() ;
    qDebug() << "  \n" ;
    QObject *parent;
    QProcess *myProcess = new QProcess(parent);
       myProcess->start(cmd);
}
void ROSfeatures :: ROScoreOff(){
    QString cmd = "killall -9 roscore && killall -9 rosmaster" ;
    std :: string cmd0 = cmd.toUtf8().constData();
    qDebug() << cmd0.c_str() ;
    qDebug() << "  \n" ;
    QObject *parent;
    QProcess *myProcess = new QProcess(parent);
       myProcess->start(cmd);
}

/*
void Motion :: updatePWM3(int tmp1){
    PWM3 = tmp1;
    //getPWM3();
    emit updatedPWM3();
    Forward();
}*/

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    //Mine moved ;
    //QQmlApplicationEngine engine;
    //engine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    Motion move;
    ROSfeatures rosFeat ;
    // rosFeat.ROScore();
    qmlRegisterType <Motion> ("Motion.header",1,0,"Remote");

    QQuickView view;
        view.engine()->rootContext()->setContextProperty("move", &move);
        view.engine()->rootContext()->setContextProperty("ROS", &rosFeat);

        view.setSource(QUrl("qrc:/main.qml"));
        view.show();

    return app.exec();
    //return 0;
}





