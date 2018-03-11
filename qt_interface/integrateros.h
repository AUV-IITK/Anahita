#ifndef INTEGRATEROS_H
#define INTEGRATEROS_H
#include <QObject>
class ROSfeatures : public QObject
{
    Q_OBJECT
public:
    ROSfeatures() {

    }




public Q_SLOTS:

void ROScore();
void ROScoreOff();
/*signals:
    void valueChanged(int newValue);*/
signals :


private:

};


#endif // INTEGRATEROS_H
