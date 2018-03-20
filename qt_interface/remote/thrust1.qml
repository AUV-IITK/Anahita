import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Rectangle {

    property int flag3 : 0
    property int dirn: 1

    height:228;
    width:246;
    id:detTh1;
    Column{
        spacing: 40;
        anchors.horizontalCenter: detTh1.horizontalCenter;
        anchors.verticalCenter: detTh1.verticalCenter;
        id:col;
        Slider{
            id:setPWM1;
            maximumValue: 255.0
            stepSize: 1.0
            activeFocusOnPress : true
            onValueChanged: {
                //if (flag3 == 0){
                    move.updatePWM1(dirn * this.value)
                //}
                //else{
                 //   flag3 = 0;
                //}


                //fwdPWMchanged3(this.value)
                //console.log(this.value)
            }
            Connections{
                target: move ;
                onUpdatedPWM1 :{
                    console.log(121) ;
                    //flag3 = 1
                    setPWM1.value = Math.abs(move.getPWM1());
                    dirn = Math.abs(move.getPWM1()) / move.getPWM1();

                }
            }
        }
        Row{
            spacing: 25;
            anchors.horizontalCenter: col.horizontalCenter;
            Button{

                height:55;
                width:55;
                id:clockwise;

                onClicked: {
                    console.log(456)
                    dirn = 1 ;
                    move.updatePWM1(dirn * setPWM1.value);
                }
                style: ButtonStyle {
                    background: Rectangle {
                        border.width: 1;
                        radius: 10;

                    }
                 }
                Image {
                    source: "clock.png"
                    height:35;
                    width:35;
                    anchors.horizontalCenter: clockwise.horizontalCenter;
                    anchors.verticalCenter: clockwise.verticalCenter;
                }
            }
            Button{
                height:55;
                width:55;
                id:anticlockwise;
                //border.width: 1;
                //radius:10;
                onClicked: {
                    console.log(456)
                    dirn = -1 ;
                    move.updatePWM1(dirn * setPWM1.value);
                }
                style: ButtonStyle {
                    background: Rectangle {
                        border.width: 1;
                        radius: 10;

                    }
                 }



                Image {
                    source: "anticlock.png"
                    height:35;
                    width:35;
                    anchors.horizontalCenter: anticlockwise.horizontalCenter;
                    anchors.verticalCenter: anticlockwise.verticalCenter;

                    }


            }
        }


        }
    }


