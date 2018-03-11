import QtQuick 2.0
import QtQuick.Controls 1.4
 import QtQuick.Controls.Styles 1.4
Rectangle {

    signal fwdPWMchanged3(int nwPWM3)
    property int flag3 : 0
    property int dirn: 1
    height:228;
    width:246;
    id:detTh3;
    Column{
        spacing: 40;
        anchors.horizontalCenter: detTh3.horizontalCenter;
        anchors.verticalCenter: detTh3.verticalCenter;
        id:col;
        Slider{
            id:setPWM3;
            maximumValue: 255.0
            stepSize: 1.0
            activeFocusOnPress : true
            onValueChanged: {
                //if (flag3 == 0){
                    move.updatePWM3(dirn * this.value)
                //}
                //else{
                 //   flag3 = 0;
                //}


                //fwdPWMchanged3(this.value)
                //console.log(this.value)
            }
            Connections{
                target: move ;
                onUpdatedPWM3 :{
                    console.log(121) ;
                    //flag3 = 1
                    setPWM3.value = Math.abs(move.getPWM3());
                    dirn = Math.abs(move.getPWM3()) / move.getPWM3();

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
                    move.updatePWM3(dirn * setPWM3.value);
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
                    move.updatePWM3(dirn * setPWM3.value);
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
