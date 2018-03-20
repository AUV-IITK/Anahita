import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import Motion.header 1.0

Rectangle {

    property int pWM1 : 0
    property int pWM2: 0

    id:page;
    height:420;
    width:250;
    border.color:"black";
    border.width: 2;
    Column{
        anchors.horizontalCenter: page.horizontalCenter;
        anchors.verticalCenter: page.verticalCenter;
        Rectangle{
         height: 230;
         width: 250;
         border.color: 'black';
         border.width: 2;
         id:cover;
         TabView{
             height: 228;
             width: 246;
             anchors.horizontalCenter: cover.horizontalCenter;
             anchors.verticalCenter: cover.verticalCenter;
             Tab{
                 title: 'T-1'
                 Loader{
                     source: 'thrust1.qml'
                 }
             }
             Tab{
                 title: 'T-2'
                 Loader{
                     source: 'thrust2.qml'
                 }
             }

             style: TabViewStyle {
                 frameOverlap: 3
                 tab: Rectangle {
                     color: styleData.selected ? "#F6F6F6" :"lightgrey"
                     border.color:  "black"
                     implicitWidth: Math.max(text.width + 30, 80)
                     implicitHeight: styleData.selected ? "32" :"30"
                     radius: 2.5
                     Text {
                         id: text
                         anchors.centerIn: parent
                         text: styleData.title
                         color:  "black"
                     }
                 }
                 frame: Rectangle { color: "white" ;border.color:'black';border.width: 1;}
             }

         }


        }
        Rectangle{
         height: 190;
         width: 250;
         border.width: 2;
         border.color: 'black';
         id:detailsPWM
         Grid{
             columns:2;
             rows:2;
             rowSpacing:30;
             anchors.horizontalCenter: detailsPWM.horizontalCenter;
             anchors.verticalCenter: detailsPWM.verticalCenter;

             //PWM 1
             Rectangle{
                 height:50;
                 width:130;
                 color:'black';
                 id:p1;
                 Text{
                     text:'PWM 2';
                     color:'white';
                     font.pointSize: 12;
                     anchors.horizontalCenter: p1.horizontalCenter;
                     anchors.verticalCenter: p1.verticalCenter;
                     font.bold: true;

                 }
             }
             Rectangle{
                 height:50;
                 width:70;
                 border.color:'black';
                 id:pv1
                 TextField{
                     text:move.getPWM1();
                     horizontalAlignment : TextInput.AlignHCenter;
                     //color:'black';
                     font.pointSize: 12;
                     anchors.horizontalCenter: pv1.horizontalCenter;
                     anchors.verticalCenter: pv1.verticalCenter;
                     width: 68 ;
                     height : 48;
                     font.bold: true;
                     id:valuePWM1
                     validator: IntValidator {bottom: -255; top: 255;}
                     focus: true
                     onAccepted: {
                         move.updatePWM1(this.text);
                     }


                     Connections{
                         target: move ;
                         onUpdatedPWM1 :{
                             console.log(121) ;
                             valuePWM1.text=move.getPWM1();
                         }
                     }

                    /* Remote{
                         onUpdatedPWM1: {
                             valuePWM1.text = move.getPWM1();
                             console.log("ccccc\n");
                         }
                     }*/
                 }

             }


             //PWM 2
             Rectangle{
                 height:50;
                 width:130;
                 color:'black';
                 id:p2;
                 Text{
                     text:'PWM 2';
                     color:'white';
                     font.pointSize: 12;
                     anchors.horizontalCenter: p2.horizontalCenter;
                     anchors.verticalCenter: p2.verticalCenter;
                     font.bold: true;

                 }
             }
             Rectangle{
                 height:50;
                 width:70;
                 border.color:'black';
                 id:pv2
                 TextField{
                     text:move.getPWM2();
                     horizontalAlignment : TextInput.AlignHCenter;
                     //color:'black';
                     font.pointSize: 12;
                     anchors.horizontalCenter: pv2.horizontalCenter;
                     anchors.verticalCenter: pv2.verticalCenter;
                     width: 68 ;
                     height : 48;
                     font.bold: true;
                     id:valuePWM2
                     validator: IntValidator {bottom: -255; top: 255;}
                     focus: true
                     onAccepted: {
                         move.updatePWM2(this.text);
                     }


                     Connections{
                         target: move ;
                         onUpdatedPWM2 :{
                             console.log(121) ;
                             valuePWM2.text=move.getPWM2();
                         }
                     }

                    /* Remote{
                         onUpdatedPWM2: {
                             valuePWM2.text = move.getPWM2();
                             console.log("ccccc\n");
                         }
                     }*/
                 }
             }
         }
        }
    }
}










