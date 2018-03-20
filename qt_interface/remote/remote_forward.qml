import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import Motion.header 1.0
Rectangle {

    //signal someClicked()
    property int pWM3 : 0
    property int pWM4: 0

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
                 title: 'T-3'
                 Loader{
                     source: 'thrust3.qml'

                 }
             }
             Tab{
                 title: 'T-4'
                 Loader{
                     source: 'thrust4.qml'
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
                 id:p3;
                 Text{
                     text:'PWM 3';
                     color:'white';
                     font.pointSize: 12;
                     anchors.horizontalCenter: p3.horizontalCenter;
                     anchors.verticalCenter: p3.verticalCenter;
                     font.bold: true;

                 }
             }
             Rectangle{
                 height:50;
                 width:70;
                 border.color:'black';
                 id:pv3
                 TextField{
                     text:move.getPWM3();
                     horizontalAlignment : TextInput.AlignHCenter;
                     //color:'black';
                     font.pointSize: 12;
                     anchors.horizontalCenter: pv3.horizontalCenter;
                     anchors.verticalCenter: pv3.verticalCenter;
                     width: 68 ;
                     height : 48;
                     font.bold: true;
                     id:valuePWM3
                     validator: IntValidator {bottom: -255; top: 255;}
                     focus: true
                     onAccepted: {
                         move.updatePWM3(this.text);
                     }


                     Connections{
                         target: move ;
                         onUpdatedPWM3 :{
                             console.log(121) ;
                             valuePWM3.text=move.getPWM3();
                         }
                     }

                 }


             }


             //PWM 2
             Rectangle{
                 height:50;
                 width:130;
                 color:'black';
                 id:p4;
                 Text{
                     text:'PWM 4';
                     color:'white';
                     font.pointSize: 12;
                     anchors.horizontalCenter: p4.horizontalCenter;
                     anchors.verticalCenter: p4.verticalCenter;
                     font.bold: true;

                 }
             }
             Rectangle{
                 height:50;
                 width:70;
                 border.color:'black';
                 id:pv4
                 TextField{
                     text:move.getPWM4();
                     horizontalAlignment : TextInput.AlignHCenter;
                     //color:'black';
                     font.pointSize: 12;
                     anchors.horizontalCenter: pv4.horizontalCenter;
                     anchors.verticalCenter: pv4.verticalCenter;
                     width: 68 ;
                     height : 48;
                     font.bold: true;
                     id:valuePWM4
                     validator: IntValidator {bottom: -255; top: 255;}
                     focus: true
                     onAccepted: {
                         move.updatePWM4(this.text);
                     }


                     Connections{
                         target: move ;
                         onUpdatedPWM4 :{
                             console.log(121) ;
                             valuePWM4.text=move.getPWM4();
                         }
                     }

                 }
             }
         }
        }
    }

}




/*




*/




