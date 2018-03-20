import QtQuick 2.0

Rectangle{
    id:page;
    height:450;
    width:350;
    color:'red';
    border.color:"black";
    border.width: 2;
    Column{
        //sensor1
        anchors.horizontalCenter: page.horizontalCenter;
        anchors.verticalCenter: page.verticalCenter;
        Rectangle{
            id:sen1
            height:64;
            width:350;
            border.width:1;
            border.color:'black';
            Text {
                id: sensor1
                text:'Sensor 1';
                font.pointSize: 15;
                anchors.horizontalCenter: sen1.horizontalCenter;
                anchors.verticalCenter: sen1.verticalCenter;
            }
        }

        //sensor2
        Rectangle{
            id:sen2;
            height:64;
            width:350;
            border.width:1;
            border.color:'black';
            Text {
                id: sensor2;
                text:'Sensor 2';
                font.pointSize: 15;
                anchors.horizontalCenter: sen2.horizontalCenter;
                anchors.verticalCenter: sen2.verticalCenter;
            }
        }

        //sensor1
        Rectangle{
            id:sen3
            height:64;
            width:350;
            border.width:1;
            border.color:'black';
            Text {
                id: sensor3
                text:'Sensor 3';
                font.pointSize: 15;
                anchors.horizontalCenter: sen3.horizontalCenter;
                anchors.verticalCenter: sen3.verticalCenter;
            }
        }

        //sensor4
        Rectangle{
            id:sen4
            height:64;
            width:350;
            border.width:1;
            border.color:'black';
            Text {
                id: sensor4
                text:'Sensor 4';
                font.pointSize: 15;
                anchors.horizontalCenter: sen4.horizontalCenter;
                anchors.verticalCenter: sen4.verticalCenter;
            }
        }

        //sensor5
        Rectangle{
            id:sen5
            height:64;
            width:350;
            border.width:1;
            border.color:'black';
            Text {
                id: sensor5
                text:'Sensor 5';
                font.pointSize: 15;
                anchors.horizontalCenter: sen5.horizontalCenter;
                anchors.verticalCenter: sen5.verticalCenter;
            }
        }

        //sensor6
        Rectangle{
            id:sen6
            height:64;
            width:350;
            border.width:1;
            border.color:'black';
            Text {
                id: sensor6
                text:'Sensor 6';
                font.pointSize: 15;
                anchors.horizontalCenter: sen6.horizontalCenter;
                anchors.verticalCenter: sen6.verticalCenter;
            }
        }

        //sensor7
        Rectangle{
            id:sen7
            height:64;
            width:350;
            border.width:1;

            border.color:'black';
            Text {
                id: sensor7
                text:'Sensor 7';
                font.pointSize: 15;
                anchors.horizontalCenter: sen7.horizontalCenter;
                anchors.verticalCenter: sen7.verticalCenter;
            }
        }
    }

}
