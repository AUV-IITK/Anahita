import QtQuick 2.0

Rectangle{
    id:page;
    height:450;
    width:100;

    border.color:"black";
    border.width: 2;
    Column{
        //sensor1
        anchors.horizontalCenter: page.horizontalCenter;
        anchors.verticalCenter: page.verticalCenter;
        Rectangle{
            id:sen1
            height:64;
            width:100;
            border.width:1;
            border.color:'black';
            Rectangle {
                id: sensor1
                height:30;
                width:30;
                border.width: 2;
                border.color:'black';
                radius: 15;
                color:'green';
                anchors.horizontalCenter: sen1.horizontalCenter;
                anchors.verticalCenter: sen1.verticalCenter;

            }
        }

        //sensor2
        Rectangle{
            id:sen2;
            height:64;
            width:100;
            border.width:1;
            border.color:'black';
            Rectangle {
                id: sensor2
                height:30;
                width:30;
                border.width: 2;
                border.color:'black';
                radius: 15;
                color:'green';
                anchors.horizontalCenter: sen2.horizontalCenter;
                anchors.verticalCenter: sen2.verticalCenter;
            }
        }

        //sensor3
        Rectangle{
            id:sen3
            height:64;
            width:100;
            border.width:1;
            border.color:'black';
            Rectangle {
                id: sensor3
                height:30;
                width:30;
                border.width: 2;
                border.color:'black';
                radius: 15;
                color:'green';
                anchors.horizontalCenter: sen3.horizontalCenter;
                anchors.verticalCenter: sen3.verticalCenter;
            }
        }

        //sensor4
        Rectangle{
            id:sen4
            height:64;
            width:100;
            border.width:1;
            border.color:'black';
            Rectangle {
                id: sensor4
                height:30;
                width:30;
                border.width: 2;
                border.color:'black';
                radius: 15;
                color:'green';
                anchors.horizontalCenter: sen4.horizontalCenter;
                anchors.verticalCenter: sen4.verticalCenter;

            }
        }

        //sensor5
        Rectangle{
            id:sen5
            height:64;
            width:100;
            border.width:1;
            border.color:'black';
            Rectangle {
                id: sensor5
                height:30;
                width:30;
                border.width: 2;
                border.color:'black';
                radius: 15;
                color:'green';
                anchors.horizontalCenter: sen5.horizontalCenter;
                anchors.verticalCenter: sen5.verticalCenter;
            }
        }

        //sensor6
        Rectangle{
            id:sen6
            height:64;
            width:100;
            border.width:1;
            Rectangle {
                id: sensor6
                height:30;
                width:30;
                border.width: 2;
                border.color:'black';
                radius: 15;
                color:'red';
                anchors.horizontalCenter: sen6.horizontalCenter;
                anchors.verticalCenter: sen6.verticalCenter;
            }
        }

        //sensor7
        Rectangle{
            id:sen7
            height:64;
            width:100;
            border.width:1;

            border.color:'black';
            Rectangle {
                id: sensor7
                height:30;
                width:30;
                border.width: 2;
                border.color:'black';
                radius: 15;
                color:'green';
                anchors.horizontalCenter: sen7.horizontalCenter;
                anchors.verticalCenter: sen7.verticalCenter;
            }

        }
    }

}
