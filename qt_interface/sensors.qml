import QtQuick 2.0

Rectangle{
    id:page;
    width:900;
    height:600;

    Rectangle{
        id:frame;
        anchors.horizontalCenter: page.horizontalCenter;
        anchors.verticalCenter: page.verticalCenter;
        border.color:" black";
        border.width: 5;
        width:860;
        height:550;
        Grid{

            columnSpacing:10;
            columns:3;
            rows:2;
            anchors.horizontalCenter: frame.horizontalCenter;
            anchors.verticalCenter: frame.verticalCenter;
            Rectangle{
                color:"black";
                id:head1;
                height:50;
                width:350;
                Text {

                    text: qsTr("SENSORS");
                    font.bold: true;
                    font.pointSize: 15;
                    color:"white";
                    anchors.horizontalCenter: head1.horizontalCenter;
                    anchors.verticalCenter: head1.verticalCenter;

                }
            }
            Rectangle{
                color:"black";
                id:head2;
                height:50;
                width:100;
                Text {

                    text: qsTr("STATS");
                    font.bold: true;
                    font.pointSize: 15;
                    color:"white";
                    anchors.horizontalCenter: head2.horizontalCenter;
                    anchors.verticalCenter: head2.verticalCenter;

                }
            }
            Rectangle{
                height:50;
                id:head3;
                color:"black";
                width:350;
                Text {

                    text: qsTr("TOPICS");
                    font.bold: true;
                    font.pointSize: 15;
                    color:"white";
                    anchors.horizontalCenter: head3.horizontalCenter;
                    anchors.verticalCenter: head3.verticalCenter;

                }

            }
            Rectangle{
                height:450;
                border.color:"black";
                border.width: 2;
                width:350;
                Loader{
                    source:"sensor_name.qml";
                }

            }
            Rectangle{
                height:450;
                border.color:"black";
                border.width: 2;
                width:100;
                Loader{
                    source:"sensor_stats.qml";
                }

            }
            Rectangle{
                height:450;
                border.color:"black";
                border.width: 2;
                width:350;
                Loader{
                    source:"sensor_topics.qml";
                }

            }
    }
}
}
