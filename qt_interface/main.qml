import QtQuick 2.0
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4


Window{
    width:1000;
    height:850;
    visible:true;
    Rectangle{
        id: page
        width: 1000; height: 850;
        color: "#404144"

       /* Text {
            id: helloText
            text: "Varun"
            y: 0
            anchors.horizontalCenter: page.horizontalCenter
            font.pointSize: 24; font.bold: true
        }
        */
        State { name: 'loaded'; when: logo.status == Image.Ready }


        Image {
            id: logo
            source: "http://auviitk.com/images/logo.jpg"
            y:20;
            cache: false;
            onStatusChanged: if (logo.status == Image.Ready) console.log('')

            anchors.horizontalCenter: page.horizontalCenter

        }
        Text { text: logo.status == Image.Ready ? '' : 'AUV-IITK'; font.family: "comic sans ms"; font.pointSize:80 ;height:40; color: "#ffffff";y:20;
            anchors.horizontalCenter: page.horizontalCenter

        }

        TabView {
            y:200;
            anchors.horizontalCenter: page.horizontalCenter;
            height:600;
            width:900;
            id:frame
            Tab {
                title: "WELCOME"
                Loader  {  source:"welcome.qml" }
            }
          Tab {
                title: "SENSORS"
                Loader  { source:"sensors.qml" }
            }
            Tab {
                title: "NODES"
                Loader  { source:"nodes.qml" }
            }
            Tab {
                title: "REMOTE CONTROL"
                Loader  { source:"remote.qml" }
            }
            Tab {
                title: "RECONFIG"
                Loader  { source:"reconfig.qml" }
            }
            Tab {
                title: "DATA LOG"
                Loader  { source:"data_log.qml" }
            }
            Tab {
                title: "HEALTH MONITOR"
                Loader  { source:"monitor.qml" }

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
                frame: Rectangle { color: "white" }
            }





        }



      }
}



