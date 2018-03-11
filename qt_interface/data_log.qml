import QtQuick 2.0
import QtQuick.Controls 1.4
Rectangle{
    height:600;width:900;
    id:page;
   Column{
       anchors.horizontalCenter: page.horizontalCenter;
       anchors.verticalCenter: page.verticalCenter;
       spacing:20;
       Rectangle{
           width:860;
           height:160;
           border.color:'black';
           border.width: 5;
           id:pack;
           Column{
               spacing: 20
               anchors.horizontalCenter: pack.horizontalCenter;
               anchors.verticalCenter: pack.verticalCenter;


               Row{
                   Rectangle{
                       height:50;
                       width:400;
                       color:'black';
                       border.color: 'black';
                       border.width: 2;
                       id:right1;
                       Text{
                           text:' TOPIC NAME'
                           color:'white';
                           font.bold: true;
                           font.family: 'comic sans ms';
                           font.pointSize: 15;
                           anchors.horizontalCenter: right1.horizontalCenter;
                           anchors.verticalCenter: right1.verticalCenter;


                       }
                   }
                   Rectangle{
                       height:50;
                       width:400;
                       border.color: 'black';
                       border.width: 2;
                       id:left1;
                       ComboBox{
                           model: [ "Topic 1", "Topic 2", "Topic 3" ];
                           width:350;
                           height:35;
                           anchors.horizontalCenter: left1.horizontalCenter;
                           anchors.verticalCenter: left1.verticalCenter;

                       }
                   }
               }


               Row{
                   Rectangle{
                       height:50;
                       width:400;
                       color:'black';
                       border.color: 'black';
                       border.width: 2;
                       id:nodeRight;
                       Text{
                           text:' PACKAGE NAME'
                           color:'white';
                           font.bold: true;
                           font.family: 'comic sans ms';
                           font.pointSize: 15;
                           anchors.horizontalCenter: nodeRight.horizontalCenter;
                           anchors.verticalCenter: nodeRight.verticalCenter;


                       }
                   }
                   Rectangle{
                       height:50;
                       width:400;
                       border.color: 'black';
                       border.width: 2;
                       id:nodeBox;
                       ComboBox{
                           model: [ "Package 1", "Package 2", "Package 3" ];
                           width:350;
                           height:35;
                           anchors.horizontalCenter: nodeBox.horizontalCenter;
                           anchors.verticalCenter: nodeBox.verticalCenter;

                       }
                   }
               }


           }

       }
       Rectangle{
           width:860;
           height:380;
           border.color:'black';
           border.width: 5;
           id:half2;
           Column{
               anchors.centerIn: parent;
               spacing: 15;


               Row{
                   Rectangle{
                       height:50;
                       width:400;
                       color:'black';
                       border.color: 'black';
                       border.width: 2;
                       Text{
                           text:' Sensor Info'
                           color:'white';
                           font.bold: true;
                           font.family: 'comic sans ms';
                           font.pointSize: 15;
                           anchors.centerIn: parent

                       }
                   }
                   Rectangle{
                       height:50;
                       width:400;
                       border.color: 'black';
                       border.width: 2;
                       Text{
                           text:' *****'
                           color:'black';
                           font.bold: true;
                           font.family: 'comic sans ms';
                           font.pointSize: 15;
                           anchors.centerIn: parent

                       }

                   }
               }



               Row{
                   Rectangle{
                       height:50;
                       width:400;
                       color:'black';
                       border.color: 'black';
                       border.width: 2;
                       Text{
                           text:' ****'
                           color:'white';
                           font.bold: true;
                           font.family: 'comic sans ms';
                           font.pointSize: 15;
                           anchors.centerIn: parent

                       }
                   }
                   Rectangle{
                       height:50;
                       width:400;
                       border.color: 'black';
                       border.width: 2;
                       Text{
                           text:' ****'
                           color:'black';
                           font.bold: true;
                           font.family: 'comic sans ms';
                           font.pointSize: 15;
                           anchors.centerIn: parent

                       }

                   }
               }


               Row{

                   spacing:40;
                   Rectangle{
                       width:380;
                       height:200;
                       border.width: 3;
                       border.color: "black";
                       color:'grey';
                       Text{
                           anchors.centerIn: parent;
                           text:'description <br> description';
                           color:'white';
                           font.pointSize: 11;
                       }
                   }

                   Rectangle{
                       width:380;
                       height:200;
                       border.width: 3;
                       border.color: "black";
                       color:'grey'
                       Text{
                           anchors.centerIn: parent;
                           text:'description <br> description';
                           color:'white';
                           font.pointSize: 11;
                       }
                   }
               }
           }
       }
   }
}






