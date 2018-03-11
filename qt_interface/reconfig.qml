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
           height:100;
           border.color:'black';
           border.width: 5;
           id:pack;
           Row{
               anchors.horizontalCenter: pack.horizontalCenter;
               anchors.verticalCenter: pack.verticalCenter;
               Rectangle{
                   height:60;
                   width:400;
                   color:'black';
                   border.color: 'black';
                   border.width: 2;
                   id:right1;
                   Text{
                       text:' PACKAGE'
                       color:'white';
                       font.bold: true;
                       font.family: 'comic sans ms';
                       font.pointSize: 15;
                       anchors.horizontalCenter: right1.horizontalCenter;
                       anchors.verticalCenter: right1.verticalCenter;


                   }
               }
               Rectangle{
                   height:60;
                   width:400;
                   border.color: 'black';
                   border.width: 2;
                   id:left1;
                   ComboBox{
                       model: [ "Package 1", "Package 2", "Package 3" ];
                       width:350;
                       height:40;
                       anchors.horizontalCenter: left1.horizontalCenter;
                       anchors.verticalCenter: left1.verticalCenter;

                   }
               }
           }

       }
       Rectangle{
           width:860;
           height:440;
           border.color:'black';
           border.width: 5;
           id:half2;
           Column{
               anchors.horizontalCenter: half2.horizontalCenter;
               anchors.verticalCenter: half2.verticalCenter;
               spacing: 20;
               Row{
                   Rectangle{
                       height:60;
                       width:400;
                       color:'black';
                       border.color: 'black';
                       border.width: 2;
                       id:nodeRight;
                       Text{
                           text:' NODE'
                           color:'white';
                           font.bold: true;
                           font.family: 'comic sans ms';
                           font.pointSize: 15;
                           anchors.horizontalCenter: nodeRight.horizontalCenter;
                           anchors.verticalCenter: nodeRight.verticalCenter;


                       }
                   }
                   Rectangle{
                       height:60;
                       width:400;
                       border.color: 'black';
                       border.width: 2;
                       id:nodeBox;
                       ComboBox{
                           model: [ "Node 1", "Node 2", "Node 3" ];
                           width:350;
                           height:40;
                           anchors.horizontalCenter: nodeBox.horizontalCenter;
                           anchors.verticalCenter: nodeBox.verticalCenter;

                       }
                   }
               }
               Rectangle{
                   width:800;
                   height:310;
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
           }
       }
   }
}






