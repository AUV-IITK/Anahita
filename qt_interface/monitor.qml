import QtQuick 2.0

Rectangle{
    height:600;width:900;
    id:page;
   Column{
       anchors.horizontalCenter: page.horizontalCenter;
       anchors.verticalCenter: page.verticalCenter;
       spacing:20;
       Rectangle{
           width:860;
           height:280;
           border.color:'black';
           border.width: 5;
           id:half1;
           Grid{
               columns:2;
               rows:3;
               rowSpacing:15;
               anchors.horizontalCenter: half1.horizontalCenter;
               anchors.verticalCenter: half1.verticalCenter;
               Rectangle{
                   height:70;
                   width:400;
                   color:'black';
                   border.color: 'black';
                   border.width: 2;
                   id:right1;
                   Text{
                       text:'VECHILE NAME'
                       color:'white';
                       font.bold: true;
                       font.family: 'comic sans ms';
                       font.pointSize: 15;
                       anchors.horizontalCenter: right1.horizontalCenter;
                       anchors.verticalCenter: right1.verticalCenter;


                   }
               }
               Rectangle{
                   height:70;
                   width:400;
                   border.color: 'black';
                   border.width: 2;
                   id:left1;
                   Text{
                       text:'VARUN';
                       font.pointSize: 15;
                       font.family: 'comic sans ms';
                       anchors.horizontalCenter: left1.horizontalCenter;
                       anchors.verticalCenter: left1.verticalCenter;

                   }
               }
               Rectangle{
                   height:70;
                   width:400;
                   color:'black';
                   border.color: 'black';
                   border.width: 2;
                   id:right2;
                   Text{
                       text:'TOTAL TESTING TIME'
                       color:'white';
                       font.bold: true;
                       font.family: 'comic sans ms';
                       font.pointSize: 15;
                       anchors.horizontalCenter: right2.horizontalCenter;
                       anchors.verticalCenter: right2.verticalCenter;


                   }
               }
               Rectangle{
                   height:70;
                   width:400;
                   border.color: 'black';
                   border.width: 2;
                   id:left2;
                   Text{
                       text:'time';
                       font.pointSize: 15;
                       font.family: 'comic sans ms';
                       anchors.horizontalCenter: left2.horizontalCenter;
                       anchors.verticalCenter: left2.verticalCenter;

                   }
               }
               Rectangle{
                   height:70;
                   width:400;
                   color:'black';
                   border.color: 'black';
                   border.width: 2;
                   id:right3;
                   Text{
                       text:'CURRENT TESTING TIME'
                       color:'white';
                       font.bold: true;
                       font.family: 'comic sans ms';
                       font.pointSize: 15;
                       anchors.horizontalCenter: right3.horizontalCenter;
                       anchors.verticalCenter: right3.verticalCenter;


                   }
               }
               Rectangle{
                   height:70;
                   width:400;
                   border.color: 'black';
                   border.width: 2;
                   id:left3;
                   Text{
                       text:'time';
                       font.pointSize: 15;
                       font.family: 'comic sans ms';
                       anchors.horizontalCenter: left3.horizontalCenter;
                       anchors.verticalCenter: left3.verticalCenter;

                   }
               }
           }

       }
       Rectangle{
           width:860;
           height:260;
           border.color:'black';
           border.width: 2.5;
           id:half2;
           Row{
               anchors.horizontalCenter: half2.horizontalCenter;
               anchors.verticalCenter: half2.verticalCenter;
               Rectangle{
                   height:255;width:427.5;
                   border.color:'black';
                   border.width: 2.5;
                   id:bat;
                   Column{
                       anchors.horizontalCenter: bat.horizontalCenter;
                       anchors.verticalCenter: bat.verticalCenter;
                       Rectangle{
                           width:320;
                           height:50;
                           color:'black';
                           id:cover1;
                           Text {
                               font.pointSize: 12;
                               font.bold: true;
                               color:'white';
                               text: 'BATTERY';
                               anchors.horizontalCenter: cover1.horizontalCenter;
                               anchors.verticalCenter: cover1.verticalCenter;
                           }
                       }
                       Rectangle{
                           width:320;
                           height:180;
                           border.color:'black';
                           border.width: 2;
                           id:cover2;
                           //battery plot
                           Rectangle{
                               height:154;
                               width:304;
                               anchors.horizontalCenter: cover2.horizontalCenter;
                               anchors.verticalCenter: cover2.verticalCenter;
                               border.color:'black';
                               border.width:2;
                               //battery

                               Rectangle{
                                   x:2;y:2;
                                   height:150;
                                   width:100; //battery=width/3
                                   color:'lightgreen';
                               }
                           }
                       }
                   }

               }
               //status
               Rectangle{
                   height:255;width:427.5;
                   border.color:'black';
                   border.width: 2.5;
                   id:statusInfo;
                   Column{
                       anchors.horizontalCenter: statusInfo.horizontalCenter;
                       anchors.verticalCenter: statusInfo.verticalCenter;
                       Rectangle{
                           width:320;
                           height:50;
                           color:'black';
                           id:coverstat1;
                           Text {
                               font.pointSize: 12;
                               font.bold: true;
                               color:'white';
                               text: 'STATUS';
                               anchors.horizontalCenter: coverstat1.horizontalCenter;
                               anchors.verticalCenter: coverstat1.verticalCenter;
                           }
                       }
                       Rectangle{
                           width:320;
                           height:180;
                           border.color:'black';
                           border.width: 2;
                           id:coverstat2;
                           color:'#04dc64';
                           //status plot
                           Rectangle{
                               height:154;
                               width:204;
                               anchors.horizontalCenter: coverstat2.horizontalCenter;
                               anchors.verticalCenter: coverstat2.verticalCenter;
                               border.color:'black';
                               border.width:2;
                               id:s1;
                               //status
                               Image {
                                   id: status;
                                   height:140;
                                   width: 140;

                                   source: "https://healthycities.zendesk.com/hc/en-us/article_attachments/209680088/Thumbs_up_icon_-_500px.png";
                                   anchors.horizontalCenter: s1.horizontalCenter;
                                   anchors.verticalCenter: s1.verticalCenter;

                               }


                           }
                       }
                   }

               }

           }


       }
   }
}






