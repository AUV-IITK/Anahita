import QtQuick 2.0
import QtQuick.Controls 1.4
Rectangle {
    height:440;
    width:850;
    Column{
        spacing:0;
        Rectangle{
            width:850;
            height:370;
            id:nodeCover;
            Row{
                spacing:0;
                anchors.horizontalCenter: nodeCover.horizontalCenter;
                anchors.verticalCenter: nodeCover.verticalCenter;

                Rectangle{
                    height:350;
                    width:500;
                    border.color: 'black';
                    border.width:2.5;
                    id:nodeCoverIn;

                    Rectangle{
                        height: 330;
                        width:480;
                        border.width:2.5;
                        border.color: 'black';
                        anchors.horizontalCenter: nodeCoverIn.horizontalCenter;
                        anchors.verticalCenter: nodeCoverIn.verticalCenter;
                        id:nodeHead;
                        Column{
                            anchors.horizontalCenter: nodeHead.horizontalCenter;
                            anchors.verticalCenter: nodeHead.verticalCenter;
                            spacing:0;
                            Rectangle{
                                height:55;
                                width:475;

                                Row{
                                    spacing:1;
                                    Rectangle{
                                        height:55;
                                        width:350;
                                        color:'black';
                                        id:nodeNameHead;
                                        Text{
                                            font.pointSize: 12;
                                            text:"NODE NAME";
                                            anchors.horizontalCenter: nodeNameHead.horizontalCenter;
                                            anchors.verticalCenter: nodeNameHead.verticalCenter;
                                            color:'white';
                                        }
                                    }
                                    Rectangle{
                                        height:55;
                                        width:124;
                                        color:'black';
                                        id:nodeStatusHead;
                                        Text{
                                            font.pointSize: 12;
                                            text:"STATUS";
                                            anchors.horizontalCenter: nodeStatusHead.horizontalCenter;
                                            anchors.verticalCenter: nodeStatusHead.verticalCenter;
                                            color:'white';
                                        }
                                    }
                                }

                            }
                            Rectangle{
                                height:270;
                                width:475;
                                ScrollView{
                                    width:475;
                                    height: 270;
                                    id:scrolls;
                                    verticalScrollBarPolicy: Qt.ScrollBarAlwaysOff;
                                    horizontalScrollBarPolicy: Qt.ScrollBarAlwaysOff;
                                    Grid{
                                        y:2;
                                        columns:2;
                                        rowSpacing: 2;

////Node 1 info block
                                        Rectangle{

                                            height:70;
                                            width:350;
                                            border.color: 'black';
                                            border.width:1.5;
                                            id:nodeName1;
                                            Text{
                                                text:"Node 1";
                                                font.pointSize: 15;
                                                anchors.horizontalCenter: nodeName1.horizontalCenter;
                                                anchors.verticalCenter: nodeName1.verticalCenter;

                                            }
                                        }
                                        Rectangle{

                                            height:70;
                                            width:125;
                                            border.color:'black';
                                            border.width:1.5;
                                            id:nodeStatus1
                                            Rectangle{
                                                height:50;
                                                width:50;
                                                border.color:'black';
                                                border.width: 3;
                                                color:'red';
                                                anchors.horizontalCenter: nodeStatus1.horizontalCenter;
                                                anchors.verticalCenter: nodeStatus1.verticalCenter;
                                                MouseArea{
                                                    anchors.fill: parent
                                                     onClicked: {
                                                         // console.log("111")
                                                     }
                                                }

                                            }
//                                            MouseArea{
//
//                                            }

                                        }
////node 2 block info
                                        Rectangle{

                                            height:70;
                                            width:350;
                                            border.color: 'black';
                                            border.width:1.5;
                                            id:nodeName2;
                                            Text{
                                                text:"Node 2";
                                                font.pointSize: 15;
                                                anchors.horizontalCenter: nodeName2.horizontalCenter;
                                                anchors.verticalCenter: nodeName2.verticalCenter;

                                            }
                                        }
                                        Rectangle{

                                            height:70;
                                            width:125;
                                            border.color:'black';
                                            border.width:1.5;
                                            id:nodeStatus2
                                            Rectangle{
                                                height:50;
                                                width:50;
                                                border.color:'black';
                                                border.width: 3;
                                                color:'red';
                                                anchors.horizontalCenter: nodeStatus2.horizontalCenter;
                                                anchors.verticalCenter: nodeStatus2.verticalCenter;

                                            }

                                        }


///Node 3 block info
                                        Rectangle{

                                            height:70;
                                            width:350;
                                            border.color: 'black';
                                            border.width:1.5;
                                            id:nodeName3;
                                            Text{
                                                text:"Node 3";
                                                font.pointSize: 15;
                                                anchors.horizontalCenter: nodeName3.horizontalCenter;
                                                anchors.verticalCenter: nodeName3.verticalCenter;

                                            }
                                        }
                                        Rectangle{

                                            height:70;
                                            width:125;
                                            border.color:'black';
                                            border.width:1.5;
                                            id:nodeStatus3
                                            Rectangle{
                                                height:50;
                                                width:50;
                                                border.color:'black';
                                                border.width: 3;
                                                color:'#04dc64';
                                                anchors.horizontalCenter: nodeStatus3.horizontalCenter;
                                                anchors.verticalCenter: nodeStatus3.verticalCenter;

                                            }

                                        }
////Node 4 block info
                                        Rectangle{

                                            height:70;
                                            width:350;
                                            border.color: 'black';
                                            border.width:1.5;
                                            id:nodeName4;
                                            Text{
                                                text:"Node 4";
                                                font.pointSize: 15;
                                                anchors.horizontalCenter: nodeName4.horizontalCenter;
                                                anchors.verticalCenter: nodeName4.verticalCenter;

                                            }
                                        }
                                        Rectangle{

                                            height:70;
                                            width:125;
                                            border.color:'black';
                                            border.width:1.5;
                                            id:nodeStatus4
                                            Rectangle{
                                                height:50;
                                                width:50;
                                                border.color:'black';
                                                border.width: 3;
                                                color:'red';
                                                anchors.horizontalCenter: nodeStatus4.horizontalCenter;
                                                anchors.verticalCenter: nodeStatus4.verticalCenter;

                                            }

                                        }

/////Node 5 block info

                                        Rectangle{

                                            height:70;
                                            width:350;
                                            border.color: 'black';
                                            border.width:1.5;
                                            id:nodeName5;
                                            Text{
                                                text:"Node 5";
                                                font.pointSize: 15;
                                                anchors.horizontalCenter: nodeName5.horizontalCenter;
                                                anchors.verticalCenter: nodeName5.verticalCenter;

                                            }
                                        }
                                        Rectangle{

                                            height:70;
                                            width:125;
                                            border.color:'black';
                                            border.width:1.5;
                                            id:nodeStatus5
                                            Rectangle{
                                                height:50;
                                                width:50;
                                                border.color:'black';
                                                border.width: 3;
                                                color:'red';
                                                anchors.horizontalCenter: nodeStatus5.horizontalCenter;
                                                anchors.verticalCenter: nodeStatus5.verticalCenter;

                                            }

                                        }

/*********************************Nodes Done*******************/
                                    }
                                }

                            }
                        }



                    }
                }
                Rectangle{
                    height:350;
                    width:300;
                    border.color:'black';
                    border.width:2.5;
                    id:description;
                    Column{
                        anchors.horizontalCenter: description.horizontalCenter;
                        anchors.verticalCenter: description.verticalCenter;

                        spacing:0;
                        //description heding
                        Rectangle{
                            height:60;
                            color:'black';
                            width:295;
                            id:desHead;
                            Text{
                                text:'Description';
                                anchors.horizontalCenter: desHead.horizontalCenter;
                                anchors.verticalCenter: desHead.verticalCenter;
                                color:'white';
                                font.pointSize: 15;
                            }
                        }
                        //description for node
                        Rectangle{
                            height:285;
                            color:'light grey';
                            width:295;
                            id:decCover;
                            Text{
                                text:'node description '
                                anchors.horizontalCenter: decCover.horizontalCenter;
                                anchors.verticalCenter: decCover.verticalCenter;
                                id:decContent;
                            }

                        }
                    }

                }
            }
        }
        //description for package
        Rectangle{
            width:850;
            color:'black';
            height:70;
            id:packDes;
            Text{
                color:'white';
                anchors.horizontalCenter: packDes.horizontalCenter;
                anchors.verticalCenter: packDes.verticalCenter;
                text:'package 1 description';

            }
        }
    }

}
