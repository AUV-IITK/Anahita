import QtQuick 2.0

Rectangle{
    height:600;
    width:900;
    id:welcome;
    /*Image{
        source: "https://i0.wp.com/indiannoob.in/wp-content/uploads/2016/07/gow-newomega-final-1-1.jpg";
        width:parent.width;
        height:parent.height;
    }*/
    Column{
        spacing: 40;
        anchors.verticalCenter: welcome.verticalCenter;
        anchors.horizontalCenter: welcome.horizontalCenter;
        Text{
            text:'Hii,  ';
            font.pointSize: 40;
            color: "white";
            font.bold: true;
        }
        Text{
            text:'Wanna see what i can do <br> Choose any of the tabs'
            font.pointSize: 25;
            color: "white";
            font.bold: true;
        }
    }
}
