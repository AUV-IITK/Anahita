# AUV-Qt-Interface
Installation of qt 
--------
- Install the qt creator and qt development tools by following the instructions from [here](https://wiki.qt.io/Install_Qt_5_on_Ubuntu)

Build Instructions
-------
- To build the app , make a build directory, and enter that directory

        mkdir build && cd build
- Run the qmake to build 

        qmake ..
- Run make 

        make
 - Now run the executable by 
 
        ./auv
 

Trouble Shooting
-----
- In case you receive following error in make , or something similiar to that

    
        <QQmlContext> not found
     then it can be resolved by changing the qmake path (from where the qt libraries are included ) and that can be done by .
     
          export PATH="your_path_to_qt/5.7/gcc_64/bin:$PATH"
     replace your_path_to_qt by directory where you installed the qt tool
      
