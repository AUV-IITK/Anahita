TEMPLATE = app

QT += qml quick core quickwidgets widgets
CONFIG += c++11

SOURCES += main.cpp \
    signalsfromcpp.cpp

RESOURCES += qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH = /home/abhi/Qt5.7.0/5.7/gcc_64/include/QtQml
QML_IMPORT_PATH += /opt/ros/kinetic/bin
# Default rules for deployment.
include(deployment.pri)

DISTFILES += \
    welcome.qml \
    tab2.qml \
    sensor_name.qml \
    sensor_topics.qml \
    sensor_stats.qml \
    monitor.qml \
    nodes.qml \
    node_des1.qml \
    reconfig.qml \
    alpha.qml \
    remote.qml \
    build/Alpha3 \
    auv \
    down.png \
    logo .png \
    up.png \
    README.md \
    logo .png \
    Remote/remote_sway.qml \
    Remote/remote_vrward.qml \
    Remote/remote_forward.qml \
    logo .png \
    remote/remote_forward.qml \
    remote/remote_sway.qml \
    remote/remote_vrward.qml \
    remote/remote_sway.qml.autosave \
    ../../../../../opt/ros/kinetic/bin/binvox2bt \
    ../../../../../opt/ros/kinetic/bin/bt2vrml \
    ../../../../../opt/ros/kinetic/bin/compare_octrees \
    ../../../../../opt/ros/kinetic/bin/convert_octree \
    ../../../../../opt/ros/kinetic/bin/edit_octree \
    ../../../../../opt/ros/kinetic/bin/eval_octree_accuracy \
    ../../../../../opt/ros/kinetic/bin/graph2tree \
    ../../../../../opt/ros/kinetic/bin/log2graph \
    ../../../../../opt/ros/kinetic/bin/opencv_annotation \
    ../../../../../opt/ros/kinetic/bin/opencv_createsamples \
    ../../../../../opt/ros/kinetic/bin/opencv_interactive-calibration \
    ../../../../../opt/ros/kinetic/bin/opencv_traincascade \
    ../../../../../opt/ros/kinetic/bin/opencv_version \
    ../../../../../opt/ros/kinetic/bin/opencv_visualisation \
    ../../../../../opt/ros/kinetic/bin/opencv_waldboost_detector \
    ../../../../../opt/ros/kinetic/bin/rospack \
    ../../../../../opt/ros/kinetic/bin/rosstack \
    ../../../../../opt/ros/kinetic/bin/run_selftest \
    ../../../../../opt/ros/kinetic/bin/rviz \
    ../../../../../opt/ros/kinetic/bin/selftest_example \
    ../../../../../opt/ros/kinetic/bin/selftest_rostest \
    ../../../../../opt/ros/kinetic/bin/stage \
    ../../../../../opt/ros/kinetic/bin/rosrun \
    ../../../../../opt/ros/kinetic/bin/tf_remap \
    ../../../../../opt/ros/kinetic/bin/view_frames \
    ../../../../../opt/ros/kinetic/bin/catkin_create_qt_pkg \
    ../../../../../opt/ros/kinetic/bin/catkin_find \
    ../../../../../opt/ros/kinetic/bin/catkin_init_workspace \
    ../../../../../opt/ros/kinetic/bin/catkin_make \
    ../../../../../opt/ros/kinetic/bin/catkin_make_isolated \
    ../../../../../opt/ros/kinetic/bin/catkin_package_version \
    ../../../../../opt/ros/kinetic/bin/catkin_prepare_release \
    ../../../../../opt/ros/kinetic/bin/catkin_test_results \
    ../../../../../opt/ros/kinetic/bin/catkin_topological_order \
    ../../../../../opt/ros/kinetic/bin/rosbag \
    ../../../../../opt/ros/kinetic/bin/rosboost-cfg \
    ../../../../../opt/ros/kinetic/bin/rosclean \
    ../../../../../opt/ros/kinetic/bin/rosconsole \
    ../../../../../opt/ros/kinetic/bin/roscore \
    ../../../../../opt/ros/kinetic/bin/roscreate-pkg \
    ../../../../../opt/ros/kinetic/bin/rosgraph \
    ../../../../../opt/ros/kinetic/bin/roslaunch \
    ../../../../../opt/ros/kinetic/bin/roslaunch-complete \
    ../../../../../opt/ros/kinetic/bin/roslaunch-deps \
    ../../../../../opt/ros/kinetic/bin/roslaunch-logs \
    ../../../../../opt/ros/kinetic/bin/rosmake \
    ../../../../../opt/ros/kinetic/bin/rosmaster \
    ../../../../../opt/ros/kinetic/bin/rosmsg \
    ../../../../../opt/ros/kinetic/bin/rosmsg-proto \
    ../../../../../opt/ros/kinetic/bin/rosnode \
    ../../../../../opt/ros/kinetic/bin/rosparam \
    ../../../../../opt/ros/kinetic/bin/rosservice \
    ../../../../../opt/ros/kinetic/bin/rossrv \
    ../../../../../opt/ros/kinetic/bin/rostest \
    ../../../../../opt/ros/kinetic/bin/rostopic \
    ../../../../../opt/ros/kinetic/bin/rosunit \
    ../../../../../opt/ros/kinetic/bin/roswtf \
    ../../../../../opt/ros/kinetic/bin/rqt \
    ../../../../../opt/ros/kinetic/bin/rqt_bag \
    ../../../../../opt/ros/kinetic/bin/rqt_console \
    ../../../../../opt/ros/kinetic/bin/rqt_dep \
    ../../../../../opt/ros/kinetic/bin/rqt_graph \
    ../../../../../opt/ros/kinetic/bin/rqt_image_view \
    ../../../../../opt/ros/kinetic/bin/rqt_logger_level \
    ../../../../../opt/ros/kinetic/bin/rqt_plot \
    ../../../../../opt/ros/kinetic/bin/rqt_shell \
    ../../../../../opt/ros/kinetic/bin/xacro

HEADERS += \
    main.h \
    signalsfromcpp.h \
    integrateros.h

