# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include".split(';') if "${prefix}/include;/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;hector_uav_msgs;roscpp".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lhector_quadrotor_propulsion;-lhector_quadrotor_aerodynamics;/usr/local/lib/libboost_thread.so;/usr/local/lib/libboost_chrono.so;/usr/local/lib/libboost_system.so;/usr/local/lib/libboost_date_time.so;/usr/local/lib/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so".split(';') if "-lhector_quadrotor_propulsion;-lhector_quadrotor_aerodynamics;/usr/local/lib/libboost_thread.so;/usr/local/lib/libboost_chrono.so;/usr/local/lib/libboost_system.so;/usr/local/lib/libboost_date_time.so;/usr/local/lib/libboost_atomic.so;/usr/lib/x86_64-linux-gnu/libpthread.so" != "" else []
PROJECT_NAME = "hector_quadrotor_model"
PROJECT_SPACE_DIR = "/home/liu/Multi_Robot_System/install_isolated"
PROJECT_VERSION = "0.3.5"
