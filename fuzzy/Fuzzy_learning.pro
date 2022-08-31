TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
COMFIG -= qt

HOME_DIR = $$system(echo $HOME)

INCLUDEPATH += \
            $$HOME_DIR/.aubo/include \
            /usr/include/eigen3

LIBS += -L$$HOME_DIR/.aubo/lib -laubo_driver -lpthread -ldl

HEADERS += robot.h\
	robot_math.h\
	ATI_UDP.h\
	Fuzzy.h\
	fuzzy_learning.h\

SOURCES += main.cpp\
	robot.cpp\
	robot_math.cpp\
	ATI_UDP.cpp\
	Fuzzy.cpp\
	fuzzy_learning.cpp\
	

