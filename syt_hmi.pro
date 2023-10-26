#-------------------------------------------------
#
# Copyright (C) 2003-2103 CamelSoft Corporation
#
#-------------------------------------------------

TEMPLATE = app
CONFIG += c++14
QT       += core gui widgets

SOURCES += src/main.cpp \
           src/syt_btn/appendbutton.cpp \
           src/syt_btn/generalbuttoninterface.cpp \
           src/syt_btn/infobutton.cpp \
           src/syt_btn/interactivebuttonbase.cpp \
           src/syt_btn/pointmenubutton.cpp \
           src/syt_btn/threedimenbutton.cpp \
           src/syt_btn/watercirclebutton.cpp \
           src/syt_btn/waterfallbuttongroup.cpp \
           src/syt_btn/waterfloatbutton.cpp \
           src/syt_btn/waterzoombutton.cpp \
           src/syt_btn/winclosebutton.cpp \
           src/syt_btn/winmaxbutton.cpp \
           src/syt_btn/winmenubutton.cpp \
           src/syt_btn/winminbutton.cpp \
           src/syt_btn/winrestorebutton.cpp \
           src/syt_btn/winsidebarbutton.cpp \
           src/syt_btn/appendbutton.cpp \
           src/syt_btn/appendbutton.cpp \
           src/syt_edit/bottomlineedit.cpp \
           src/syt_edit/labelededit.cpp \
           src/syt_hmi/auto_create_wizard.cpp \
           src/syt_hmi/choose_style_dialog.cpp \
           src/syt_hmi/cloth_style_dialog.cpp \
           src/syt_hmi/create_from_cad_wizard.cpp \
           src/syt_hmi/create_from_source_wizard.cpp \
           src/syt_hmi/developer_widget.cpp \
           src/syt_hmi/dev_login_window.cpp \
           src/syt_hmi/hand_eye_dialog.cpp \
           src/syt_hmi/image_item.cpp \
           src/syt_hmi/input_extra_param_widget.cpp \
           src/syt_hmi/input_length_param_widget.cpp \
           src/syt_hmi/input_tolerance_param_widget.cpp \
           src/syt_hmi/lock_dialog.cpp \
           src/syt_hmi/main_window.cpp \
           src/syt_hmi/manual_input_param_wizard.cpp \
           src/syt_hmi/ota_update_dialog.cpp \
           src/syt_hmi/progress_bar.cpp \
           src/syt_hmi/show_color_widget.cpp \
           src/syt_hmi/style_display_widget.cpp \
           src/syt_hmi/translate_dialog.cpp \
           src/syt_hmi/wizard_pages.cpp \
           src/syt_hmi/param_set_widget.cpp \
           src/syt_number/led_number.cpp \
           src/syt_plaintextedit/filtertextedit.cpp \
           src/syt_process/roundprogressbar.cpp \
           src/syt_process/waterprocess.cpp \
           src/syt_rclcomm/rcl_comm.cpp \
           src/utils/utils.cpp \
           src/utils/waitingspinnerwidget.cpp


INCLUDEPATH += include \
               /usr/include/opencv4 \
               /home/syt/thanos_pre_release/install/include
               /opt/ros/foxy/include

LIBS += /usr/lib/x86_64-linux-gnu/libopencv_*.a

HEADERS  += include/syt_btn/appendbutton.h \
            include/syt_btn/generalbuttoninterface.h \
            include/syt_btn/infobutton.h \
            include/syt_btn/interactivebuttonbase.h \
            include/syt_btn/pointmenubutton.h \
            include/syt_btn/threedimenbutton.h \
            include/syt_btn/watercirclebutton.h \
            include/syt_btn/waterfallbuttongroup.h \
            include/syt_btn/waterfloatbutton.h \
            include/syt_btn/waterzoombutton.h \
            include/syt_btn/winclosebutton.h \
            include/syt_btn/winmaxbutton.h \
            include/syt_btn/winmenubutton.h \
            include/syt_btn/winminbutton.h \
            include/syt_btn/winrestorebutton.h \
            include/syt_btn/winsidebarbutton.h \
            include/syt_btn/appendbutton.h \
            include/syt_btn/appendbutton.h \
            include/syt_edit/bottomlineedit.h \
            include/syt_edit/labelededit.h \
            include/syt_hmi/auto_create_wizard.h \
            include/syt_hmi/choose_style_dialog.h \
            include/syt_hmi/cloth_style_dialog.h \
            include/syt_hmi/create_from_cad_wizard.h \
            include/syt_hmi/create_from_source_wizard.h \
            include/syt_hmi/developer_widget.h \
            include/syt_hmi/dev_login_window.h \
            include/syt_hmi/hand_eye_dialog.h \
            include/syt_hmi/image_item.h \
            include/syt_hmi/input_extra_param_widget.h \
            include/syt_hmi/input_length_param_widget.h \
            include/syt_hmi/input_tolerance_param_widget.h \
            include/syt_hmi/lock_dialog.h \
            include/syt_hmi/main_window.h \
            include/syt_hmi/manual_input_param_wizard.h \
            include/syt_hmi/ota_update_dialog.h \
            include/syt_hmi/progress_bar.h \
            include/syt_hmi/show_color_widget.h \
            include/syt_hmi/style_display_widget.h \
            include/syt_hmi/translate_dialog.h \
            include/syt_hmi/wizard_pages.h \
            include/syt_hmi/param_set_widget.h \
            include/syt_number/led_number.h \
            include/syt_plaintextedit/filtertextedit.h \
            include/syt_process/roundprogressbar.h \
            include/syt_process/waterprocess.h \
            include/syt_rclcomm/rcl_comm.h \
            include/utils/utils.h \
            include/utils/waitingspinnerwidget.h

FORMS    += \
            ui/choose_style_dialog.ui \
            ui/cloth_style_dialog.ui \
            ui/developer_widget.ui \
            ui/dev_login_window.ui \
            ui/hand_eye_dialog.ui \
            ui/input_extra_param_widget.ui \
            ui/input_length_param_widget.ui \
            ui/input_tolerance_param_widget.ui \
            ui/lock_dialog.ui \
            ui/main_window.ui \
            ui/ota_update_dialog.ui \
            ui/param_set_widget.ui \
            ui/progress_bar.ui \
            ui/style_display_widget.ui \
            ui/translate_dialog.ui \

TRANSLATIONS = zh_CN.ts en_US.ts

RESOURCES += \
             resource/hmi.qrc
