QT += core gui opengl

TARGET = bulletdice
TEMPLATE = app

VERSION = 1.1

include(bullet/bullet.pri)

RESOURCES += \
    data/texture.qrc


SOURCES += \
    Mesh.cpp \
    main.cpp \
    PhysicsWidget.cpp

HEADERS  += \
    Mesh.h \
    PhysicsWidget.h

win32 {
    RC_FILE = bulletdice.rc
}


unix:!symbian {
    # Harmattan specific

    CONFIG += mobility
    MOBILITY += sensors

    BINDIR    = /opt/usr/bin
    DATADIR   = /usr/share

    DEFINES  += DATADIR=\\\"$$DATADIR\\\" \
                PKGDATADIR=\\\"$$PKGDATADIR\\\" \
                Q_WS_HARMATTAN

    target.path = $$BINDIR

    icon64.path = $$DATADIR/icons/hicolor/64x64/apps
    icon64.files += icons/80x80/bulletdice_80x80.png

    desktop.path = $$DATADIR/applications
    desktop.files += qtc_packaging/debian_harmattan/bulletdice.desktop

    INSTALLS += target \
                desktop \
                icon64
}


symbian {
    ICON = icons/bulletdice.svg

    TARGET = BulletDice
    TARGET.UID3 = 0xe079010c

    # Bigger heap and stack are needed for Bullet and graphics
    TARGET.EPOCHEAPSIZE = 0x100000 0x2000000
    TARGET.EPOCSTACKSIZE = 0x14000

    # To lock the application to landscape orientation
    LIBS += -lcone -leikcore -lavkon

    # For accelerometer sensor
    CONFIG += mobility
    MOBILITY += sensors

    # FOR ENABLING HARDWARE FLOATS
    MMP_RULES += "OPTION gcce -march=armv6"
    MMP_RULES += "OPTION gcce -mfpu=vfp"
    MMP_RULES += "OPTION gcce -mfloat-abi=softfp"
    MMP_RULES += "OPTION gcce -marm"
}


OTHER_FILES += \
    qtc_packaging/debian_harmattan/rules \
    qtc_packaging/debian_harmattan/README \
    qtc_packaging/debian_harmattan/copyright \
    qtc_packaging/debian_harmattan/control \
    qtc_packaging/debian_harmattan/compat \
    qtc_packaging/debian_harmattan/changelog \
    bulletdice.rc
