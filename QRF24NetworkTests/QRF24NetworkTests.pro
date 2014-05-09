QT       += testlib

QT       -= gui

top_srcdir=$$PWD
top_builddir=$$shadowed($$PWD)

TARGET = QRF24NetworkTests
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

target.path = /opt/$$TARGET/
INSTALLS += target

SOURCES += $$top_srcdir/QRF24NetworkTests.cpp
DEFINES += SRCDIR=\\\"$$top_srcdir/\\\"

INCLUDEPATH += $$top_srcdir/../QRF24NetworkLib
LIBS += -L$$top_builddir/../QRF24NetworkLib/ -lQRF24Network

OTHER_FILES += \
    detailReference.txt

RESOURCES += \
    res.qrc
