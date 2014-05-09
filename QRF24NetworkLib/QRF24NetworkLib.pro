top_srcdir=$$PWD

!linux-rasp-pi-g++: {
    error("QRF24Network lib only available for Raspberry Pi")
}

QT       -= gui

TARGET = QRF24Network
TEMPLATE = lib

target.path = /opt/$$TARGET/
INSTALLS += target

DEFINES += QRF24NETWORK_LIBRARY

HEADERS += \
          $$top_srcdir/bcm2835.h \
          $$top_srcdir/nRF24L01.h \
          $$top_srcdir/QRF24.h \
          $$top_srcdir/QRF24Network_config.h \
          $$top_srcdir/QRF24Network.h \
          $$top_srcdir/QRF24NetworkWorker.h \
          #$$top_srcdir/Sync.h \ not ready

SOURCES += \
          $$top_srcdir/QRF24.cpp \
          $$top_srcdir/QRF24Network.cpp \
          $$top_srcdir/QRF24NetworkWorker.cpp \
          #$$top_srcdir/Sync.cpp \ not ready
          $$top_srcdir/bcm2835.c \
