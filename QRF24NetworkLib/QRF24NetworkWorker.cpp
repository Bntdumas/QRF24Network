/*
 Copyright (C) 2014 Benoit Dumas <bntdumas@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "QRF24NetworkWorker.h"
#include "QRF24.h"

#include <QTimer>
#include <QByteArray>
#include <QDebug>
#include <QtGlobal>

QRF24NetworkWorker::QRF24NetworkWorker() :
    QObject(),
    m_stopRequested(false),
    m_listeningTime(500)
{
}

void QRF24NetworkWorker::setListeningTime(const unsigned int ms)
{
    m_listeningTime = ms;
}

void QRF24NetworkWorker::mainRadioLoop(QRF24Network *networkModule)
{
    QRF24 *radioModule = networkModule->radioModule();

    while (!m_stopRequested) {

        networkModule->update();

        // Listen for new data
        for (int i = 0; i < 100; ++i) {
            listenForNewData(networkModule);
        }

        // Write data
        while (m_writeQueue.count())
            write(radioModule);
    }
    Q_EMIT finished(tr("Radio module thread finished."), false);
}

void QRF24NetworkWorker::requestWrite(const QString &data, const quint64 pipe)
{
    QPair<quint64, QString> payload;
    payload.first = pipe;
    payload.second = data;

    m_writeQueue.append(payload);
}


void QRF24NetworkWorker::stop()
{
    qDebug() << Q_FUNC_INFO;
    m_stopRequested = true;
}

void QRF24NetworkWorker::listenForNewData(QRF24Network *networkNode)
{
#ifdef TESTING
    struct payload_test
    {
        unsigned long ms;
        unsigned long counter;
    };
    if (networkNode->available()) {
        QRF24NetworkHeader header;
        payload_test payload;
        networkNode->read(header,&payload,sizeof(payload));
        qDebug() << QString("%1, frame #%2").arg(payload.ms).arg(payload.counter);
        Q_EMIT newData(QString("%1, frame #%2").arg(payload.ms, payload.counter), header.from_node);
    }
    delayMicroseconds(m_listeningTime * 1000 / 100);
    return;
#endif


    if (networkNode->available()) {
        qDebug() << "Data available!";
        QRF24NetworkHeader header;
        char buff[1024];
        int  size = networkNode->read(header, buff, sizeof(buff));
        if (!size) {
            Q_EMIT error(tr("Reading failed."));
        }
        //     Q_EMIT newData(QString(data), header.from_node);
    }
    delayMicroseconds(m_listeningTime * 1000 / 100);
}

void QRF24NetworkWorker::write(QRF24 *radioModule)
{
    for (int i = 0; i < m_writeQueue.count(); ++i) {
        QPair<quint64, QString> data = m_writeQueue.at(i);
        radioModule->openWritingPipe(data.first);
        if (!radioModule->write(qPrintable(data.second), data.second.length())) {
            Q_EMIT error(tr("Writing to pipe %1 failed.").arg(QString::number(data.first, 16)));
        }
        m_writeQueue.remove(i);
    }
}
