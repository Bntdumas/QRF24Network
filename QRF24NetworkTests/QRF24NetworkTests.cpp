#include <QString>
#include <QTemporaryFile>
#include <QtTest>

#include "QRF24.h"
#include "QRF24Network.h"

class QRF24NetworkTestsTest : public QObject
{
    Q_OBJECT

public:
    QRF24NetworkTestsTest();

private Q_SLOTS:
    void initTestCase();
    void cleanupTestCase();
    void displayInfoTest();
    void communicationTest();

private:
    QRF24Network *m_network;
    QRF24 *m_radio;
};

QRF24NetworkTestsTest::QRF24NetworkTestsTest()
{
}

void QRF24NetworkTestsTest::initTestCase()
{
    m_radio = new QRF24(RPI_V2_GPIO_P1_24, RPI_V2_GPIO_P1_22, BCM2835_SPI_SPEED_8MHZ);
    m_radio->begin();
    m_network = new QRF24Network(*m_radio);
    m_network->begin(90, 0);
}

void QRF24NetworkTestsTest::cleanupTestCase()
{
    delete m_network;
    delete m_radio;
}

void QRF24NetworkTestsTest::displayInfoTest()
{
       QFile detailReal("detailReal.txt");
       if (detailReal.open(QIODevice::ReadWrite | QIODevice::Text)) {
           detailReal.write(m_network->radioModule()->getDetails(false).toLatin1());
           detailReal.reset();
       }

       QFile expectedFile("://detailReference.txt");
          if (!expectedFile.open(QIODevice::ReadOnly | QIODevice::Text))
              return;

          int lineNumber = 0;
       while (!detailReal.atEnd()) {
           QString realLine( detailReal.readLine());
           QString expectedLine(expectedFile.readLine());
           QCOMPARE(realLine, expectedLine);
           lineNumber++;
       }
}

void QRF24NetworkTestsTest::communicationTest()
{
    QSignalSpy spyNewData(m_network, SIGNAL(newData(QString,quint64)));
    while (!m_network->threadTerminated())
        QCoreApplication::instance()->processEvents();
    QVERIFY(spyNewData.count() > 3); //one frame per second is sent from the arduino, we should get at least 3
}

QTEST_MAIN(QRF24NetworkTestsTest)

#include "QRF24NetworkTests.moc"
