/*
 * Network Server for Robot Control Commands
 * Receives commands from client and forwards to Arduino via I2C
 */

#ifndef ROBOT_COMMAND_SERVER_H
#define ROBOT_COMMAND_SERVER_H

#include <QTcpServer>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QDataStream>
#include <QThread>
#include <QDebug>
#include <QObject>

// Включаем заголовок RobotController как отдельный include
// В реализации будет подключение к настоящему классу
class RobotController;  // Предварительное объявление

class RobotCommandServer : public QObject {
    Q_OBJECT

public:
    explicit RobotCommandServer(QObject *parent = nullptr);
    ~RobotCommandServer();

    bool startServer(int tcpPort = 8888, int udpPort = 8889);
    void stopServer();

    // Подключение к Arduino
    bool connectToArduino(uint8_t address = 0x08);

signals:
    void commandReceived(const QString &command);
    void errorOccurred(const QString &error);

private slots:
    void onNewTcpConnection();
    void onTcpDataReceived();
    void onUdpDataReceived();

private:
    QTcpServer *tcpServer;
    QUdpSocket *udpSocket;
    QTcpSocket *activeTcpSocket;
    
    RobotController *robotController;
    
    void processCommand(qint8 cmd, qint8 param1, qint8 param2);
};

#endif // ROBOT_COMMAND_SERVER_H

