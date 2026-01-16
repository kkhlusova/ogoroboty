/*
 * Network communication module for robot control
 * Handles sending control commands from client to robot via TCP/UDP
 */

#ifndef NETWORK_HANDLER_H
#define NETWORK_HANDLER_H

#include <QObject>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QHostAddress>
#include <QTimer>
#include <QByteArray>

enum class RobotCommand {
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    STOP,
    SET_SPEED,
    GET_STATUS
};

struct MotorSpeeds {
    int left;
    int right;
};

class NetworkHandler : public QObject {
    Q_OBJECT

public:
    explicit NetworkHandler(QObject *parent = nullptr);
    ~NetworkHandler();

    // Подключение к роботу
    bool connectToRobot(const QString &ip, int port, bool useTcp = true);

    // Отправка команд
    bool sendCommand(RobotCommand cmd, int speed = 150);
    bool sendMotorSpeeds(int leftSpeed, int rightSpeed);
    bool requestStatus();

    // Состояние подключения
    bool isConnected() const;

signals:
    void connected();
    void disconnected();
    void errorOccurred(const QString &error);
    void statusReceived(const QString &status);

public slots:
    void disconnectFromRobot();

private slots:
    void onTcpReadyRead();
    void onUdpReadyRead();
    void onSocketError(QAbstractSocket::SocketError error);

private:
    QTcpSocket *tcpSocket;
    QUdpSocket *udpSocket;
    QString robotIp;
    int robotPort;
    bool usingTcp;
    QTimer *reconnectTimer;

    // Внутренние методы
    QByteArray createCommandPacket(RobotCommand cmd, int param1 = 0, int param2 = 0);
    void parseResponse(const QByteArray &data);
};

#endif // NETWORK_HANDLER_H