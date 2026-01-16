/*
 * Network communication module implementation
 * Handles sending control commands from client to robot via TCP/UDP
 */

#include "network_handler.h"
#include <QDataStream>
#include <QDebug>

NetworkHandler::NetworkHandler(QObject *parent)
    : QObject(parent)
    , tcpSocket(nullptr)
    , udpSocket(nullptr)
    , robotPort(8888)  // порт по умолчанию
    , usingTcp(true)
{
    reconnectTimer = new QTimer(this);
    reconnectTimer->setSingleShot(true);
}

NetworkHandler::~NetworkHandler() {
    disconnectFromRobot();
}

bool NetworkHandler::connectToRobot(const QString &ip, int port, bool useTcp) {
    disconnectFromRobot(); // отключиться если было подключение
    
    robotIp = ip;
    robotPort = port;
    usingTcp = useTcp;
    
    if (useTcp) {
        tcpSocket = new QTcpSocket(this);
        
        connect(tcpSocket, &QTcpSocket::readyRead, this, &NetworkHandler::onTcpReadyRead);
        connect(tcpSocket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
                this, &NetworkHandler::onSocketError);
        connect(tcpSocket, &QTcpSocket::connected, this, [this]() { emit connected(); });
        connect(tcpSocket, &QTcpSocket::disconnected, this, [this]() { emit disconnected(); });
        
        tcpSocket->connectToHost(QHostAddress(ip), port);
        
        if (tcpSocket->waitForConnected(5000)) {  // таймаут 5 секунд
            qDebug() << "Connected to robot at" << ip << ":" << port;
            return true;
        } else {
            qDebug() << "Failed to connect to robot:" << tcpSocket->errorString();
            return false;
        }
    } else {
        udpSocket = new QUdpSocket(this);
        
        connect(udpSocket, &QUdpSocket::readyRead, this, &NetworkHandler::onUdpReadyRead);
        
        // Для UDP нет явного подключения, просто проверяем возможность отправки
        bool result = true; // условно считаем, что UDP "подключен"
        
        if (result) {
            qDebug() << "UDP socket ready for robot communication at" << ip << ":" << port;
            return true;
        } else {
            qDebug() << "Failed to prepare UDP socket for robot communication";
            return false;
        }
    }
}

void NetworkHandler::disconnectFromRobot() {
    if (tcpSocket) {
        tcpSocket->disconnectFromHost();
        tcpSocket->deleteLater();
        tcpSocket = nullptr;
    }
    
    if (udpSocket) {
        udpSocket->close();
        udpSocket->deleteLater();
        udpSocket = nullptr;
    }
    
    qDebug() << "Disconnected from robot";
}

bool NetworkHandler::sendCommand(RobotCommand cmd, int speed) {
    if (!isConnected()) {
        qDebug() << "Not connected to robot";
        return false;
    }
    
    QByteArray packet = createCommandPacket(cmd, speed);
    
    if (usingTcp && tcpSocket) {
        qint64 bytesWritten = tcpSocket->write(packet);
        tcpSocket->flush();
        
        if (bytesWritten == -1) {
            qDebug() << "Error sending TCP command:" << tcpSocket->errorString();
            return false;
        }
        
        qDebug() << "TCP command sent, bytes:" << bytesWritten;
        return true;
    } else if (!usingTcp && udpSocket) {
        qint64 bytesWritten = udpSocket->writeDatagram(packet, QHostAddress(robotIp), robotPort);
        
        if (bytesWritten == -1) {
            qDebug() << "Error sending UDP command:" << udpSocket->errorString();
            return false;
        }
        
        qDebug() << "UDP command sent, bytes:" << bytesWritten;
        return true;
    }
    
    return false;
}

bool NetworkHandler::sendMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (!isConnected()) {
        qDebug() << "Not connected to robot";
        return false;
    }
    
    QByteArray packet = createCommandPacket(RobotCommand::SET_SPEED, leftSpeed, rightSpeed);
    
    if (usingTcp && tcpSocket) {
        qint64 bytesWritten = tcpSocket->write(packet);
        tcpSocket->flush();
        
        if (bytesWritten == -1) {
            qDebug() << "Error sending motor speeds via TCP:" << tcpSocket->errorString();
            return false;
        }
        
        return true;
    } else if (!usingTcp && udpSocket) {
        qint64 bytesWritten = udpSocket->writeDatagram(packet, QHostAddress(robotIp), robotPort);
        
        if (bytesWritten == -1) {
            qDebug() << "Error sending motor speeds via UDP:" << udpSocket->errorString();
            return false;
        }
        
        return true;
    }
    
    return false;
}

bool NetworkHandler::requestStatus() {
    return sendCommand(RobotCommand::GET_STATUS);
}

bool NetworkHandler::isConnected() const {
    if (usingTcp) {
        return tcpSocket && tcpSocket->state() == QTcpSocket::ConnectedState;
    } else {
        return udpSocket && udpSocket->isValid();
    }
}

QByteArray NetworkHandler::createCommandPacket(RobotCommand cmd, int param1, int param2) {
    QByteArray packet;
    QDataStream stream(&packet, QIODevice::WriteOnly);
    
    // Простой формат пакета: [ID команды][параметр1][параметр2]
    stream << static_cast<qint8>(cmd);
    stream << static_cast<qint8>(param1);
    stream << static_cast<qint8>(param2);
    
    return packet;
}

void NetworkHandler::parseResponse(const QByteArray &data) {
    QDataStream stream(data);
    qint8 cmd, param1, param2;
    
    stream >> cmd >> param1 >> param2;
    
    // Обработка ответа от робота
    QString status = QString("Status - Cmd: %1, Param1: %2, Param2: %3")
                        .arg(cmd).arg(param1).arg(param2);
                        
    emit statusReceived(status);
}

void NetworkHandler::onTcpReadyRead() {
    if (!tcpSocket) return;
    
    QByteArray data = tcpSocket->readAll();
    parseResponse(data);
}

void NetworkHandler::onUdpReadyRead() {
    if (!udpSocket) return;
    
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        
        QHostAddress sender;
        quint16 senderPort;
        
        udpSocket->readDatagram(datagram.data(), datagram.size(),
                                &sender, &senderPort);
                                
        parseResponse(datagram);
    }
}

void NetworkHandler::onSocketError(QAbstractSocket::SocketError error) {
    qDebug() << "Socket error:" << error << tcpSocket->errorString();
    emit errorOccurred(tcpSocket->errorString());
}