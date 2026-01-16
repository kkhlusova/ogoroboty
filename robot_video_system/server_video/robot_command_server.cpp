/*
 * Network Server for Robot Control Commands - Implementation
 * Receives commands from client and forwards to Arduino via I2C
 */

#include "robot_command_server.h"
#include <QTcpServer>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QDataStream>
#include <QThread>
#include <QDebug>
#include <QObject>
#include <QTimer>

#include "../server/include/RobotController.h"  // используем существующий контроллер

RobotCommandServer::RobotCommandServer(QObject *parent)
    : QObject(parent)
    , tcpServer(nullptr)
    , udpSocket(nullptr)
    , activeTcpSocket(nullptr)
    , robotController(nullptr)
{
}

RobotCommandServer::~RobotCommandServer() {
    stopServer();
    if (robotController) {
        delete robotController;
    }
}

bool RobotCommandServer::startServer(int tcpPort, int udpPort) {
    // Создание TCP сервера
    tcpServer = new QTcpServer(this);
    
    connect(tcpServer, &QTcpServer::newConnection, this, &RobotCommandServer::onNewTcpConnection);
    
    if (!tcpServer->listen(QHostAddress::Any, tcpPort)) {
        qDebug() << "Cannot start TCP server on port" << tcpPort << ":" << tcpServer->errorString();
        return false;
    }
    
    qDebug() << "TCP Command server listening on port" << tcpPort;
    
    // Создание UDP сервера
    udpSocket = new QUdpSocket(this);
    
    connect(udpSocket, &QUdpSocket::readyRead, this, &RobotCommandServer::onUdpDataReceived);
    
    if (!udpSocket->bind(QHostAddress::Any, udpPort)) {
        qDebug() << "Cannot start UDP server on port" << udpPort << ":" << udpSocket->errorString();
        return false;
    }
    
    qDebug() << "UDP Command server listening on port" << udpPort;
    
    return true;
}

void RobotCommandServer::stopServer() {
    if (tcpServer) {
        tcpServer->close();
        tcpServer->deleteLater();
        tcpServer = nullptr;
    }
    
    if (udpSocket) {
        udpSocket->close();
        udpSocket->deleteLater();
        udpSocket = nullptr;
    }
    
    if (activeTcpSocket) {
        activeTcpSocket->disconnectFromHost();
        activeTcpSocket->deleteLater();
        activeTcpSocket = nullptr;
    }
}

bool RobotCommandServer::connectToArduino(uint8_t address) {
    if (robotController) {
        delete robotController;
    }
    
    robotController = new RobotController(address);
    
    if (!robotController->connect()) {
        qDebug() << "Failed to connect to Arduino:" << robotController->getLastError();
        return false;
    }
    
    qDebug() << "Successfully connected to Arduino at address 0x" << QString::number(address, 16);
    return true;
}

void RobotCommandServer::onNewTcpConnection() {
    if (activeTcpSocket) {
        // Закрыть предыдущее соединение если есть
        activeTcpSocket->disconnectFromHost();
        activeTcpSocket->deleteLater();
    }
    
    activeTcpSocket = tcpServer->nextPendingConnection();
    
    connect(activeTcpSocket, &QTcpSocket::readyRead, this, &RobotCommandServer::onTcpDataReceived);
    connect(activeTcpSocket, &QTcpSocket::disconnected, [this]() {
        qDebug() << "Client disconnected";
        activeTcpSocket->deleteLater();
        activeTcpSocket = nullptr;
    });
    
    qDebug() << "New client connected";
}

void RobotCommandServer::onTcpDataReceived() {
    if (!activeTcpSocket) return;
    
    QDataStream stream(activeTcpSocket);
    qint8 cmd, param1, param2;
    
    while (activeTcpSocket->bytesAvailable() >= sizeof(qint8) * 3) {
        stream >> cmd >> param1 >> param2;
        
        processCommand(cmd, param1, param2);
    }
}

void RobotCommandServer::onUdpDataReceived() {
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        
        QHostAddress sender;
        quint16 senderPort;
        
        udpSocket->readDatagram(datagram.data(), datagram.size(),
                                &sender, &senderPort);
        
        QDataStream stream(datagram);
        qint8 cmd, param1, param2;
        
        if (datagram.size() >= 3) {
            stream >> cmd >> param1 >> param2;
            
            processCommand(cmd, param1, param2);
        }
    }
}

void RobotCommandServer::processCommand(qint8 cmd, qint8 param1, qint8 param2) {
    if (!robotController || !robotController->isConnected()) {
        qDebug() << "No connection to Arduino, ignoring command";
        return;
    }
    
    switch (cmd) {
        case 0: // FORWARD
            robotController->moveForward(static_cast<uint8_t>(param1));
            qDebug() << "Moving forward at speed" << param1;
            break;
            
        case 1: // BACKWARD
            robotController->moveBackward(static_cast<uint8_t>(param1));
            qDebug() << "Moving backward at speed" << param1;
            break;
            
        case 2: // TURN_LEFT
            robotController->turnLeft(static_cast<uint8_t>(param1));
            qDebug() << "Turning left at speed" << param1;
            break;
            
        case 3: // TURN_RIGHT
            robotController->turnRight(static_cast<uint8_t>(param1));
            qDebug() << "Turning right at speed" << param1;
            break;
            
        case 4: // STOP
            robotController->stop();
            qDebug() << "Stopping robot";
            break;
            
        case 5: // SET_SPEED
            robotController->setMotorSpeeds(static_cast<uint8_t>(param1), static_cast<uint8_t>(param2));
            qDebug() << "Setting motor speeds - Left:" << param1 << "Right:" << param2;
            break;
            
        case 6: // GET_STATUS
            {
                auto status = robotController->getStatus();
                // В реальной реализации можно отправить статус обратно клиенту
                qDebug() << "Status requested - Battery:" << status.batteryLevel 
                         << "%, Temperature:" << status.temperature << "C";
            }
            break;
            
        default:
            qDebug() << "Unknown command received:" << cmd;
            break;
    }
    
    emit commandReceived(QString("Command %1 executed").arg(cmd));
}