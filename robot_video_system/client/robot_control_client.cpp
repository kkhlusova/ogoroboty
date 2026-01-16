/*
 * Video Streaming Client for Robot Camera
 * Uses GStreamer to receive video stream via UDP and Qt for UI
 */

#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QLineEdit>
#include <QFormLayout>

#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <QMainWindow>
#include <QFrame>

#include "network_handler.h"

class RobotControlWidget : public QMainWindow {
    Q_OBJECT

public:
    explicit RobotControlWidget(QWidget *parent = nullptr);
    ~RobotControlWidget();

public slots:
    void startStreaming();
    void stopStreaming();

private slots:
    void onForwardClicked();
    void onBackwardClicked();
    void onLeftClicked();
    void onRightClicked();
    void onStopClicked();
    void onStartClicked();

private:
    void setupUI();
    void setupGStreamer();
    void sendRobotCommand(const QString& command, int value = 0);

    QWidget *centralWidget;
    QLabel *videoLabel;
    QPushButton *btnForward;
    QPushButton *btnBackward;
    QPushButton *btnLeft;
    QPushButton *btnRight;
    QPushButton *btnStop;
    QPushButton *btnStart;
    QPushButton *btnExit;
    QLineEdit *ipInput;
    QLineEdit *portInput;
    QPushButton *btnConnect;
    
    GstElement *pipeline;
    GstElement *source, *decoder, *converter, *sink;
    GstBus *bus;
    
    bool streamingActive;
    NetworkHandler *networkHandler;
};

RobotControlWidget::RobotControlWidget(QWidget *parent)
    : QMainWindow(parent), streamingActive(false) {
    
    // Создание network handler
    networkHandler = new NetworkHandler(this);
    
    setupUI();
    setupGStreamer();
    
    // Подключение к сигналам network handler
    connect(networkHandler, &NetworkHandler::connected, []() {
        qDebug() << "Connected to robot";
    });
    
    connect(networkHandler, &NetworkHandler::errorOccurred, [](const QString &error) {
        qDebug() << "Network error:" << error;
    });
}

RobotControlWidget::~RobotControlWidget() {
    if (pipeline) {
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(pipeline);
    }
}

void RobotControlWidget::setupUI() {
    centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    // Метка для видео
    videoLabel = new QLabel("Video Stream Will Appear Here");
    videoLabel->setAlignment(Qt::AlignCenter);
    videoLabel->setMinimumSize(640, 480);
    videoLabel->setStyleSheet("QLabel { background-color : black; color : white; border: 1px solid gray; }");

    // Поля для ввода IP и порта
    ipInput = new QLineEdit("192.168.1.101"); // IP по умолчанию
    portInput = new QLineEdit("8888"); // порт по умолчанию
    btnConnect = new QPushButton("Connect to Robot");

    // Кнопки управления
    btnForward = new QPushButton("↑ Forward");
    btnBackward = new QPushButton("↓ Backward");
    btnLeft = new QPushButton("← Left");
    btnRight = new QPushButton("→ Right");
    btnStop = new QPushButton("● Stop");
    btnStart = new QPushButton("Start Video");
    btnExit = new QPushButton("Exit");

    // Настройка размеров кнопок
    QList<QPushButton*> buttons = {btnForward, btnBackward, btnLeft, btnRight, btnStop, btnStart, btnExit, btnConnect};
    for (auto button : buttons) {
        button->setMinimumHeight(50);
        button->setStyleSheet("font-size: 14pt;");
    }

    // Макет для подключения
    QFormLayout *connectionLayout = new QFormLayout;
    connectionLayout->addRow("Robot IP:", ipInput);
    connectionLayout->addRow("Robot Port:", portInput);
    connectionLayout->addWidget(btnConnect);

    // Макет управления
    QHBoxLayout *hLayout1 = new QHBoxLayout;
    hLayout1->addWidget(new QWidget()); // пустышка слева
    hLayout1->addWidget(btnForward);
    hLayout1->addWidget(new QWidget()); // пустышка справа

    QHBoxLayout *hLayout2 = new QHBoxLayout;
    hLayout2->addWidget(btnLeft);
    hLayout2->addWidget(btnStop);
    hLayout2->addWidget(btnRight);

    QHBoxLayout *hLayout3 = new QHBoxLayout;
    hLayout3->addWidget(new QWidget()); // пустышка слева
    hLayout3->addWidget(btnBackward);
    hLayout3->addWidget(new QWidget()); // пустышка справа

    QVBoxLayout *controlsLayout = new QVBoxLayout;
    controlsLayout->addLayout(hLayout1);
    controlsLayout->addLayout(hLayout2);
    controlsLayout->addLayout(hLayout3);

    // Кнопки старт/стоп и выход
    QHBoxLayout *bottomLayout = new QHBoxLayout;
    bottomLayout->addWidget(btnStart);
    bottomLayout->addWidget(btnExit);

    // Общий макет
    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(videoLabel);
    mainLayout->addLayout(connectionLayout);
    mainLayout->addLayout(controlsLayout);
    mainLayout->addLayout(bottomLayout);

    centralWidget->setLayout(mainLayout);

    // Подключение сигналов
    connect(btnForward, &QPushButton::clicked, this, &RobotControlWidget::onForwardClicked);
    connect(btnBackward, &QPushButton::clicked, this, &RobotControlWidget::onBackwardClicked);
    connect(btnLeft, &QPushButton::clicked, this, &RobotControlWidget::onLeftClicked);
    connect(btnRight, &QPushButton::clicked, this, &RobotControlWidget::onRightClicked);
    connect(btnStop, &QPushButton::clicked, this, &RobotControlWidget::onStopClicked);
    connect(btnStart, &QPushButton::clicked, this, &RobotControlWidget::onStartClicked);
    connect(btnExit, &QPushButton::clicked, this, &QMainWindow::close);
    connect(btnConnect, &QPushButton::clicked, [this]() {
        QString ip = ipInput->text();
        int port = portInput->text().toInt();
        networkHandler->connectToRobot(ip, port, true); // используем TCP
    });

    setWindowTitle("Robot Control Interface");
    resize(800, 700);
}

void RobotControlWidget::setupGStreamer() {
    gst_init(NULL, NULL);

    // Создание элементов GStreamer
    source = gst_element_factory_make("udpsrc", "source");
    decoder = gst_element_factory_make("rtph264depay", "depayload");
    converter = gst_element_factory_make("avdec_h264", "decoder");
    sink = gst_element_factory_make("autovideosink", "sink");

    // Установка порта для udpsrc
    g_object_set(source, "port", 5000, NULL);

    // Создание pipeline
    pipeline = gst_pipeline_new("video-pipeline");
    
    if (!pipeline || !source || !decoder || !converter || !sink) {
        qDebug() << "Could not create GStreamer pipeline elements";
        return;
    }

    // Добавление элементов в pipeline
    gst_bin_add_many(GST_BIN(pipeline), source, decoder, converter, sink, NULL);

    // Связывание элементов
    if (gst_element_link_many(source, decoder, converter, sink, NULL) != TRUE) {
        qDebug() << "Elements could not be linked";
        gst_object_unref(pipeline);
        return;
    }

    // Получение шины
    bus = gst_element_get_bus(pipeline);
}

void RobotControlWidget::startStreaming() {
    if (streamingActive) return;

    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        qDebug() << "Unable to set the pipeline to the playing state.";
        return;
    }

    streamingActive = true;
    btnStart->setText("Stop Video");
    qDebug() << "Video streaming started";
}

void RobotControlWidget::stopStreaming() {
    if (!streamingActive) return;

    gst_element_set_state(pipeline, GST_STATE_NULL);
    streamingActive = false;
    btnStart->setText("Start Video");
    qDebug() << "Video streaming stopped";
}

void RobotControlWidget::onForwardClicked() {
    sendRobotCommand("forward", 150);
    qDebug() << "Forward command sent";
}

void RobotControlWidget::onBackwardClicked() {
    sendRobotCommand("backward", 150);
    qDebug() << "Backward command sent";
}

void RobotControlWidget::onLeftClicked() {
    sendRobotCommand("turn_left", 150);
    qDebug() << "Left command sent";
}

void RobotControlWidget::onRightClicked() {
    sendRobotCommand("turn_right", 150);
    qDebug() << "Right command sent";
}

void RobotControlWidget::onStopClicked() {
    sendRobotCommand("stop");
    qDebug() << "Stop command sent";
}

void RobotControlWidget::onStartClicked() {
    if (streamingActive) {
        stopStreaming();
    } else {
        startStreaming();
    }
}

void RobotControlWidget::sendRobotCommand(const QString& command, int value) {
    // Отправляем команду через network handler
    if (!networkHandler->isConnected()) {
        qDebug() << "Not connected to robot!";
        return;
    }
    
    if (command == "forward") {
        networkHandler->sendCommand(RobotCommand::FORWARD, value);
        qDebug() << "Sent forward command with speed:" << value;
    } else if (command == "backward") {
        networkHandler->sendCommand(RobotCommand::BACKWARD, value);
        qDebug() << "Sent backward command with speed:" << value;
    } else if (command == "turn_left") {
        networkHandler->sendCommand(RobotCommand::TURN_LEFT, value);
        qDebug() << "Sent turn left command with speed:" << value;
    } else if (command == "turn_right") {
        networkHandler->sendCommand(RobotCommand::TURN_RIGHT, value);
        qDebug() << "Sent turn right command with speed:" << value;
    } else if (command == "stop") {
        networkHandler->sendCommand(RobotCommand::STOP);
        qDebug() << "Sent stop command";
    } else if (command == "set_speed") {
        // Для установки индивидуальных скоростей двигателей
        networkHandler->sendMotorSpeeds(value, value); // для простоты обе скорости одинаковые
        qDebug() << "Sent set speed command with value:" << value;
    }
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Инициализация GStreamer
    gst_init(&argc, &argv);

    RobotControlWidget widget;
    widget.show();

    return app.exec();
}

#include "robot_control_client.moc"