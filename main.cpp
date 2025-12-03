#include <QApplication>
#include <QMediaPlayer>
#include <QVideoWidget>  
#include <QMainWindow>   

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    QMainWindow window;                    
    QVideoWidget *video = new QVideoWidget(&window);  
    window.setCentralWidget(video);        
    window.resize(800, 600);              
    window.show();        

    QMediaPlayer *player = new QMediaPlayer;           
    player->setVideoOutput(video);                     
    player->setMedia(QUrl("gst-pipeline:udpsrc port=5000 caps=... ! qtvideosink"));

    player->play();               
    return app.exec();            


}
