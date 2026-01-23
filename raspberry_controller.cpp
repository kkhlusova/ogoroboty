#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <chrono>
#include <thread> 

class RobotController {
private:
    int uart_fd;
    std::string uart_port;
    std::ofstream log_file;
    
    std::string getCurrentTime() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

public:    
    int getUartFD() const { return uart_fd; };

    RobotController(const std::string& port = "/dev/ttyUSB0") : uart_port(port) {
        // Открытие UART порта (существующий код)
        uart_fd = open(uart_port.c_str(), O_RDWR | O_NOCTTY);
        if (uart_fd < 0) {
            std::cerr << "Error opening UART port " << uart_port << std::endl;
            throw std::runtime_error("UART connection failed");
        }
        
        // Настройка UART (существующий код)
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(uart_fd, &tty) != 0) {
            std::cerr << "Error getting UART attributes" << std::endl;
            close(uart_fd);
            throw std::runtime_error("UART configuration failed");
        }
        
        cfsetospeed(&tty, B9600);  // Измените на 9600 если Arduino использует эту скорость
        cfsetispeed(&tty, B9600);
        
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ISIG;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;
        
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;
        
        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting UART attributes" << std::endl;
            close(uart_fd);
            throw std::runtime_error("UART configuration failed");
        }
        
        // Открытие файла для логирования
        std::string log_filename = "robot_log_" + getCurrentTime() + ".txt";
        std::replace(log_filename.begin(), log_filename.end(), ' ', '_');
        std::replace(log_filename.begin(), log_filename.end(), ':', '-');
        
        log_file.open(log_filename);
        if (!log_file.is_open()) {
            std::cerr << "Error opening log file" << std::endl;
            close(uart_fd);
            throw std::runtime_error("Log file creation failed");
        }
        
        log_file << "=== Robot Log Started at " << getCurrentTime() << " ===" << std::endl;
        std::cout << "Logging to file: " << log_filename << std::endl;
    }
    
    ~RobotController() {
        if (log_file.is_open()) {
            log_file << "=== Robot Log Ended at " << getCurrentTime() << " ===" << std::endl;
            log_file.close();
        }
        if (uart_fd >= 0) {
            close(uart_fd);
        }
    }
    
    bool sendCommand(char cmd) {
        ssize_t bytes_written = write(uart_fd, &cmd, 1);
        if (bytes_written < 0) {
            std::cerr << "Error writing to UART" << std::endl;
            log_file << "ERROR: Failed to send command '" << cmd << "'" << std::endl;
            return false;
        }
        
        // Логируем отправленную команду
        std::string cmd_name;
        switch(cmd) {
            case 'W': cmd_name = "Move Forward"; break;
            case 'S': cmd_name = "Move Backward"; break;
            case 'A': cmd_name = "Turn Left"; break;
            case 'D': cmd_name = "Turn Right"; break;
            case 'X': cmd_name = "Stop"; break;
            default: cmd_name = "Unknown"; break;
        }
        
        log_file << "[" << getCurrentTime() << "] Sent command: " 
                 << cmd << " (" << cmd_name << ")" << std::endl;
        return true;
    }
    
    void checkAndLogArduinoData() {
        char buffer[256];
        ssize_t bytes_read = read(uart_fd, buffer, sizeof(buffer) - 1);
        
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            
            // Записываем в лог файл
            log_file << "[" << getCurrentTime() << "] Arduino: " << buffer;
            
            // Также выводим на экран если это важное сообщение
            std::string log_line(buffer);
            if (log_line.find("LOG:") != std::string::npos ||
                log_line.find("!!!") != std::string::npos ||
                log_line.find("ERROR") != std::string::npos) {
                std::cout << "Arduino: " << buffer;
            }
            
            // Флэш лог файла для немедленной записи
            log_file.flush();
        }
    }
    
    void printControls() {
        // Существующий код...
        std::cout << "\n=== OmegaBot Controller ===" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "W - Move Forward" << std::endl;
        std::cout << "S - Move Backward" << std::endl;
        std::cout << "A - Turn Left" << std::endl;
        std::cout << "D - Turn Right" << std::endl;
        std::cout << "X - Stop Robot" << std::endl;
        std::cout << "Q - Quit" << std::endl;
        std::cout << "=========================\n" << std::endl;
    }
};

int main() {
    try {
        RobotController controller("/dev/ttyUSB0");
        controller.printControls();

        int stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, stdin_flags | O_NONBLOCK);
        
        fd_set read_fds;
        struct timeval timeout;

        auto lastHeartbeat = std::chrono::steady_clock::now();
        const auto heartbeatTimeoutMs = std::chrono::milliseconds(500);
        const char HEARTBEAT_CMD = 'K';
        
        std::cout << "Robot controller started. Press Q to quit." << std::endl;
        
        while (true) {
            FD_ZERO(&read_fds);
            FD_SET(STDIN_FILENO, &read_fds);
            FD_SET(controller.getUartFD(), &read_fds); // Нужно добавить метод getUartFD()
            
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000; // 100ms
            
            int activity = select(FD_SETSIZE, &read_fds, NULL, NULL, &timeout);
            
            if (activity < 0) {
                if (errno == EINTR) continue; //сигнал прервал
                perror("select error");
                break;
            }
            
            if (FD_ISSET(controller.getUartFD(), &read_fds)) {
                controller.checkAndLogArduinoData();
            }
            
            if (FD_ISSET(STDIN_FILENO, &read_fds)) {
                char buffer[256];
                ssize_t bytes_read = read(STDIN_FILENO, buffer, sizeof(buffer) - 1);
                
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    
                    // Обрабатываем каждую команду в буфере
                    for (int i = 0; i < bytes_read; i++) {
                        char cmd = buffer[i];
                        
                        // Пропускаем переводы строк
                        if (cmd == '\n' || cmd == '\r') continue;
                        
                        if (cmd == HEARTBEAT_CMD) {
                            lastHeartbeat = std::chrono::steady_clock::now();
                            continue;
                        }

                        if (cmd == 'Q' || cmd == 'q') {
                            controller.sendCommand('X');
                            std::cout << "\nStopping robot and exiting..." << std::endl;
                            usleep(100000);
                            return 0;
                        } else if (strchr("WSADXwsadx", cmd)) {
                            char upper_cmd = toupper(cmd);
                            
                            controller.sendCommand(upper_cmd);
                            std::cout << "Executed command: " << upper_cmd << std::endl;
                            if(strchr("WSADwsad", cmd)) {
                                lastHeartbeat = std::chrono::steady_clock::now();
                            }
                        }
                    }
                }
            }

            auto now = std::chrono::steady_clock::now();
            if (now - lastHeartbeat > heartbeatTimeoutMs) {
                std::cout << "Heartbeat timeout! Stopping robot." << std::endl;
                controller.sendCommand('X');
                lastHeartbeat = now;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
