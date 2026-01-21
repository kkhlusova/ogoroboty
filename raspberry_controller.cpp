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

    int getUartFD() const {
        return uart_fd;
    }

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
        
        cfsetospeed(&tty, B9600); 
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


int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: robot_controller <W|A|S|D|X>" << std::endl;
        return 1;
    }

    char cmd = argv[1][0];

    try {
        RobotController controller("/dev/ttyUSB0");

        if (!controller.sendCommand(cmd)) {
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}


