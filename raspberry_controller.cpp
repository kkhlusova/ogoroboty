#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>

class RobotController {
private:
    int uart_fd;
    std::string uart_port;

public:
    RobotController(const std::string& port = "/dev/ttyACM0") : uart_port(port) {
        uart_fd = open(uart_port.c_str(), O_RDWR | O_NOCTTY);
        if (uart_fd < 0) {
            std::cerr << "Error opening UART port " << uart_port << std::endl;
            throw std::runtime_error("UART connection failed");
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));

        if (tcgetattr(uart_fd, &tty) != 0) {
            std::cerr << "Error getting UART attributes" << std::endl;
            close(uart_fd);
            throw std::runtime_error("UART configuration failed");
        }

        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // 1 Stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8; // 8 bits
        tty.c_cflag &= ~CRTSCTS; // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

        tty.c_lflag &= ~ICANON; // Non-canonical mode
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ISIG; // No interpretation of interrupt, quit and suspend

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes

        tty.c_cc[VMIN] = 0; // Read doesn't block
        tty.c_cc[VTIME] = 10; // 1 second timeout

        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting UART attributes" << std::endl;
            close(uart_fd);
            throw std::runtime_error("UART configuration failed");
        }
    }

    ~RobotController() {
        if (uart_fd >= 0) {
            close(uart_fd);
        }
    }

    bool sendCommand(char cmd) {
        ssize_t bytes_written = write(uart_fd, &cmd, 1);
        if (bytes_written < 0) {
            std::cerr << "Error writing to UART" << std::endl;
            return false;
        }
        return true;
    }

    void printControls() {
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
        RobotController controller("/dev/ttyUSB0"); // Порт может отличаться, при необходимости измените
        controller.printControls();

        fd_set read_fds;
        struct timeval timeout;
        
        while (true) {
            FD_ZERO(&read_fds);
            FD_SET(STDIN_FILENO, &read_fds);
            
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000; // 100ms timeout
            
            int activity = select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout);
            
            if (activity < 0) {
                perror("select error");
                break;
            } else if (FD_ISSET(STDIN_FILENO, &read_fds)) {
                char input_char;
                if (read(STDIN_FILENO, &input_char, 1) > 0) {
                    // Обработка команды
                    char cmd = 0;
                    
                    switch(input_char) {
                        case 'w':
                        case 'W':
                            cmd = 'W';
                            break;
                        case 's':
                        case 'S':
                            cmd = 'S';
                            break;
                        case 'a':
                        case 'A':
                            cmd = 'A';
                            break;
                        case 'd':
                        case 'D':
                            cmd = 'D';
                            break;
                        case 'x':
                        case 'X':
                            cmd = 'X';
                            break;
                        case 'q':
                        case 'Q':
                            cmd = 'X'; // Остановить робота перед выходом
                            controller.sendCommand(cmd);
                            std::cout << "\nStopping robot and exiting..." << std::endl;
                            return 0;
                        default:
                            // Для любых других символов просто игнорируем или посылаем команду остановки
                            continue;
                    }
                    
                    if (cmd != 0) {
                        if (!controller.sendCommand(cmd)) {
                            std::cerr << "Failed to send command to robot" << std::endl;
                            break;
                        }
                        // Вывод текущей команды для отладки
                        std::cout << "Sent command: " << cmd << std::endl;
                    }
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
