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
#include <atomic>
#include <thread>
#include <mutex>

class RobotController {
private:
    int uart_fd;
    std::string uart_port;
    std::ofstream log_file;
    
    // Переменные для отслеживания состояния связи и робота
    std::atomic<bool> connection_lost{false};
    std::atomic<bool> safety_stop_active{false};
    std::atomic<char> last_robot_state{0}; // 'W'=вперед, 'S'=назад, 'A'=влево, 'D'=вправо, 'X'=стоп
    std::mutex connection_mutex;
    
    // Таймер для отслеживания потери связи
    std::chrono::steady_clock::time_point last_command_time;
    const std::chrono::milliseconds CONNECTION_TIMEOUT{500}; // 500ms таймаут
    
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

    // Функция безопасной остановки при потере связи
    void performSafetyStop() {
        std::lock_guard<std::mutex> lock(connection_mutex);
        
        if (safety_stop_active) return; // Уже выполняется
        
        safety_stop_active = true;
        char last_state = last_robot_state.load();
        
        log_file << "[" << getCurrentTime() << "] SAFETY: Connection lost! Last state: " 
                 << last_state << ", performing safety stop..." << std::endl;
        log_file.flush();
        std::cout << "\n!!! SAFETY: Connection lost! Performing safety stop..." << std::endl;
        
        switch(last_state) {
            case 'W': // Ехал вперед -> нужно проехать назад
                sendEmergencyCommand('S', 300); // Едем назад 300ms
                usleep(350000); // Ждем завершения маневра
                sendEmergencyCommand('X', 100); // Полная остановка
                break;
                
            case 'S': // Ехал назад -> нужно проехать вперед
                sendEmergencyCommand('W', 300); // Едем вперед 300ms
                usleep(350000); // Ждем завершения маневра
                sendEmergencyCommand('X', 100); // Полная остановка
                break;
                
            case 'A': // Поворачивал влево
            case 'D': // Поворачивал вправо
                sendEmergencyCommand('X', 100); // Просто останавливаемся
                break;
                
            case 'X': // Уже был остановлен
            default:
                sendEmergencyCommand('X', 100); // Останавливаем на всякий случай
                break;
        }
        
        log_file << "[" << getCurrentTime() << "] SAFETY: Safety stop completed." << std::endl;
        log_file.flush();
        std::cout << "SAFETY: Safety stop completed." << std::endl;
        
        safety_stop_active = false;
    }
    
    // Отправка аварийной команды (без проверки связи)
    void sendEmergencyCommand(char cmd, int duration_ms) {
        ssize_t bytes_written = write(uart_fd, &cmd, 1);
        if (bytes_written > 0) {
            tcdrain(uart_fd); // Ждем отправки всех данных
            usleep(duration_ms * 1000); // Ждем указанное время
        }
    }
    
    // Проверка связи
    void checkConnectionTimeout() {
    // Добавьте проверку, что прошло достаточно времени с момента инициализации
    static auto program_start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    
    // Не проверяем таймаут в первые 2 секунды работы программы
    if (std::chrono::duration_cast<std::chrono::seconds>(now - program_start) < std::chrono::seconds(2)) {
        return;
    }
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_command_time);
        
    if (elapsed > CONNECTION_TIMEOUT && !connection_lost) {
        connection_lost = true;
        log_file << "[" << getCurrentTime() << "] CONNECTION: Timeout detected!" << std::endl;
        log_file.flush();
        
        // Запускаем безопасную остановку
        performSafetyStop();
    }
}
    
    // Обновление времени последней команды
    void updateConnectionTime() {
        std::lock_guard<std::mutex> lock(connection_mutex);
        last_command_time = std::chrono::steady_clock::now();
    
        // Логируем восстановление только если связь была потеряна
        if (connection_lost) {
            connection_lost = false;
            log_file << "[" << getCurrentTime() << "] CONNECTION: Restored." << std::endl;
            log_file.flush();
        }
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
        
        // Инициализация таймера связи
        last_command_time = std::chrono::steady_clock::now();
        updateConnectionTime();
        last_robot_state = 'X'; // Начинаем с остановленного состояния
        
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

        // Если связь потеряна, сначала восстанавливаем ее статус
        if (connection_lost) {
            connection_lost = false;
            log_file << "[" << getCurrentTime() << "] CONNECTION: Restored via command." << std::endl;
        }
    
        // Обновляем время последней команды
        updateConnectionTime();
        
        // Сохраняем текущее состояние
        if (cmd != 'X' && cmd != ' ') {
            last_robot_state = cmd;
        }
        
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
            // Обновляем время связи при получении данных от Arduino
            updateConnectionTime();
            
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
    
    // Метод для проверки таймаута (должен вызываться в основном цикле)
    void checkForConnectionIssues() {
        checkConnectionTimeout();
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
        
        std::cout << "Robot controller started. Press Q to quit." << std::endl;
        
        while (true) {
            FD_ZERO(&read_fds);
            FD_SET(STDIN_FILENO, &read_fds);
            FD_SET(controller.getUartFD(), &read_fds);
            
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000; // 100ms
            
            int activity = select(FD_SETSIZE, &read_fds, NULL, NULL, &timeout);
            
            if (activity < 0) {
                if (errno == EINTR) continue; //сигнал прервал
                perror("select error");
                break;
            }
            
            // Проверяем данные от Arduino
            if (FD_ISSET(controller.getUartFD(), &read_fds)) {
                controller.checkAndLogArduinoData();
            }
            
            // Проверяем таймаут связи
            controller.checkForConnectionIssues();
            
            // Обработка команд с клавиатуры
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
                        
                        if (cmd == 'Q' || cmd == 'q') {
                            controller.sendCommand('X');
                            std::cout << "\nStopping robot and exiting..." << std::endl;
                            usleep(100000);
                            return 0;
                        } else if (strchr("WSADXwsadx", cmd)) {
                            char upper_cmd = toupper(cmd);
                            controller.sendCommand(upper_cmd);
                            std::cout << "Executed command: " << upper_cmd << std::endl;
                        }
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
