#!/bin/bash

# Тестовый скрипт для проверки системы видеонаблюдения и управления роботом

echo "=== Тестирование системы видеонаблюдения и управления роботом ==="

# Проверка наличия исполняемых файлов
SERVER_PATH="/workspace/robot_video_system/server_video/build/video_stream_server"
CLIENT_PATH="/workspace/robot_video_system/client/build/robot_control_client"

if [ ! -f "$SERVER_PATH" ]; then
    echo "Ошибка: Серверный исполняемый файл не найден по пути: $SERVER_PATH"
    echo "Сначала выполните сборку проекта: ./build.sh"
    exit 1
fi

if [ ! -f "$CLIENT_PATH" ]; then
    echo "Ошибка: Клиентский исполняемый файл не найден по пути: $CLIENT_PATH"
    echo "Сначала выполните сборку проекта: ./build.sh"
    exit 1
fi

echo "✓ Все исполняемые файлы найдены"

# Проверка зависимостей
echo "Проверка зависимостей..."

if command -v gst-launch-1.0 &> /dev/null; then
    echo "✓ GStreamer установлен"
else
    echo "⚠ GStreamer не установлен, но это может быть нормально если вы собираете на целевой системе"
fi

if command -v qmake &> /dev/null; then
    echo "✓ Qt установлен"
else
    echo "⚠ Qt не установлен, но это может быть нормально если вы собираете на целевой системе"
fi

# Проверка наличия возможных камер
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "✓ Найдены видеокамеры:"
    ls /dev/video*
else
    echo "⚠ Не найдено видеокамер (/dev/video*) - возможно, камера не подключена"
fi

echo ""
echo "=== Информация о проекте ==="
echo "Проект расположен в: /workspace/robot_video_system/"
echo ""
echo "Серверная часть (для Raspberry Pi):"
echo "  - Файлы: /workspace/robot_video_system/server_video/"
echo "  - Исполняемый файл после сборки: $SERVER_PATH"
echo ""
echo "Клиентская часть (для ноутбука):"
echo "  - Файлы: /workspace/robot_video_system/client/"
echo "  - Исполняемый файл после сборки: $CLIENT_PATH"
echo ""
echo "Для сборки выполните: ./build.sh"
echo ""
echo "Для запуска сервера (на Raspberry Pi):"
echo "  $SERVER_PATH [IP_НОУТБУКА] [ПОРТ_ВИДЕО] [ПОРТ_КОМАНД] [ID_КАМЕРЫ] [АДРЕС_ARDUINO]"
echo ""
echo "Для запуска клиента (на ноутбуке):"
echo "  $CLIENT_PATH"
echo ""

echo "=== Тестирование завершено ==="