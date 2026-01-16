# Online_mapping_with_OpenCV

Проект реализует систему автономного исследования и навигации для робота на платформе LEGO Mindstorms EV3, оснащенного лидаром HOKUYO URG-04LX-UG01. Для исследования пространства используется библиотека `OpenCV`, а для навигации — фреймворк `Nav2` из ROS2.

## Структура пакетов
- **`driver`** - управление движением робота
- **`exploring`** - основной алгоритм автономного исследования и управления
- **`lidar_vibes`** - драйвер для подключения и чтения данных с лидара HOKUYO
- **`robot_description`** - URDF-модель робота и конфигурационные файлы

## Технологический стек
- **ROS 2** - основа системы управления
- **Nav2** - стек автономной навигации
- **OpenCV** - обработка данных и построение карты
- **HOKUYO URG-04LX-UG01** - лидар для сканирования окружения
- **LEGO Mindstorms EV3** - роботизированная платформа

## Использование 
1. Установите необходимые зависимости ROS 2 и Python:
```bash
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
pip install opencv-python
pip install "numpy<1.25" "scipy<1.12"
```
2. Установите библиотеку для работы с лидаром:
```bash
git clone https://github.com/pasuder/hokuyo-python-lib
cd hokuyo-python-lib
pip install .
```
3. Склонируйте и соберите проект:
```bash
git clone https://github.com/haruaaaa/Online_mapping_with_OpenCV.git
cd Online_mapping_with_OpenCV
colcon build
source install/setup.bash
```
4. Подключитесь к роботу EV3 по SSH и запустите сервер:
```bash
python3 server.py
```
5. На управляющем компьютере запустите все ноды. **Примечание: Перед первым запуском в лаунч-файле необходимо указать корректный путь к конфигурационному файлу**
```bash
ros2 launch lidar_vibes all_nodes.launch.py
```
6. Система готова к работе! Робот начнёт автономное исследование пространства, строя карту окружения в реальном времени. Искусственно в топик `/goal_pose` можно отправить координаты предполагаемой метки и робот начнет двигаться к ней.
