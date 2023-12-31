# Controllers ROS2 Webots TurtleBot3Burger, Nodes ant Topics
В пакете лежит всё что нужно для запуска мира. Но только предваритльно надо еще поставить сами пакеты Webots, лучше всего ставить их локально в директорию Вашего рабочего пространства.
## Предварительная настройка
Необходимо сделать сурс и пререйти папку вашего рабочего пространства, замените <your_name_of_workspace> на имя вашего
```bash
source /opt/ros/humble/setup.bash
cd ~/<your_name_of_workspace>
```
И далее склонировать репоизторий пакетов webots в папку src
```bash
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
```
После этого необходимо провести зависимости, находясь
```bash
sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble
```
Далее необходимо собрать пакеты
```bash
colcon build
```
После этого можно проверять работоспособность пакетов
```bash
source install/local_setup.bash
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```
После этого в папку необходимо создать в папке __src__ папку __bot_controller__ и закинуть все файлы с этого пакета, должна получится следующая структура\
.
├── __bot_controller__\
│   ├── __init__.py\
│   ├── camera_cv_sub.py\
│   ├── keyboard_manager.py\
│   ├── my_robot_driver.py\
│   └── odom_get.py\
├── __launch__\
│   ├── robot_launch.py\
│   └── robot_tools_launch.py\
├── package.xml\
├── __resource__\
│   ├── bot_controller\
│   └── my_robot.urdf\
├── setup.cfg\
├── setup.py\
├── __test__\
│   ├── test_copyright.py\
│   ├── test_flake8.py\
│   └── test_pep257.py\
└── __worlds__\
    ├── my_world.wbt\
    └── spot2.wbt\
## Запуск мира
Далее необходимо запустить саму симуляцию мира
```bash
cd ~/ros2_ws
source install/local_setup.sh
```
```bash
ros2 launch bot_controlller robot_launch.py
```
Или с Rviz, добавив соотвтествующий параметер
```bash
ros2 launch bot_controller robot_launch.py rviz:=true
```
После этого должен запуститься webots и rviz. В терминале должно быть сообщение об успешном подключении контроллера. Если пристутсвуют ошибки свзяанные с webots, to необходимо удалить папки __install__, __build__, __log__ исправить ошибки, или установить нехватающие компоненты, а потом повторить __colcon build__. Удалять ранее упомянутые папки обязательно, иначе изменения не будут внесены.\
Если будет ошибка, связанная с launch файлами, то в этом случае файл _bot_controller/launch/__robot_tools_launch.py___ скопировать в папку ___../install/bot_controller/share/bot_controller/launch___. Это происходит из-за того, что colcon не весгда переносит файлы в install или build.
## Запуск нод
### Управление с помощью клавиатуры 
Сначала необходимо запустить мир по инструкции выше. После, в новом терминале ввести следующее\
```bash
cd ~/ros2_ws
source install/local_setup.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Это уже встроенный в базовый ROS2 пакет для управления по twist. Управление будет указано в терминале. Также в нем можно будет указать скорость поворота. keyboard_manager.py на данный момент не работает. Преобразование передвижения находится в файле _bot_controller/my_robot_driver.py_ в методе _step_.
### Получение данных с камеры
Получение данных с камера и вывод изображения находтся в файле _bot_controller/camera_sv_sub.py_. Для запуска этой ноды необходимо ввести команду в новом терминале
```bash
cd ~/ros2_ws
source install/local_setup.sh
ros2 run bot_controller print_image
```
## получение данных с лидара
Данные лидара уже по умолчанию публикуется в топик __/scan__. И подписавшись на него можно их получать. Пример базовой ноды для получения данных с лидара.
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class getLaser(Node):
    
    def __init__(self):
        super().__init__('get_laser')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.get_laser, 10)

    def get_laser(self, msg):
        las_msg = msg
        print(las_msg)        


def main(args=None):
    rclpy.init(args=args)
    las_info = getLaser()
    rclpy.spin(las_info)
    las_info.destroy_node()
    rclpy.shutdown()
```
И в файле setup.py добавить в entry_point в console_script добавит строчку
```python
'<command_name> = bot_controller.<file_name>:main'
```
Например выглядит это так:
```python
entry_points={
        'console_scripts': [
            'my_robot_driver = bot_controller.my_robot_driver:main',
            'kbd = bot_controller.keyboard_manager:main',
            'print_image = bot_controller.camera_cv_sub:main',
            'get_odom = bot_controller.odom_get:main',
            'get_las = bot_controller.laser_get:main'
        ]
```
После этого консоль должно засыпать данными с лидара





