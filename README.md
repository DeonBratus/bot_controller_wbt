# Controllers ROS2 Webots TurtleBot3Burger, Nodes ant Topics
В пакете лежит всё что нужно для запуска мира. Но только предваритльно надо еще поставить сами пакеты Webots, лучше всего ставить их локально в директорию Вашего рабочего пространства.\ 
## Предварительная настройка
Необходимо сделать сурс и пререйти папку вашего рабочего пространства, замените <your_name_of_workspace> на имя вашего
```
source /opt/ros/humble/setup.bash
cd ~/<your_name_of_workspace>
```
И далее склонировать репоизторий пакетов webots в папку src
```
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
```
После этого необходимо провести зависимости, находясь
```
sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble
```
Далее необходимо собрать пакеты
```
colcon build
```
После этого можно проверять работоспособность пакетов
```
source install/local_setup.bash
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```
После этого в папку необходимо создать в папке __src__ папку __bot_controller__ и закинуть все файлы с этого пакета, должна получится следующая структура\
.
├── __bot_controller__
│   ├── __init__.py
│   ├── aruco_edu.ipynb
│   ├── camera_cv_sub.py
│   ├── keyboard_manager.py
│   ├── my_robot_driver.py
│   └── odom_get.py
├── __controllers__
│   └── govno_controller
│       └── govno_controller.py
├── __launch__
│   ├── robot_launch.py
│   └── robot_tools_launch.py
├── package.xml
├── __resource__
│   ├── bot_controller
│   └── my_robot.urdf
├── setup.cfg
├── setup.py
├── __test__
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── __worlds__
    ├── my_world.wbt
    └── spot2.wbt











