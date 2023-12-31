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











