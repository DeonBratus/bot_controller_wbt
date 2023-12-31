# Controllers ROS2 Webots TurtleBot3Burger, Nodes ant Topics
В пакете лежит всё что нужно для запуска мира. Но только предваритльно надо еще поставить сами пакеты Webots, лучше всего ставить их локально в директорию Вашего рабочего пространства.\ 
## Предварительная настройка
Необходимо сделать сурс и пререйти папку вашего рабочего пространства
```
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
```
И далее склонировать репоизторий пакетов webots в папку src
```
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
```
