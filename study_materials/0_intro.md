# Введение

В этой теме мы проверим работоспособность установленного ROS и чуток поуправляем манипулятором.

## Содержание

- [Содержание](#содержание)
- [Проверяем, активировано ли пространство](#проверяем-активировано-ли-пространство)
- [Запустим и сконфигурируем демонстрацию](#запустим-и-сконфигурируем-демонстрацию)
- [Поиграем с роботом](#поиграем-с-роботом)

## Проверяем, активировано ли пространство

Первым делом проверим установленную версию ROS:

```bash
echo $ROS_DISTRO
```

На экране должно появиться: `noetic`.

Теперь проверим все переменные, которые относятся к ROS:

```bash
env | grep "^ROS_"
```

Вот и все переменные, которые начинаются с "ROS_":

```
ROS_DISTRO=noetic
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ROS_PACKAGE_PATH=/opt/ros/noetic/share
ROS_PYTHON_VERSION=3
ROS_VERSION=1
ROS_ROOT=/opt/ros/noetic/share/ros
ROS_MASTER_URI=http://localhost:11311
```

С этого момента мы считаем, что установлен пакет `ros-noetic-desktop-full` и активировано пространство ROS в системе.

## Запустим и сконфигурируем демонстрацию

Теперь попробуем запустить какой-нибудь пакет.

```bash
roslaunch panda_moveit_config demo.launch rviz_tutorial:=true
```

Если вы запускаете это впервые, вы должны увидеть пустой мир в RViz, и вам придется добавить плагин "Motion Planning".

<p align="center">
    <img src=../assets/ros_topics/0_rviz_empty.png />
</p>

На вкладке "Displays" нажмите "Add"

<p align="center">
    <img src=../assets/ros_topics/0_rviz_click_add.png />
</p>

Из папки `moveit_ros_visualization` выберите `MotionPlanning` в качестве DisplayType. Нажмите `Ok`.

<p align="center">
    <img src=../assets/ros_topics/0_rviz_plugin_motion_planning_add.png />
</p>

Теперь в RViz должен появиться робот Panda:

<p align="center">
    <img src=../assets/ros_topics/0_rviz_start.png />
</p>

* После того как плагин Motion Planning загружен, мы можем его настроить. В окне `Displays` во вкладке `Global Options` установите для поля `Fixed Frame` значение `/panda_link0`.
* Теперь можно приступить к настройке плагина для вашего робота (в данном случае Panda). Разверните `MotionPlanning` в окне `Displays`.
    * Убедитесь, что в поле `Robot Description` установлено значение `robot_description`.
    * Убедитесь, что в поле `Planning Scene Topic` установлено значение `/planning_scene`. Щелкните на имени темы, чтобы открыть выпадающий список имен тем.
    * В разделе `Planning Request` измените Группу планирования на `panda_arm`.
    * В разделе `Planned Path` измените значение темы траектории на `/move_group/display_planned_path`.

<p align="center">
    <img src=../assets/ros_topics/0_rviz_plugin_start.png />
</p>

## Поиграем с роботом

Отлично! Теперь у нас все готово для взаимодействия с роботом!

Наведите на маркер (как показано на анимации ниже) и переместите его в желаемое целевое место для робота. В разделе `Planing` окна `Motion Planning` нажмите кнопку `Plan & Exicute` и наблюдайте за перемещением манипулятора!

<p align="center">
    <img src=../assets/ros_topics/0_rviz_move_panda.gif />
</p>

Если все получилось, то УРА. Все установилось и работает 🎉. Пора переходить к самому вкусному!

> Наверное, хорошо будет, если тебе еще подскажут, как выключать программы =) Говорим: Нажмите на терминал (в котором запускали робота), нажмите Ctrl, а затем букву C - такие комбинации будут показываться как Ctrl+C.
