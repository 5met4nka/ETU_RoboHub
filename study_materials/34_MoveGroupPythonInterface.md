# Интерфейс Move Group на Python

![Move Group Python Interface](../assets/moveit/34_move_group_python_interface.png)

**Интерфейс Move Group на Python** — это один из самых простых пользовательских интерфейсов MoveIt. Этот интерфейс предоставляет обертки для выполнения большинства операций, необходимых пользователю. Он позволяет:

- Устанавливать цели для сочленений или поз.
- Создавать планы движений.
- Перемещать робота.
- Добавлять объекты в окружающую среду.
- Присоединять и отсоединять объекты от робота.

Посмотрите это короткое [видео на YouTube](https://www.youtube.com/watch?v=sample) для демонстрации возможностей интерфейса Move Group на Python.

---

## Запуск RViz и узла MoveGroup

1. Откройте два терминала.  
2. В первом терминале запустите RViz и дождитесь завершения загрузки:

   ```bash
   roslaunch panda_moveit_config demo.launch
   ```

3. Во втором терминале выполните Python-скрипт с использованием команды **rosrun**:

   ```bash
   rosrun moveit_tutorials move_group_python_interface_tutorial.py
   ```

## Ожидаемый результат

В **RViz** вы должны увидеть следующее (после нажатия клавиши `<Enter>` в терминале, где была выполнена команда `rosrun`):

1. Робот планирует и перемещает руку к заданной цели для сочленений.  
2. Робот планирует путь к заданной целевой позе.  
3. Робот планирует декартовый путь.  
4. Робот отображает запланированный декартовый путь повторно.  
5. Робот выполняет декартовый путь.  
6. В месте расположения конечного эффектора Panda появляется коробка.  
7. Коробка меняет цвет, указывая, что она присоединена.  
8. Робот планирует и выполняет декартовый путь с присоединенной коробкой.  
9. Коробка снова меняет цвет, указывая, что она отсоединена.  
10. Коробка исчезает.

---

## Полный код

Код можно найти в [репозитории GitHub](https://github.com/ros-planning/moveit_tutorials) с уроками.

---

## Использование интерфейсов MoveIt на Python

Для работы с интерфейсом MoveIt на Python мы импортируем пространство имен `moveit_commander`. Оно предоставляет классы:

- **MoveGroupCommander** — для работы с группами планирования.  
- **PlanningSceneInterface** — для управления сценой планирования.  
- **RobotCommander** — для получения информации о кинематической модели робота и текущих состояниях его сочленений.  

Мы также импортируем `rospy` и некоторые сообщения, которые понадобятся в ходе работы.

### Код инициализации

```python
# Импорты для совместимости Python 2/3
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # Совместимость с Python 2
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Инициализация moveit_commander и узла rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

# Создание объекта RobotCommander
# Этот объект предоставляет информацию о кинематической модели робота и текущих состояниях суставов
robot = moveit_commander.RobotCommander()

# Создание объекта PlanningSceneInterface
# Этот объект предоставляет удаленный интерфейс для работы с внутренним пониманием окружающего мира
scene = moveit_commander.PlanningSceneInterface()
```

## Создание объекта MoveGroupCommander

`MoveGroupCommander` — это интерфейс для работы с группой планирования (группой сочленений). В данном уроке мы работаем с основными суставами руки робота Panda, поэтому устанавливаем имя группы как `panda_arm`. Если вы используете другой робот, замените это значение на название вашей группы планирования руки. Этот интерфейс позволяет планировать и выполнять движения.

```python
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
```

## Создание ROS-публишера для отображения траекторий

Публишер `DisplayTrajectory` используется для визуализации траекторий в RViz.

```python
display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)
```

---

## Получение базовой информации

Вы можете получить полезную информацию о роботе и его текущем состоянии:

1. Имя опорного кадра робота:
   ```python
   planning_frame = move_group.get_planning_frame()
   print("============ Planning frame: %s" % planning_frame)
   ```

2. Имя звена конечного эффектора:
   ```python
   eef_link = move_group.get_end_effector_link()
   print("============ End effector link: %s" % eef_link)
   ```

3. Список всех групп планирования робота:
   ```python
   group_names = robot.get_group_names()
   print("============ Available Planning Groups:", robot.get_group_names())
   ```

4. Текущее состояние робота:
   ```python
   print("============ Printing robot state")
   print(robot.get_current_state())
   print("")
   ```

---

## Планирование движения к цели для сочленений

Конфигурация робота Panda в положении "ноль" находится в сингулярности, поэтому первым шагом мы задаем более оптимальное положение. Для удобства используем константу `tau = 2 * pi`.

### Пример:

1. Получите текущие значения сочленений и измените некоторые из них:
   ```python
   joint_goal = move_group.get_current_joint_values()
   joint_goal[0] = 0
   joint_goal[1] = -tau / 8
   joint_goal[2] = 0
   joint_goal[3] = -tau / 4
   joint_goal[4] = 0
   joint_goal[5] = tau / 6  # 1/6 оборота
   joint_goal[6] = 0
   ```

2. Используйте команду `go`, чтобы отправить значения сочленений роботу:
   ```python
   move_group.go(joint_goal, wait=True)
   ```

3. Вызовите метод `stop()` для предотвращения остаточного движения:
   ```python
   move_group.stop()
   ```

## Планирование движения к целевой позе

Вы можете запланировать движение группы к заданной целевой позе для конечного эффектора.

### Пример:

1. Укажите целевую позу:
   ```python
   pose_goal = geometry_msgs.msg.Pose()
   pose_goal.orientation.w = 1.0
   pose_goal.position.x = 0.4
   pose_goal.position.y = 0.1
   posepp_initialize_goal.position.z = 0.4

   move_group.set_pose_target(pose_goal)
   ```

2. Запустите планирование и выполнение:
   ```python
   # Метод `go()` возвращает True, если планирование и выполнение прошли успешно
   success = move_group.go(wait=True)
   
   # Останавливаем остаточное движение
   move_group.stop()
   
   # Очищаем цели после завершения планирования
   move_group.clear_pose_targets()
   ```

---

## Планирование декартового пути

Вы можете запланировать декартовый путь, задав список промежуточных точек, через которые должен пройти конечный эффектор.

### Пример:

1. Задайте точки траектории:
   ```python
   waypoints = []

   # Получаем текущую позу конечного эффектора
   wpose = move_group.get_current_pose().pose

   # Первая точка: движение вверх по оси z и в сторону по оси y
   wpose.position.z -= scale * 0.1  # Вверх на 0.1 м
   wpose.position.y += scale * 0.2  # В сторону на 0.2 м
   waypoints.append(copy.deepcopy(wpose))

   # Вторая точка: движение вперед/назад по оси x
   wpose.position.x += scale * 0.1  # Вперед на 0.1 м
   waypoints.append(copy.deepcopy(wpose))

   # Третья точка: движение в сторону по оси y
   wpose.position.y -= scale * 0.1  # В сторону на 0.1 м
   waypoints.append(copy.deepcopy(wpose))
   ```

2. Запланируйте декартовый путь:
   ```python
   # Планируем путь с разрешением 1 см (eef_step = 0.01)
   # Устанавливаем порог прыжка (jump_threshold) в 0.0, чтобы отключить проверку
   plan, fraction = move_group.compute_cartesian_path(
       waypoints, 0.01,  # Шаг конечного эффектора
       0.0  # Порог прыжка
   )
   ```

3. **Примечание**: на данном этапе мы только планируем траекторию, но не выполняем движение робота.

   ```python
   return plan, fraction
   ``` 

Теперь вы можете использовать запланированный путь для выполнения движения или дальнейшего анализа.

## Визуализация траектории

Вы можете попросить RViz визуализировать запланированную траекторию. Однако метод `group.plan()` уже делает это автоматически, поэтому повторная визуализация полезна, если вы хотите показать ту же траекторию повторно.

### Пример:

Сообщение `DisplayTrajectory` имеет два основных поля:
- **trajectory_start**: начальное состояние робота.
- **trajectory**: список траекторий.

Мы заполняем поле `trajectory_start` текущим состоянием робота, чтобы скопировать все присоединенные объекты, и добавляем наш план в `trajectory`.

```python
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)

# Публикация сообщения
display_trajectory_publisher.publish(display_trajectory)
```

---

## Выполнение плана

Для выполнения ранее рассчитанного плана используйте метод `execute()`:

```python
move_group.execute(plan, wait=True)
```

> **Примечание:** Текущее состояние сочленений робота должно находиться в пределах заданной погрешности от первой точки в `RobotTrajectory`, иначе `execute()` завершится с ошибкой.

---

## Добавление объектов в сцену планирования

Вы можете добавлять объекты в сцену планирования. Например, создадим коробку между пальцами робота:

### Пример:

```python
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "panda_hand"
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.z = 0.11  # Над рамой panda_hand
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))
```

Этот код добавляет коробку размером `7.5 x 7.5 x 7.5 см` над рамой `panda_hand`. Она теперь становится частью сцены планирования и может быть использована для задач, таких как манипуляция или избегание столкновений.

## Обеспечение обновлений столкновений

Если узел Python только что был создан или завершает работу до публикации сообщения об обновлении сцены, сообщение может быть потеряно, и объект не появится. Чтобы гарантировать, что обновления произошли, мы проверяем, отражены ли изменения в списках `get_attached_objects()` и `get_known_object_names()`. Для этого можно использовать следующий метод, который вызывается после добавления, удаления, присоединения или отсоединения объекта.

### Пример:

```python
start = rospy.get_time()
seconds = rospy.get_time()
while (seconds - start < timeout) and not rospy.is_shutdown():
    # Проверяем, прикреплен ли объект
    attached_objects = scene.get_attached_objects([box_name])
    is_attached = len(attached_objects.keys()) > 0

    # Проверяем, существует ли объект в сцене
    # Обратите внимание, что при прикреплении объект удаляется из списка известных объектов
    is_known = box_name in scene.get_known_object_names()

    # Проверяем, соответствует ли состояние ожиданиям
    if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

    # Пауза для других потоков
    rospy.sleep(0.1)
    seconds = rospy.get_time()

# Если цикл завершился без возврата, произошел тайм-аут
return False
```

> **Примечание:** Чтобы избежать ожидания обновлений сцены, инициализируйте интерфейс сцены с параметром `synchronous=True`.

---

## Присоединение объектов к роботу

Для манипуляции объектами необходимо разрешить роботу касаться их, чтобы сцена планирования не считала это столкновением. Это достигается добавлением имен звеньев в массив `touch_links`. Для робота Panda задайте `grasping_group` как `'hand'`. Для других роботов измените значение на имя группы вашего конечного эффектора.

### Пример:

```python
grasping_group = "panda_hand"
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(eef_link, box_name, touch_links=touch_links)
```

---

## Отсоединение объектов от робота

Вы можете отсоединить объект и удалить его из сцены планирования:

```python
scene.remove_attached_object(eef_link, name=box_name)
```

---

## Удаление объектов из сцены планирования

Вы также можете удалить объект из мира:

```python
scene.remove_world_object(box_name)
```

> **Примечание:** Перед удалением объекта из мира необходимо его отсоединить.

---

## Файл запуска

Полный файл запуска доступен в [репозитории на GitHub](https://github.com/ros-planning/moveit_tutorials).  
Весь код из этого урока можно запустить из пакета `moveit_tutorials`, который является частью вашей настройки MoveIt.