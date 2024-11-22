# Скрипты MoveIt Commander

![MoveIt Commander Scripting](../assets/moveit/35_moveit_commander_scripting.png)

Пакет Python [`moveit_commander`](http://wiki.ros.org/moveit_commander) предоставляет удобные обертки для функций, доступных в MoveIt. Доступны простые интерфейсы для планирования движений, вычисления декартовых траекторий, а также операций захвата и размещения. Пакет `moveit_commander` также включает интерфейс командной строки — `moveit_commander_cmdline.py`.

---

## Запуск RViz и инструмента командной строки

1. Откройте два терминала.  
2. В первом терминале запустите **RViz** и дождитесь завершения загрузки:

   ```bash
   roslaunch panda_moveit_config demo.launch
   ```

3. Во втором терминале инициализируйте интерфейс `moveit_commander`:

   ```bash
   rosrun moveit_commander moveit_commander_cmdline.py
   ```

---

## Использование инструмента командной строки MoveIt Commander

Первой командой, которую вы должны ввести, будет:

```bash
use panda_arm
```

Где `panda_arm` — это имя группы, для которой вы хотите отправлять команды. Эта команда подключает вас к запущенному экземпляру узла `move_group`. Теперь вы можете выполнять команды для указанной группы.

### Работа с текущим состоянием группы

Команда `current` позволяет отобразить текущее состояние вашей группы:

```bash
current
```

---

### Сохранение состояния группы

Чтобы сохранить текущее состояние под определенным именем, используйте команду:

```bash
rec c
```

Эта команда сохранит текущие значения суставов вашей группы робота под именем `c`.

---

### Изменение значений суставов и движение робота

Вы можете использовать синтаксис, похожий на MATLAB, для изменения значений суставов. Например:

1. Скопируйте значения суставов состояния `c` в новую переменную `goal`:
   ```bash
   goal = c
   ```

2. Измените первое значение сустава в `goal` на `0.2` (или другое допустимое значение, не вызывающее столкновений):
   ```bash
   goal[0] = 0.2
   ```

3. Для выполнения движения используйте команду:
   ```bash
   go goal
   ```

---

### Визуализация плана перед выполнением

Вместо команды `go` вы можете спланировать и визуализировать движение перед выполнением:

1. Задайте новые значения суставов:
   ```bash
   goal[0] = 0.2
   goal[1] = 0.2
   ```

2. Запустите планирование:
   ```bash
   plan goal
   ```

3. Выполните движение:
   ```bash
   execute
   ```

Этот подход позволяет вам сначала визуализировать рассчитанный план движения в **RViz**, прежде чем его выполнять.

---

### Полезные команды

- Для отображения списка поддерживаемых команд введите:
   ```bash
   help
   ```

- Для выхода из интерфейса `moveit_commander` введите:
   ```bash
   quit
   ```

Теперь нужно обернуть все это в скрипт и получить так называемый `pipeline` планирования манипулятора.

---

### Скрипт MoveIt Commander

В данном разделе мы подробно разберем код, который использует библиотеку moveit_commander для управления манипулятором в ROS. Этот код генерирует случайные позы и выполняет их с помощью манипулятора.

```python
import rospy
import random
import moveit_commander
import geometry_msgs.msg
import sys
```

* `rospy` — библиотека для работы с ROS, которая позволяет взаимодействовать с узлами и сообщениями.
* `random` — используется для генерации случайных чисел, что позволяет создавать случайные позы.
* `moveit_commander` — библиотека для управления манипуляторами в ROS с помощью MoveIt.
* `geometry_msgs.msg` — содержит сообщения, необходимые для работы с геометрическими данными, такими как позы.
* `sys` — позволяет работать с аргументами командной строки.

Функция `generate_random_pose`

```python
def generate_random_pose():
   # Функция для генерации случайной позы
   # Возвращает позицию (x, y, z) и ориентацию (x, y, z, w) в виде списков

   position = [
      random.uniform(-0.5, 0.5),
      random.uniform(-0.5, 0.5),
      random.uniform(0.1, 0.5)
   ]
   
   orientation = [
      random.uniform(-1, 1),
      random.uniform(-1, 1),
      random.uniform(-1, 1),
      random.uniform(-1, 1)
   ]
   
   return position, orientation
```

* Эта функция генерирует случайные значения для позиции (x, y, z) и ориентации (в виде кватерниона) в заданных пределах.
* Позиция ограничена диапазоном от -0.5 до 0.5 для x и y, и от 0.1 до 0.5 для z, что позволяет избежать столкновений с полом.
* Ориентация генерируется в виде кватерниона, который используется для представления поворотов в 3D пространстве.

Основная функция `main`

```python
def main():
      rospy.init_node('moveit_random_pose', anonymous=True)

      moveit_commander.roscpp_initialize(sys.argv)

      robot = moveit_commander.RobotCommander()

      scene = moveit_commander.PlanningSceneInterface()

      group_arm = moveit_commander.MoveGroupCommander("panda_arm")

      group_hand = moveit_commander.MoveGroupCommander("panda_hand")

      num_random_poses = 5

      for _ in range(num_random_poses):
         position, orientation = generate_random_pose()

         pose_target = geometry_msgs.msg.Pose()
         pose_target.position.x = position[0]
         pose_target.position.y = position[1]
         pose_target.position.z = position[2]
   pose_target.orientation.x = orientation[0]
         pose_target.orientation.y = orientation[1]
         pose_target.orientation.z = orientation[2]
         pose_target.orientation.w = orientation[3]

         group_arm.set_pose_target(pose_target)

         plan = group_arm.plan()

         if isinstance(plan, tuple):
               success = plan[0]
               trajectory = plan[1]
         else:
               success = True
               trajectory = plan

         if success and len(trajectory.joint_trajectory.points) > 0:
               group_arm.execute(trajectory, wait=True)
         else:
               rospy.logwarn("Планирование не удалось для данной позы.")

         rospy.sleep(1)

      moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
   main()
```

Описание работы основной функции

* Инициализация узла: Создается ROS-узел с именем moveit_random_pose, что позволяет взаимодействовать с другими узлами в системе ROS.

* Инициализация MoveIt: `moveit_commander` инициализируется для управления манипулятором.

* Создание объектов:

   * `RobotCommander` — для получения информации о роботе.
   * `PlanningSceneInterface` — для взаимодействия с окружающей средой.
   * `MoveGroupCommander` — для управления манипулятором и захватом.

* Генерация и выполнение поз: В цикле генерируются случайные позы, которые затем устанавливаются как целевые для манипулятора. После этого планируется движение к целевой позе и выполняется, если планирование прошло успешно.

* Завершение работы: После выполнения всех поз, `moveit_commander` корректно завершает свою работу.

* Задание 1. Сделайте так, чтобы манипулятор перемещался на по случайно сгенерировнным точкам, а по заранее определенным вами.

* Задание 2. Модицифируйте код так, чтобы 