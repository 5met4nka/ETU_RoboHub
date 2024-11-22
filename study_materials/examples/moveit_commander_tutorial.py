import rospy  # Импортируем библиотеку ROS для работы с узлами и сообщениями
import moveit_commander  # Импортируем библиотеку MoveIt для управления манипуляторами
import geometry_msgs.msg  # Импортируем сообщения для работы с геометрией (например, позы)
import sys  # Импортируем библиотеку sys для работы с аргументами командной строки

def predefined_poses():
    # Функция для возвращения заранее определенных поз
    poses = [
        # Каждая поза состоит из позиции (x, y, z) и ориентации (x, y, z, w)
        ([0.2, 0.0, 0.3], [0, 0, 0, 1]),  # Поза 1
        ([0.3, 0.2, 0.4], [0, 0, 0, 1]),  # Поза 2
        ([0.1, -0.2, 0.5], [0, 0, 0, 1]), # Поза 3
        ([0.0, 0.3, 0.2], [0, 0, 0, 1]),  # Поза 4
        ([0.4, 0.0, 0.1], [0, 0, 0, 1])   # Поза 5
    ]
    return poses  # Возвращаем список поз

def main():
    # Основная функция, выполняющаяся при запуске программы

    # Инициализация ROS-узла с именем 'moveit_random_pose'
    rospy.init_node('moveit_random_pose', anonymous=True)

    # Инициализация moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Создание объекта RobotCommander для управления роботом
    robot = moveit_commander.RobotCommander()
    
    # Создание объекта PlanningSceneInterface для работы с окружением
    scene = moveit_commander.PlanningSceneInterface()
    
    # Создание объекта MoveGroupCommander для управления манипулятором (в данном случае "panda_arm")
    group_arm = moveit_commander.MoveGroupCommander("panda_arm")
    
    # Создание объекта MoveGroupCommander для управления захватом (в данном случае "panda_hand")
    group_hand = moveit_commander.MoveGroupCommander("panda_hand")

    # Получаем заранее определенные позы
    poses = predefined_poses()

    # Цикл для выполнения заранее определенных поз
    for position, orientation in poses:
        # Создание объекта Pose для установки целевой позы
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = position[0]  # Установка x-координаты
        pose_target.position.y = position[1]  # Установка y-координаты
        pose_target.position.z = position[2]  # Установка z-координаты
        pose_target.orientation.x = orientation[0]  # Установка x компонента ориентации
        pose_target.orientation.y = orientation[1]  # Установка y компонента ориентации
        pose_target.orientation.z = orientation[2]  # Установка z компонента ориентации
        pose_target.orientation.w = orientation[3]  # Установка w компонента ориентации

        # Установка целевой позы для манипулятора
        group_arm.set_pose_target(pose_target)

        # Планирование движения к целевой позе
        plan = group_arm.plan()  # Получаем план

        # Проверка на успешность планирования
        if isinstance(plan, tuple):  # Если план является кортежем
            success = plan[0]  # Получаем статус (успех/неуспех)
            trajectory = plan[1]  # Получаем сам план (траекторию)
        else:
            success = True  # Если не кортеж, считаем, что планирование прошло успешно
            trajectory = plan  # В противном случае просто берем план

        # Проверяем, есть ли точки в плане
        if success and len(trajectory.joint_trajectory.points) > 0:  # Если планирование успешно и есть точки
            # Выполнение движения манипулятора по запланированной траектории
            group_arm.execute(trajectory, wait=True)  # Выполняем движение и ждем его завершения
        else:
            # Если планирование не удалось, выводим предупреждение
            rospy.logwarn(" Планирование не удалось для данной позы.")

        # Опционально: добавление задержки для визуализации
        rospy.sleep(1)  # Задержка в 1 секунду перед следующей итерацией

    # Завершение работы moveit_commander
    moveit_commander.roscpp_shutdown()  # Остановка работы MoveIt

# Проверяем, является ли данный файл основным модулем
if __name__ == "__main__":
    main()  # Запускаем основную функцию