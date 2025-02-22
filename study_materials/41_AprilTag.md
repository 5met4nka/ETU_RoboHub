# Использование AprilTag с ROS

## Что такое AprilTag?

**AprilTag** — это система двоичных маркеров, которые можно легко и надежно детектировать с помощью камеры. Каждый маркер представляет собой черно-белый квадрат с уникальным двоичным кодом внутри. Эти маркеры используются в различных задачах компьютерного зрения, таких как:

- **Локализация и навигация роботов** — маркеры могут быть размещены в пространстве для обозначения позиций или ориентиров.
- **Калибровка камер** — маркеры служат точками привязки для точной настройки параметров камеры.
- **Виртуальная и дополненная реальность** — для определения положения объектов в реальном мире.
- **Манипуляция роботами** — для точного определения местоположения объектов, с которыми робот взаимодействует.

## Как работают AprilTag?

- Камера считывает изображение, на котором могут быть маркеры.
- Алгоритм обнаруживает прямоугольные контуры, соответствующие потенциальным маркерам.
- Распознанный маркер декодируется, используя его уникальный двоичный шаблон.
- Система определяет ID маркера, его координаты в пространстве и ориентацию.

AprilTag отличается высокой точностью, быстрым распознаванием и устойчивостью к шуму.

---

# Установка пакетов

Для работы с AprilTag в ROS нужно установить следующие пакеты. Выполните в терминале:

```bash
sudo apt install ros-noetic-apriltag ros-noetic-apriltag-ros
```

Эти пакеты содержат необходимые инструменты для детектирования маркеров и взаимодействия с ними через ROS.

Дополнительно, для работы с камерой глубины, мы должны добавить в наш пакет launch-файл запуска драйвера камеры. Содержимое файла можно найти по [ссылке](https://github.com/lasauveetgarde/Basics_of_ROS_for_IndustrialRobotics/blob/master/source/driver_realsense.launch), его нужно скопировать в файл `driver_realsense.launch` нашего пакета. Также для запуска камеры рекомендуется создать отдельный лаунч файл с названием `start_rs_camera.launch` и следующим содержимым:

```xml
<?xml version="1.0"?>
<launch>
  <group ns="rs_camera">
    <include file="$(find study_pkg)/launch/driver_realsense.launch">
      <arg name="tf_prefix" value="rs_camera" />
      <arg name="align_depth" value="true" /> 
      <arg name="linear_accel_cov" value="0.01" />
      <arg name="unite_imu_method" value="copy" />
      <arg name="depth_fps" value="30" />
      <arg name="color_fps" value="30" />
      <arg name="enable_gyro" value="true" />
      <arg name="enable_accel" value="true" />
      <arg name="initial_reset" default="false" />
    </include>

    <node pkg="nodelet" type="nodelet" name="rgbd_sync" args="load rtabmap_sync/rgbd_sync realsense2_camera_manager" output="screen">
        <remap from="rgb/image" to="color/image_raw" />
        <remap from="depth/image" to="aligned_depth_to_color/image_raw" />
        <remap from="rgb/camera_info" to="color/camera_info" />
        <param name="approx_sync" value="false" />
      </node>
  </group>
</launch>
```

---

# Создание и настройка лаунч-файла

## Лаунч-файл `detect_apriltag.launch`

Создайте файл в вашем пакете `study_pkg` (или другом пакете) в папке `launch`. Вставьте в него следующий код:

```xml
<launch>

    <!-- Настройка камеры -->
    <arg name="camera_name" default="/camera_rect" />
    <arg name="image_topic" default="image_rect" />

    <!-- Узел для непрерывного детектирования -->
    <node pkg="apriltag_ros" type="apriltag_ros_continuous_node" name="apriltag_ros_continuous_node" clear_params="true" output="screen">
        <!-- Переназначение топиков -->
        <remap from="image_rect" to="/color/image_raw" />
        <remap from="camera_info" to="/color/camera_info" />

        <param name="publish_tag_detections_image" type="bool" value="true" /><!-- default: false -->

        <!-- Загрузка параметров -->
        <rosparam command="load" file="$(find study_pkg)/config/settings.yaml" />
        <rosparam command="load" file="$(find study_pkg)/config/tags.yaml" />
    </node>

</launch>
```

---

# Настройка конфигурационных файлов

## Файл `settings.yaml`

Создайте в папке `config` файл `settings.yaml` и добавьте следующие настройки:

```yaml
tag_family:        'tag36h11' # Семейство маркеров (варианты: tagStandard52h13, tagStandard41h12, tag36h11 и т.д.)
tag_threads:       2          # Количество потоков для обработки изображений.
tag_decimate:      1.0        # Масштабирование изображения перед обработкой (1.0 — без изменений).
tag_blur:          0.0        # Размытие изображения для подавления шума (0 — отключено).
tag_refine_edges:  1          # Уточнение краев маркеров (1 — включено).
tag_debug:         0          # Режим отладки (0 — отключено).
max_hamming_dist:  2          # Максимальная дистанция Хэмминга (2 — хороший выбор для надежности).

# Другие параметры
publish_tf:        true       # Публикация данных в формате TF (для работы с пространственными координатами).
transport_hint:    "raw"      # Тип транспорта изображения (raw, compressed и т.д.).
```

### Подробный разбор параметров

- **`tag_family`**: Указывает семейство маркеров, которое будет детектироваться. Выбор зависит от используемых маркеров. Например, `tag36h11` — одно из наиболее распространенных семейств.
- **`tag_threads`**: Указывает, сколько потоков CPU будет использоваться для обработки. Увеличение значения может ускорить обработку, если есть свободные ресурсы.
- **`tag_decimate`**: Масштабирование изображения снижает нагрузку на процессор за счет уменьшения размера изображения перед анализом.
- **`tag_blur`**: Уровень размытия для подавления шума. Может быть полезен при работе с низкокачественными изображениями.
- **`tag_refine_edges`**: Включает более точное определение краев маркера, что повышает точность детектирования.
- **`tag_debug`**: Активирует режим отладки, показывающий внутренние данные обработки.
- **`max_hamming_dist`**: Ограничивает количество ошибок при чтении маркера. Чем меньше значение, тем меньше вероятность ложных срабатываний.

---

## Файл `tags.yaml`

Создайте файл `tags.yaml` для описания детектируемых маркеров:

```yaml
standalone_tags:
  [
    {id: 0, size: 0.15}, # Маркер с ID 0, размером 0.15 метра
    {id: 1, size: 0.15}, # Маркер с ID 1, размером 0.15 метра
  ]
```

### Подробный разбор параметров

- **`id`**: Уникальный идентификатор маркера.
- **`size`**: Размер стороны маркера в метрах. Точный размер необходим для правильного определения положения маркера в пространстве.

---

# Запуск системы

1. Убедитесь, что камера подключена и топики `/color/image_raw` и `/color/camera_info` доступны.
2. Запустите лаунч-файл:

```bash
roslaunch study_pkg detect_apriltag.launch
```

---

## Результат

После запуска:
- В консоли будут отображаться данные о детектированных маркерах.
- Изображение с выделенными маркерами будет опубликовано в топике `/tag_detections_image`.
- Данные о положении маркеров (позиция и ориентация) будут доступны в формате TF и опубликованы в топике `/tag_detections`.

---

## Задание

Запусти `rviz`. Далее, настрой отображение так, чтобы тэги были видны на изображении с камеры и в основной сцене с помощью инструментов визуализации `tf`.

## Заключение

В этом уроке мы изучили:
1. Основные принципы работы AprilTag.
2. Установку и настройку ROS-пакета для их детектирования.
3. Создание и настройку конфигурационных файлов.

Теперь вы можете использовать AprilTag для локализации, калибровки или других задач, требующих надежного обнаружения объектов.