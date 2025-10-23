# Преобразование координат точки между системами координат суставов робота

## Введение

Для работы с преобразованиями координат в ROS 2 используется утилита `tf2_echo`. Она подписывается на систему трансформаций (TF) и выводит в консоль преобразование между двумя указанными системами отсчёта (фреймами).

**Важное условие:** Для работы утилиты робот должен публиковать описания своих фреймов и преобразований между ними в соответствующем топике, как это реализовать описано ниже.

## 1. Запуск системы преобразований

В репозитории [unitree_h1_visualization_ws](https://github.com/cyberbanana777/unitree_h1_visualization_ws) при запуске `show.launch.py` реализована система преобразований (трансформ) для робота Unitree H1. Установить репозиторий можно по инструкции в его описании.

### Варианты запуска публикации преобразований

*   **Полный запуск с визуализацией в Rviz** (используется для отладки и визуального контроля):
    ```bash
    ros2 launch completed_scripts_visualization show.launch.py mode:=without_hands launch_rviz:=True robot:=simulation
    ```

*   **Запуск без визуализации:**
    ```bash
    ros2 launch completed_scripts_visualization show.launch.py mode:=without_hands robot:=simulation
    ```

### Просмотр доступных аргументов launch-файла

Чтобы увидеть все доступные аргументы launch-файла и их значения по умолчанию (что позволяет гибко настроить запуск для разных ситуаций: работа с реальным роботом, симуляция и т.д.), используйте команду:
```bash
ros2 launch completed_scripts_visualization show.launch.py -s
```

### Принцип работы системы преобразований

Для публикации преобразований в указанном launch-файле запускаются две ноды: `robot_state_publisher`. 

`robot_state_publisher` использует URDF, указанный в параметре `robot_description`, и положения суставов из топика `joint_states` для расчёта прямой кинематики робота (преобразований между системами координат) и публикации результатов через `tf`.

### Проверка работоспособности преобразований

После запуска любого из вариантов убедитесь, что система преобразований работает корректно:

```bash
ros2 node list
```
В списке активных узлов должен присутствовать узел с названием `/robot_state_publisher`.

```bash
ros2 topic list
```
В списке топиков должны быть топики `/tf_static` и `/tf`.

## 2. Использование утилиты `tf2_echo`

### Формат команды:
```bash
ros2 run tf2_ros tf2_echo <исходный_фрейм> <целевой_фрейм>
```

### Пример использования:
Чтобы получить преобразование из системы координат таза (`pelvis`) в систему координат правого плеча (`right_shoulder_pitch_link`), выполните:

```bash
ros2 run tf2_ros tf2_echo pelvis right_shoulder_pitch_link
```

### Где найти названия фреймов?
*   В окне визуализации Rviz (в разделе TF)
*   В URDF-описании модели робота

### Интерпретация результата

После запуска команды в консоль начнёт выводиться информация примерно следующего вида:

```bash
At time 1761151063.357288585
- Translation: [0.005, -0.155, 0.430]
- Rotation: in Quaternion [-0.216, 0.000, 0.000, 0.976]
```

**Расшифровка данных:**

*   **Translation `[x, y, z]`** — вектор перемещения (в метрах) от исходного фрейма к целевому:
    *   **X = 0.005 м**: Правое плечо находится почти прямо по оси X от таза (слегка впереди)
    *   **Y = -0.155 м**: Отрицательное значение по оси Y означает, что плечо смещено влево от таза (в стандартной системе координат робота ось Y обычно направлена влево)
    *   **Z = 0.430 м**: Положительное значение указывает, что плечо находится значительно выше таза

*   **Rotation `[x, y, z, w]`** — ориентация, представленная в виде кватерниона:
    *   В данном примере ненулевые компоненты `x` и `w` указывают на поворот в основном вокруг оси X

### Важное замечание

Преобразования между фреймами публикуются постоянно и меняются в реальном времени по мере движения звеньев робота.

**Для получения статичных и корректных данных:** Установите робота (или его симуляционную модель) в нужное положение, дождитесь прекращения движений, и только затем снимите показания с помощью `tf2_echo`.

## 3. Матричный метод преобразования координат

### Общая формула преобразования

Формула преобразования точки из системы 1 в систему 2:

**P₂ = R × P₁ + t**

где:
- **P₁** = [x₁, y₁, z₁] - точка в исходной системе координат
- **P₂** = [x₂, y₂, z₂] - точка в целевой системе координат  
- **R** - матрица поворота 3×3
- **t** = [tₓ, tᵧ, t₂] - вектор перемещения

### Пример данных

```
Translation: t = [0.005, -0.155, 0.430]
Rotation: q = [-0.216, 0.000, 0.000, 0.976]  # [x, y, z, w]
Point: P₁ = [0.1, 0.2, 0.3]
```

### Преобразование кватерниона в матрицу поворота

#### Формула кватерниона:
q = [x, y, z, w] = [qₓ, qᵧ, q_z, q_w]

#### Матрица поворота из кватерниона:

```
    [1-2(qᵧ²+q₂²)   2(qₓqᵧ-q_wq₂)   2(qₓq₂+q_wqᵧ) ]
R = [2(qₓqᵧ+q_wq₂)   1-2(qₓ²+q₂²)   2(qᵧq₂-q_wqₓ) ]
    [2(qₓq₂-q_wqᵧ)   2(qᵧq₂+q_wqₓ)   1-2(qₓ²+qᵧ²) ]
```

#### Расчёт для примера:
```
qₓ = -0.216, qᵧ = 0.000, q₂ = 0.000, q_w = 0.976

R = [1.000  0.000  0.000]
    [0.000  0.907  0.422]
    [0.000 -0.422  0.907]
```

### Применение поворота

**P_rotated = R × P₁**

```
[1.000  0.000  0.000]   [0.1]   [0.1]
[0.000  0.907  0.422] × [0.2] = [0.907×0.2 + 0.422×0.3] = [0.1814 + 0.1266] = [0.308]
[0.000 -0.422  0.907]   [0.3]   [-0.422×0.2 + 0.907×0.3] = [-0.0844 + 0.2721] = [0.188]
```

**P_rotated = [0.1, 0.308, 0.188]**

### Применение вектора перемещения

**P₂ = P_rotated + t**

```
P₂ = [0.1, 0.308, 0.188] + [0.005, -0.155, 0.430]
P₂ = [0.105, 0.153, 0.618]
```

## 4. Преобразование координат с помощью ROS2 ноды

### Полный код ноды

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class RobustTFNode(Node):
    def __init__(self):
        super().__init__('robust_tf_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz
        self.get_logger().info("Robust TF node started")

    def timer_callback(self):
        try:
            result = self.convert_point([0.1, 0.2, 0.3], 'pelvis', 'right_shoulder_pitch_link') # 'pelvis' - система координат таза робота, 'right_shoulder_pitch_link' - система координат плечавого сустава, замените на желаемые после теста
            if result:
                self.get_logger().info(f"Transform: {result}")
            else:
                self.get_logger().warning("TF not available yet, but still running...")
        except Exception as e:
            self.get_logger().error(f"Callback error: {e} - BUT CONTINUING!")

    def convert_point(self, point, source_frame, target_frame):
        try:
            if self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                point_msg = PointStamped()
                point_msg.header.frame_id = source_frame
                point_msg.point.x, point_msg.point.y, point_msg.point.z = point
                new_point = do_transform_point(point_msg, transform)
                return [new_point.point.x, new_point.point.y, new_point.point.z]
            return None
        except:
            return None

def main():
    rclpy.init()
    node = RobustTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Построчное объяснение работы ноды

### **1. Импорты и зависимости**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
```

**Что это значит:**
- `rclpy` - основной модуль ROS 2 для Python
- `Node` - базовый класс для создания ROS 2 нод
- `PointStamped` - сообщение для точки с указанием системы координат
- `Buffer` и `TransformListener` - для работы с TF (трансформациями)
- `do_transform_point` - функция для применения трансформации к точке

---

### **2. Создание класса ноды**
```python
class RobustTFNode(Node):
    def __init__(self):
        super().__init__('robust_tf_node')
```

**Объяснение:**
- `RobustTFNode` - наш собственный класс ноды, наследуется от `Node`
- `super().__init__('robust_tf_node')` - вызывает конструктор родительского класса и задает имя ноды
- Имя ноды `'robust_tf_node'` будет видно в ROS 2 системе

---

### **3. Инициализация TF системы**
```python
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
```

**Как это работает:**
- `Buffer()` - создает буфер для хранения трансформаций
- `TransformListener()` - слушает топики TF и заполняет буфер
- 📊 **Аналогия**: Представьте что `Buffer` - это блокнот, а `TransformListener` - секретарь, который записывает в него все изменения положений объектов

---

### **4. Таймер и периодичность**
```python
self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz
```

**Что происходит:**
- `create_timer(0.5, callback)` - создает таймер, который вызывает функцию каждые 0.5 секунд
- **2 Hz** - 2 раза в секунду
- `timer_callback` - функция, которая будет выполняться по таймеру

---

### **5. Основной цикл - timer_callback**
```python
def timer_callback(self):
    try:
        result = self.convert_point([0.1, 0.2, 0.3], 'pelvis', 'right_shoulder_pitch_link')
        if result:
            self.get_logger().info(f"Transform: {result}")
        else:
            self.get_logger().warning("TF not available yet, but still running...")
    except Exception as e:
        self.get_logger().error(f"Callback error: {e} - BUT CONTINUING!")
```

**Пошагово:**
1. **Каждые 0.5 секунд** вызывается эта функция
2. **Пытается преобразовать** точку `[0.1, 0.2, 0.3]` из системы `pelvis` в `right_shoulder_pitch_link`
3. **Если успешно** - выводит результат
4. **Если нет** - предупреждает, но продолжает работу
5. **При любой ошибке** - пишет в лог, но НЕ останавливается

---

### **6. Функция преобразования координат**
```python
def convert_point(self, point, source_frame, target_frame):
    try:
        if self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            point_msg = PointStamped()
            point_msg.header.frame_id = source_frame
            point_msg.point.x, point_msg.point.y, point_msg.point.z = point
            new_point = do_transform_point(point_msg, transform)
            return [new_point.point.x, new_point.point.y, new_point.point.z]
        return None
    except:
        return None
```

**Детальный разбор:**

#### **Шаг 1: Проверка доступности трансформации**
```python
self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=1.0)
```
- **Проверяет**: Можно ли преобразовать из `source_frame` в `target_frame`?
- **timeout=1.0**: Ждет до 1 секунды
- **Возвращает**: `True` если трансформация доступна, `False` если нет

#### **Шаг 2: Получение трансформации**
```python
transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
```
- **Получает** математическое преобразование между системами координат
- 📍 **Важно порядок**: `target_frame`, `source_frame` - "куда", "откуда"

#### **Шаг 3: Создание точки для преобразования**
```python
point_msg = PointStamped()
point_msg.header.frame_id = source_frame
point_msg.point.x, point_msg.point.y, point_msg.point.z = point
```
- `PointStamped()` - создает сообщение "точка с меткой времени и системой координат"
- `header.frame_id` - указывает в какой системе координат находится точка

#### **Шаг 4: Применение трансформации**
```python
new_point = do_transform_point(point_msg, transform)
```
- **Математически применяет** трансформацию к точке
- **Возвращает** ту же точку, но в новой системе координат
   

## Инструкция по запуску ноды

### 1. Создание ROS2 пакета

Создайте ROS2 пакет `simple_tf_demo` в папке `/src` вашей `_ws` или откройте уже апку уже сущесвтующего пакета `<ваш_пакет>/<ваш_пакет>`, создайте файл `simple_tf_node.py`. 

Подробнее о создании ROS2 пакета и его структуре смотрите в методическом указании [6_3_Публикатор_и_подписчик](https://github.com/cyberbanana777/unitree_h1_docs/blob/main/instructions_and_manuals/6_3_Публикатор_и_подписчик.md).

### 2. Настройка файла package.xml

После тега `<license></license>` добавьте:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
```

### 3. Настройка файла setup.py

Укажите точку входа (предполагается, что пакет называется `simple_tf_demo`, а файл - `simple_tf_node.py`):

```python
entry_points={
    'console_scripts': [
        'simple_tf_node = simple_tf_demo.simple_tf_node:main',
    ],
},
```

### 4. Сборка и запуск

```bash
colcon build
source install/setup.bash
```

### 5. Запуск в реальном сценарии

```bash
# Terminal 1 - Запускаем систему трансформаций (с визуализацией в rviz)
ros2 launch completed_scripts_visualization show.launch.py mode:=without_hands launch_rviz:=True robot:=simulation
```
```bash
# Terminal 2 - Запускаем нашу ноду
ros2 run simple_tf_demo simple_tf_node
```

## 🔄 **Пример работы ноды:**

```
[INFO] [1761251684.294985257] [continuous_tf_node]: Continuous TF node started - will keep running even with errors
[INFO] [1761251684.390243757] [continuous_tf_node]: [1] Transform OK: [0.0945, 0.3769926948296918, 0.032365578089181335]
[INFO] [1761251686.487127476] [continuous_tf_node]: [22] Transform OK: [0.0945, 0.3769926948296918, 0.032365578089181335]
[INFO] [1761251688.488670306] [continuous_tf_node]: [42] Transform OK: [0.0945, 0.3769926948296918, 0.032365578089181335]
[INFO] [1761251690.587000672] [continuous_tf_node]: [63] Transform OK: [0.0945, 0.3769926948296918, 0.032365578089181335]
^C[INFO] [1761251691.009605015] [continuous_tf_node]: Node stopped by user
```

**Нода будет работать всегда**, пока вы не остановите ее вручную.