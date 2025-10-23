# Как преобразовать координаты между системами отсчёта робота

Для работы с преобразованиями координат в ROS 2 используется утилита `tf2_echo`. Она подписывается на систему трансформаций (TF) и выводит в консоль преобразование между двумя указанными системами отсчёта (фреймами).

**Важное условие:** Для работы утилиты робот должен публиковать описания своих фреймов и преобразований между ними в соответствующем топике.

### 1. Запуск системы преобразований

В репозитории [unitree_h1_visualization_ws](https://github.com/cyberbanana777/unitree_h1_visualization_ws) при запуске `show.launch.py` реализована система преобразований (трансформ) для робота Unitree H1. Установить репозиторий можно по инструкции в его описании.

Запустить публикацию преобразований можно в нескольких вариантах (подробнее в разделе ["Использование"](https://github.com/cyberbanana777/unitree_h1_visualization_ws/tree/main?tab=readme-ov-file#-использование)):

*   **Полный запуск с визуализацией в Rviz:** Используется для отладки и визуального контроля.
    ```bash
    ros2 launch h1_description description.launch.py mode:=without_hands launch_rviz:=True
    ```

*   **Запуск без визуализации:**
    ```bash
    ros2 launch h1_description description.launch.py mode:=without_hands
    ```

Чтобы увидеть все доступные аргументы launch-файла и их значения по умолчанию (что позволяет гибко настроить запуск для разных ситуаций: работа с реальным роботом, симуляция и т.д.), используйте команду:
```bash
ros2 launch h1_description description.launch.py -s
```
Для публикации преобразований в указанном выше launch-файле запускаяются две ноды: `robot_state_publisher` и `robot_state_publisher`. 
`robot_state_publisher` использует URDF, указанный в параметре `robot_description` и положения суставов из топика `joint_states`, публикуемого `robot_state_publisher`, для расчёта прямой кинематики робота (перобразований между системами координат) и публикации результатов через `tf`.

**Проверка работоспособности преобразований**

После запуска любого из вариантов убедитесь, что система преобразований работает корректно. Для этого выполните команду:
```bash
ros2 node list
```
В списке активных узлов должен присутствовать узел с названием `/robot_state_publisher`.

При запуске этой команды:

```bash
ros2 topic list
```

В списке топиков должены быть топики `/tf_static` и `/tf`.

### 2. Использование утилиты `tf2_echo`

**Формат команды:**
```bash
ros2 run tf2_ros tf2_echo <исходный_фрейм> <целевой_фрейм>
```

**Пример:**
Чтобы получить преобразование из системы координат таза (`pelvis`) в систему координат правого плеча (`right_shoulder_pitch_link`), выполните:

```bash
ros2 run tf2_ros tf2_echo pelvis right_shoulder_pitch_link
```

**Где найти названия фреймов?**
*   В окне визуализации Rviz (в разделе TF).
*   В URDF-описании модели робота.

#### 3. Как интерпретировать результат

После запуска команды в консоль начнёт выводиться информация примерно следующего вида:

```bash
At time 1761151063.357288585
- Translation: [0.005, -0.155, 0.430]
- Rotation: in Quaternion [-0.216, 0.000, 0.000, 0.976]
```

**Расшифровка данных:**

*   **Translation `[x, y, z]`** — это вектор перемещения (в метрах) от исходного фрейма к целевому.
    *   **X = 0.005 м**: Правое плечо находится почти прямо по оси X от таза (слегка впереди).
    *   **Y = -0.155 м**: Отрицательное значение по оси Y означает, что плечо смещено влево от таза (в стандартной системе координат робота ось Y обычно направлена влево).
    *   **Z = 0.430 м**: Положительное значение указывает, что плечо находится значительно выше таза.

*   **Rotation `[x, y, z, w]`** — это ориентация, представленная в виде кватерниона.
    *   В данном примере ненулевые компоненты `x` и `w` указывают на поворот в основном вокруг оси X.

#### Важное замечание

Преобразования между фреймами публикуются постоянно и меняются в реальном времени по мере движения звеньев робота.

**Для получения статичных и корректных данных:** Установите робота (или его симуляционную модель) в нужное положение, дождитесь прекращения движений, и только затем снимите показания с помощью `tf2_echo`.

## Преобразование точки из одной системы координат в другую, матричный метод

Общая формула преобразования точки из системы A в систему B:

**P₂ = R × P₁ + t**

где:
- **P₁** = [x₁, y₁, z₁] - точка в исходной системе координат
- **P₂** = [x₂, y₂, z₂] - точка в целевой системе координат  
- **R** - матрица поворота 3×3
- **t** = [tₓ, tᵧ, t₂] - вектор

## 2. Данные

```
Translation: t = [0.005, -0.155, 0.430]
Rotation: q = [-0.216, 0.000, 0.000, 0.976]  # [x, y, z, w]
Point: P₁ = [0.1, 0.2, 0.3]
```

## 3. Преобразование кватерниона в матрицу поворота

### Формула кватерниона:
q = [x, y, z, w] = [qₓ, qᵧ, q₂, q_w]

### Матрица поворота из кватерниона:

```
    [1-2(qᵧ²+q₂²)   2(qₓqᵧ-q_wq₂)   2(qₓq₂+q_wqᵧ) ]
R = [2(qₓqᵧ+q_wq₂)   1-2(qₓ²+q₂²)   2(qᵧq₂-q_wqₓ) ]
    [2(qₓq₂-q_wqᵧ)   2(qᵧq₂+q_wqₓ)   1-2(qₓ²+qᵧ²) ]
```

### Для примера кватерниона q = [-0.216, 0.000, 0.000, 0.976]:

```
qₓ = -0.216, qᵧ = 0.000, q₂ = 0.000, q_w = 0.976

R = [1-2(0+0)       2(0-0)          2(0+0)        ]
    [2(0+0)         1-2(0.0467+0)   2(0-(-0.211)) ]
    [2(0-0)         2(0+(-0.211))   1-2(0.0467+0) ]

R = [1.000  0.000  0.000]
    [0.000  0.907  0.422]
    [0.000 -0.422  0.907]
```

## 4. Применение поворота

**P_rotated = R × P₁**

```
[1.000  0.000  0.000]   [0.1]   [0.1]
[0.000  0.907  0.422] × [0.2] = [0.907×0.2 + 0.422×0.3] = [0.1814 + 0.1266] = [0.308]
[0.000 -0.422  0.907]   [0.3]   [-0.422×0.2 + 0.907×0.3] = [-0.0844 + 0.2721] = [0.188]
```

**P_rotated = [0.1, 0.308, 0.188]**

## 5. Применение вектора перемещения

**P₂ = P_rotated + t**

```
P₂ = [0.1, 0.308, 0.188] + [0.005, -0.155, 0.430]
P₂ = [0.105, 0.153, 0.618]
```

## Преобразование точки из одной системы координат в другую с помщью ROS2 ноды

### **Полный код ноды**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class SimpleTFNode(Node):
    def __init__(self):
        super().__init__('simple_tf_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF node started")

    def convert_point(self, point, source_frame, target_frame):
        try:
            # Получаем трансформацию
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame, 
                rclpy.time.Time()
            )
            
            # Создаем точку для преобразования
            point_msg = PointStamped()
            point_msg.header.frame_id = source_frame
            point_msg.point.x = point[0]
            point_msg.point.y = point[1]
            point_msg.point.z = point[2]
            
            # Применяем трансформацию
            new_point = do_transform_point(point_msg, transform)
            
            return [new_point.point.x, new_point.point.y, new_point.point.z]
            
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None

def main():
    rclpy.init()
    node = SimpleTFNode()
    
    # Пример использования
    original_point = [1.0, 0.5, 0.2]
    
    result = node.convert_point(
        original_point,
        'frame_a',  # исходный фрейм
        'frame_b'   # целевой фрейм
    )
    
    if result:
        node.get_logger().info(f"Original: {original_point}")
        node.get_logger().info(f"Transformed: {result}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## **Построчное объяснение работы**

### **1. Импорты**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
```
- **`rclpy`** - основной модуль ROS2 для Python
- **`Node`** - базовый класс для создания ROS2-нод
- **`PointStamped`** - тип сообщения для точки с указанием системы координат и времени
- **`Buffer`, `TransformListener`** - компоненты TF2 для работы с преобразованиями координат
- **`do_transform_point`** - функция для применения трансформации к точке

### **2. Инициализация ноды**
```python
class SimpleTFNode(Node):
    def __init__(self):
        super().__init__('simple_tf_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF node started")
```
- **`super().__init__('simple_tf_node')`** - создаем ноду с именем `simple_tf_node`
- **`tf_buffer`** - буфер, который хранит историю трансформаций между фреймами
- **`tf_listener`** - слушатель, который автоматически заполняет буфер актуальными трансформациями
- **`get_logger().info()`** - вывод информационного сообщения при запуске

### **3. Метод преобразования точки**
```python
def convert_point(self, point, source_frame, target_frame):
    try:
        # Получаем трансформацию
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame, 
            rclpy.time.Time()
        )
```
- **`lookup_transform(target, source, time)`** - запрос трансформации:
  - `target_frame` - целевая система координат (куда преобразуем)
  - `source_frame` - исходная система координат (откуда преобразуем)
  - `rclpy.time.Time()` - время трансформации (0 = последняя доступная)

```python
        # Создаем точку для преобразования
        point_msg = PointStamped()
        point_msg.header.frame_id = source_frame
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = point[2]
```
- Создаем сообщение **`PointStamped`** - точка с "штампом":
  - `header.frame_id` - указываем систему координат точки
  - `point.x, y, z` - координаты точки в исходной системе

```python
        # Применяем трансформацию
        new_point = do_transform_point(point_msg, transform)
        
        return [new_point.point.x, new_point.point.y, new_point.point.z]
```
- **`do_transform_point(point, transform)`** - применяет матрицу трансформации к точке
- Возвращаем новые координаты в виде списка `[x, y, z]`

### **4. Обработка ошибок**
```python
    except Exception as e:
        self.get_logger().error(f"Transform failed: {e}")
        return None
```
- Ловим исключения если:
  - Трансформация между фреймами не найдена
  - Фреймы не существуют
  - Истек таймаут ожидания

### **5. Основная функция**
```python
def main():
    rclpy.init()
    node = SimpleTFNode()
    
    # Пример использования
    original_point = [1.0, 0.5, 0.2]
    
    result = node.convert_point(
        original_point,
        'frame_a',  # исходный фрейм
        'frame_b'   # целевой фрейм
    )
```
- **`rclpy.init()`** - инициализация ROS2
- Создаем экземпляр ноды
- Задаем тестовую точку `[1.0, 0.5, 0.2]`
- Вызываем преобразование из `frame_a` в `frame_b`

---

## 🚀 **Как запустить ноду**

СОздайте ROS2 пакет `simple_tf_demo` или откройте уже сущесвтуеющий и в папке /src
Создайте файл `simple_tf_node.py`. 
ПОдробнее о создении ROS2 пакета и его структуре смотри в мемтодическом указании 6.3 Публикатор и подписчик.

 **2. В файле package.xml**

После тега `<license></license>` добавьте

```xml

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  
```

### **3. В файле setup.py**

Не забудте указать точку входа, например:
`simple_tf_demo` - пакет
`simple_tf_node` - название файла

```python

    entry_points={
        'console_scripts': [
            'simple_tf_node = simple_tf_demo.simple_tf_node:main',
        ],
    },
)
```

### **4. Сборка и запуск**
```bash
colcon build
source install/setup.bash
```

### **5. Запуск в реальном сценарии**
```bash
# Terminal 1 - Запускаем систему трансформаций (с визуализацией в rviz)
ros2 launch h1_description description.launch.py mode:=without_hands launch_rviz:=True

# Terminal 2 - Запускаем нашу ноду
ros2 run simple_tf_demo simple_tf_node
```

## 📊 **Ожидаемый вывод**
```
[INFO] [simple_tf_node]: TF node started
[INFO] [simple_tf_node]: Original: [1.0, 0.5, 0.2]
[INFO] [simple_tf_node]: Transformed: [2.0, 0.5, 0.2]
```
(Точка сместилась по X на 1 метр, так как мы задали смещение между frame_a и frame_b)