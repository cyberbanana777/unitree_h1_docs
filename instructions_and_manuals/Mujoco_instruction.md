# Что есть Mujoco?
**MuJoCo** (что расшифровывается как **Mu**lti-**Jo**int **Co**ntacts with **F**riction) — это двигатель, который просчитывает все физические законы в твоей виртуальной лаборатории: гравитацию, трение, столкновения объектов и многое другое. Благодаря своей высочайшей точности и скорости он стал одним из главных инструментов для разработчиков роботов и исследователей искусственного интеллекта по всему миру.

Представь, что ты инженер, который проектирует робота-спасателя. Прежде чем создавать дорогого железного помощника в реальности, тебе нужно проверить, сможет ли он вообще ходить по обломкам и не упасть. Вот здесь на помощь и приходит **MuJoCo** — это мощный симулятор, который создаёт точную виртуальную копию любого физического тела (робота, машины, даже человека) и среды, в которой оно находится. По сути, это «цифровая песочница», где можно безопасно, быстро и дёшево проводить тысячи экспериментов: бросать, ронять, учить ходить и бегать виртуальные модели, не рискуя сломать настоящий прототип.

# Установка
На этом этапе, мы подразумеваем, что ROS2 у Вас уже установлен. Если же нет, то установите (необходимая версия дистрибутива Foxy). Инструкции можно найти [здесь](https://docs.ros.org/en/foxy/Installation.html).
Для программирования робота в Mujoco с использованием ROS2 нужно установить 3 модуля:
- нестандартные типы сообщений, которые используются в ПО реального робота Unitree H1;
- симулятор;
- надстройка над симулятором для взаимодействия с роботами Unitree.
## Custom msg
Чтобы установить пакет ROS2 с нестандартными типами сообщений от производителя нужно перейти по [ссылке]((https://github.com/unitreerobotics/unitree_ros2) и  выполнить всё, что написано до `Connect to Unitree robot`. После всех действий нужно выполнить команду:
```bash
echo "source ~/unitree_ros2/cyclonedds_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Далее проведём проверку корректности установки:
```bash
ros2 pkg list | grep "unitree"
```
Вывод данной команды, должен быть таким:
```text 
unitree_api
unitree_go
unitree_hg
```

## Unitree_sdk2
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```

## Mujoco simulator

### Устанавливаем зависимости
```bash
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```
### Скачиваем исходники Mujoco и собираем её
```bash
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
### Устанавливаем pip
```bash
sudo apt install python3-pip
```
### Устанавливаем библиотеку для взаимодействия python-mujoco
```bash
pip3 install mujoco pygame
```
### Проверка корректности установки
При запуске команды следующей команды должно открыться пустое окно симулятора.
```bash
simulate
```
Если окно открылось, то установка симулятора прошла успешно.

## Unitree Mujoco
### Установка 
```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```
### Проверка работы
```bash
cd unitree_mujoco/simulate_python
python3 ./unitree_mujoco.py
```

[Ссылка на оригинальную инструкцию, на всякий случай](https://github.com/unitreerobotics/unitree_mujoco)
# Добавление Unitree H1

# Управление с помощью ROS2

# Тонкости, которые не очевидны
