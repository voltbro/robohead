# 🤖 Пакет robohead для роботизированной головы Bbrain 1.0 

**Пакет robohead** — это модульная ROS-платформа для управления роботизированной головой. Проект включает драйверы, контроллеры и голосовые интерфейсы для образовательных и исследовательских целей.

## 📁 Структура проекта

- `display_driver/` — управление круглым дисплеем (например, отображение лиц или информации).
- `ears_driver/` — управление сервоприводами ушей.
- `neck_driver/` — управление сервоприводами шеи.
- `respeaker_driver/` — интеграция с микрофонным массивом ReSpeaker.
- `speakers_driver/` — управление аудиовыходом.
- `sensor_driver/` — обработка данных с датчика тока INA219.
- `voice_recognizer_pocketsphinx/` — распознавание речи с использованием PocketSphinx.
- `robohead_controller/` — центральный пакет, который на основе распознанных аудио-команд управляет робо-головой. 
- `setupOS/` — скрипты и инструкции по настройке операционной системы.

## 🚀 Быстрый старт

Пакет `robohead` по умолчанию входит в сервис `robohead.service`, который загружается на робо-голове при её включении. Этот сервис запускает скрипт `~/start.sh`, который уже запускает основной launch-файл
`~/robohead_ws/src/robohead_controller/launch/robohead_controller_py.launch`

> [!NOTE]
> **После загрузки робоголовы все ROS-пакеты запускаются автоматически!**

### Подача голосовой команды

Для подачи голосовой команды необходимо произнести ключевую фразу:

```
Слушай, Робот!
```

После этого устройство перейдёт в режим распознавания команд. Базовые команды по умолчанию:

1. Поздоровайся  
2. Покажи уши  
3. Покажи левое ухо  
4. Покажи правое ухо  
5. Осмотрись
6. Покажи напряжение

## Конвертация файл-словаря в фонетический словарь для новой голосовой команды

```
cd ~/robohead_ws/src/ru4sphinx/text2dict/

./dict2transcript.pl /home/pi/robohead_ws/src/robohead/robohead_controller/config/voice_recognizer_pocketsphinx/dictionary.txt /home/pi/robohead_ws/src/robohead/robohead_controller/config/voice_recognizer_pocketsphinx/dictionary.dict
```

## Получение файлов озвучки для робо-головы

Сайт: [VoiceBot](https://voicebot.su/)

### Параметры 

- **Голос**:
  - Антон
- **Скорость**:
  - 0.9
- **Высота**:
  - 0.0
- **Громкость**:
  - 0 dB
- **Эмоции**:
  - Радостный


### Тестирование отдельных пакетов

Если вы хотите протестировать какие-то ROS-пакеты отдельно, то для этого необходимо:

1. Остановить сервис:

```bash
sudo systemctl stop robohead.service
```

2. Запустить нужный пакет, например:

```bash
roslaunch ears_driver ears_driver_py.launch
```

3. После завершения теста:

```bash
sudo systemctl start robohead.service
```


## 🚀 Разворачивание образа (системное)
 
### 1. Клонирование репозитория

```bash
cd ~/robohead_ws/src
git clone https://github.com/voltbro/robohead.git
cd ..
catkin_make
```

### 2. Установка зависимостей для пакета voice_recognizer_pocketsphinx

Убедитесь, что установлены все необходимые зависимости для каждого пакета. Например, для `voice_recognizer_pocketsphinx` потребуется установить PocketSphinx и соответствующие языковые модели.

```
cd ~/robohead_ws/src/robohead/voice_recognizer_pocketsphinx/config
```

#### 2.1 Скачивание языковой модели

> [!NOTE]
> Иногда быстрее бывает скачать файл по ссылке на ноутбук и перекинуть через FileZilla на робо-голову.

```
wget https://downloads.sourceforge.net/project/cmusphinx/Acoustic%20and%20Language%20Models/Russian/zero_ru_cont_8k_v3.tar.gz

tar -xf zero_ru_cont_8k_v3.tar.gz

rm -r zero_ru_cont_8k_v3.tar.gz
```
