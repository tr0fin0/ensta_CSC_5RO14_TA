# Validation

## Hardware

Before running any demonstration algorithms, it is necessary to initialize the *Kobuki Base* by running:

```bash
roslaunch kobuki_node minimal.launch
```

For each verification listed below, ensure that the `print` statements in the corresponding Python file are updated to Python 3 syntax (`print()`). This can be done by editing the file using:

```bash
nano ~/catkin_ws/src/kobuki/kobuki_testsuite/scripts/<file>.py
```

### Battery Voltage

```bash
rosrun kobuki_testsuite test_battery_voltage.py  
```

Show *Kobuki Base* battery voltage.

### Events

```bash
rosrun kobuki_testsuite test_events.py
```

Verify *Kobuki Base* hardware events such as:

- buttons
- bumpers
- cliffs
- plug / unplug adapter
- dock / undock base

### Gyroscope

```bash
rosrun kobuki_testsuite test_gyro.py
```

Show *Kobuki Base* gyroscope data.

### LEDs

```bash
rosrun kobuki_testsuite test_led_array.py
```

Show *Kobuki Base* different color combinations for LEDs 1 and 2. 

### Locomotion

```bash
rosrun kobuki_testsuite test_rotation.py
rosrun kobuki_testsuite test_translation.py
```

Verify *Kobuki Base* rotation and translation locomotion capabilities.

### Sounds

```bash
rosrun kobuki_testsuite test_sounds.py
```

Show *Kobuki Base* different sounds and contexts.
