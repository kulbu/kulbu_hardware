# Kulbu

https://github.com/kulbu/kulbu/

# Kulbu Hardware

Based on Dave Coleman's [hardware interface boilerplate](https://github.com/davetcoleman/ros_control_boilerplate).

* GPIO controls direction.
* `VelocityJointInterface` connected to on-board PWM.

## PWM module setup

```
sudo modprobe pwm-meson npwm=2
sudo modprobe pwm-ctrl
gpio export 87 out
gpio export 88 out
```

## Usage

```
roslaunch kulbu_base real.launch
```
