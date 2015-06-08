# Kulbu

https://github.com/kulbu/kulbu/

# Kulbu Hardware

* GPIO controls direction.
* `VelocityJointInterface` connected to on-board PWM.

## PWM module setup

```
sudo modprobe pwm-meson npwm=2
sudo modprobe pwm-ctrl
```

## Usage

```
roslaunch kulbu_base real.launch
```
