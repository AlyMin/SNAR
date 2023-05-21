# SNAR

Производится импорт библиотек и объявление глобальных переменных, изначально имеющих нулевые значения.

```python
#!/usr/bin/env python

import rospy
import math
from pr_5.msg import Robot_systems

encr = 0
encl = 0
w_lg = 0
w_rg = 0
teta_g = 0
x_g = 0
y_g = 0
t_g = 0
```
Функция funny() предназначена для подсчёта показаний энкодеров, а также координат робота. Она принимает две переменные: линейную и угловую скорости робота, изменяемые с помощью кнопок пользователем. Из полученных величин высчитываются угловые скорости левого и правого колеса. Далее вызывается функция wspeed(), которая возвращает разницу времени между последним показанием и текущим, пересчитанные угловые скорости каждого из колёс, а также изменения показаний энкодеров колёс. Подробнее о функции будет рассказано позже.
По изменениям показаний инкодеров вычисляются текущие выдаваемые ими значения. Заново расчитываются угловая и линейная скорость, а также считаются угол поворода и координаты робота x и y. Поскольку времени между измерениями проходит немного, нет необходимости применять интеграл, частые измерения соответствуют в некоторой степени методу трапеции для подсчёта интеграла.


```python
def funny(V, W):
    global t_g
    global teta_g
    global x_g
    global y_g
    global t_g
    global encr
    global encl
    global w_rg
    global w_lg
    L = 0.287
    r = 0.033
    #угловые скорости колёс по полученным данным
    wl = (V - 0.5 * L * W) / r
    wr = (V + 0.5 * L * W) / r
    #угловые скорости колёс после сау
    dt, wl, wr, dencl, dencr = wspeed(wl, wr)
    encl += dencl
    encr += dencr
    #посчитанные угловая и линейная скорости
    V = 0.5 * r * (wl + wr)
    W = r * (wr - wl) / L
    teta = W * dt
    x = V * math.cos(teta) * dt
    y = V * math.sin(teta) * dt
    x_g += x
    y_g += y
    teta_g += teta
    return t_g

def callback(data):
    funny(data.linear_vel, data.angular_vel)
    rospy.loginfo("Encoder_left %0.2f", encl)
    rospy.loginfo("Encoder_right %0.2f", encr)
    rospy.loginfo("Y: %0.2f", y_g)
    rospy.loginfo("X: %0.2f", x_g)

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("custom_chatter", Robot_systems, callback)
    t_g = float(rospy.get_time())
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

Функция wspeed() предназначена для вычисления угловой скорости в соответствии с показаниями энкодера. Она принимает угловые скорости левого и правого колёс, вычисляет, сколько прошло с предыдущего измерения. Далее высчитываются изменения показаний каждого из энкодеров, приводящиеся к целочисленным значениям, после чего пересчитываются угловые скорости. А также обновляется глобальное значение времени. Все посчитанные значения обновляются. 

```python
def wspeed(w_l, w_r):
    global t_g
    N = 4096
    t = float(rospy.get_time())
    dt = t - t_g
    #вычисление показаний энкодера
    dencl = int(w_l * dt * N / (2 * 3.1415))
    dencr = int(w_r * dt * N / (2 * 3.1415))
    #вычисление угловой скорости колеса
    wl = dencl * 2 * 3.1415 / dt / N
    wr = dencr * 2 * 3.1415 / dt / N
    t_g = t
    return dt, wl,wr,  dencl, dencr
```
