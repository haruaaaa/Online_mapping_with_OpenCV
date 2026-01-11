#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, MoveTank, OUTPUT_D, OUTPUT_A, SpeedRPM
import socket
import threading
import struct # для работы с бинарными данными
import math
import time

    
class Integrator:
    def __init__(self, x0, T):
        self.val_prev = x0
        self.T = T
        self.integral = x0

    def update(self, val):
        self.integral += (val + self.val_prev) * self.T / 2
        self.val_prev = val
        return self.integral

class CountOdom:
    def __init__(self, r, B, T):
        self.x_integrator = Integrator(0, T)
        self.y_integrator = Integrator(0, T)
        self.theta_integrator = Integrator(0, T)
        self.r = r
        self.B = B
        self.theta = self.theta_integrator.integral

    def get_speed(self, wr: float, wl: float) -> tuple:
        v = (wr + wl) * self.r / 2
        dx = v * math.cos(self.theta)
        dy = v * math.sin(self.theta)
        dth = (wr - wl)  * self.r / self.B
        return (dx, dy, dth)

    def update(self, wr: float, wl: float):
        dx, dy, dth = self.get_speed(wr, wl)
        x, y, self.theta = self.x_integrator.update(dx), self.y_integrator.update(dy), self.theta_integrator.update(dth)
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        return (x, y, self.theta)


motor_d = LargeMotor(OUTPUT_D)
motor_a = LargeMotor(OUTPUT_A)

# Настройки сервера
HOST = '10.42.0.35'
PORT = 8089

# conn - объект соединения
# addr - адрес клиента
def handle_client(conn, addr):
    print("Conected:"+ str({addr}))
    try:
        r = 0.042
        B = 0.159
        T1 =  0.295
        T2 = 0.33
        
        countodome = CountOdom(r, B, T1)

        while True:
            start_time = time.time()
            # recv - получение данных с клиента (8 байт)
            data = conn.recv(8)

            if not data:
                break

            #'>ff' - два числа float
            linear, angular = struct.unpack('>ff', data)
            
            x, y, theta = countodome.update(motor_d.speed*math.pi/180, motor_a.speed*math.pi/180)

            
            #  motor_a.speed #левый
            #  motor_d.speed #правый

            # '>ff' - два числа float
            response = struct.pack('>fff', x, y, theta)
            conn.send(response)

            et = time.time()
            dt = T2 - (et - start_time)
            if dt < 0:
                print('warn')
            else:
                time.sleep(dt)


    # Ошибка соединения
    except ConnectionResetError:
        print("Off")
    # В любом случае выключаем моторы и закрываем соединение
    finally:
        motor_d.stop()
        motor_a.stop()
        conn.close()


def speed(conn, addr):
    while True:
        # recv - получение данных с клиента (8 байт)
        data = conn.recv(8)
        r = 0.042
        B = 0.159
        if not data:
            break

        #'>ff' - два числа float
        linear, angular = struct.unpack('>ff', data)
        left_speed = SpeedRPM((2*linear - B*angular) / (2*r) * 60/(2*math.pi))
        right_speed = SpeedRPM((2*linear + B*angular) / (2*r) * 60/(2*math.pi))

        motor_a.on(left_speed)
        motor_d.on(right_speed)

        # time.sleep(0.05)

def main():
    # Создаем TCP-сокет (для передачи данных между устройствами в сети)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

        # Для быстрого перезапуска сервера на том же порту

        # SOL_SOCKET - показывает что мы работаем на уровне сокета
        # SO_REUSEADDR - позволяет повторно использовать порт
        # 1 - опция включена
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Привязываем сокет к конкретному хосту и порту
        s.bind((HOST, PORT))
        # Переводит сокет в режим прослушивания входящих соединений
        s.listen()
        print("Port "+str({PORT}))

        try:
            
            #  Принимаем соединение
            conn, addr = s.accept()
            # Поток для обработки клиента
            thread = threading.Thread(target=handle_client, args=(conn, addr))
            speeds = threading.Thread(target=speed, args= (conn, addr))

            speeds.start()
            thread.start()

        # Обработка прерывания
        except KeyboardInterrupt:
            motor_d.stop()
            motor_a.stop()

if __name__ == "__main__":
    main()
