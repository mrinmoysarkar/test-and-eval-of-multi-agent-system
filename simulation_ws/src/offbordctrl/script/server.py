import socket
import os



if __name__ == "__main__":
    TCP_IP = '192.168.1.59'
    TCP_PORT = 7777
    BUFFER_SIZE = 20
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP,TCP_PORT))
    s.listen(1)
    while 1:
        conn, addr = s.accept()
        data = conn.recv(BUFFER_SIZE)
        if data:
            if data == 'estop':
               os.system("sudo shutdown now")
            elif data == 'init':
               os.system("roslaunch /home/intel1/ros_repo/ros_ws/src/offbordctrl/launch/ctrl_position.launch &")
            elif data == 'start':
                os.system("rosrun offbordctrl pub_trajectory &")
    conn.close()

