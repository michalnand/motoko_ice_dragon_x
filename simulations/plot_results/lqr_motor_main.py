import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy


import json_serial_port_parser


max_length = 300

y_data = []

def animate(i): 
    while True:
        serial_port.process()
        if serial_port.udpated():

            json_data = serial_port.get()
            
            y = numpy.array(json_data["data"])

            y_data.append(y)

            if len(y_data) > max_length:
                del y_data[:max_length]

               
            axs[0].clear()
            axs[1].clear()

            axs[1].plot(numpy.array(y_data)[:, 1], label="controller output value", color="red")
            axs[1].set_ylabel("controller output value")
            #axs[1].set_ylim(-1.1, 1.1)
            
            axs[0].plot(numpy.array(y_data)[:, 0], label="required value", color="red")
            axs[0].plot(numpy.array(y_data)[:, 2], label="encoder reading", color="blue")
            axs[0].plot(numpy.array(y_data)[:, 3], label="kalman filter", color="green")
            axs[0].set_ylabel("rpm")    
            axs[0].set_ylim(0, 600)
            axs[0].legend(loc="upper left")

            break
        else:
            time.sleep(0.1)




if __name__ == "__main__":

    serial_port_name = "/dev/tty.usbserial-110"
    serial_port = json_serial_port_parser.JsonSerialPortParser(serial_port_name)

    fig, axs = plt.subplots(2, 1)


    ani = animation.FuncAnimation(fig, animate, interval=10)
    plt.show()