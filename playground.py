from serial_controller import *

import serial
import math as m
import matplotlib.pyplot as plt
import csv


try:
    # Initialize controller (change port as needed)
    controller = SerialController(port='COM3', debug=True)
    controller.start_listener_thread()

    # Test usage
    maximum = 15000
    periods = 4
    nextNum = 100
    flip = True
    pulseList = []
    for i in range(0, maximum):
        pulseList.append(int(math.floor(((50 * math.sin(math.radians(360 * (i/maximum) * periods)))) + 100)))
    #print(f"sendingData={pulseList}")
    print("Sending PWM pulses...")
    controller.send_pwm_pulses(pulseList)

    response = controller.get_response(timeout=2.0)
    if response:
        msg_type, data = response
        print(f"Received response {i}: Type={msg_type}, length={len(data)}")
    else:
        print(f"none received {i}")

    plt.plot(data)
    plt.show()

    with open('playground_output.csv', 'w', newline='') as f:

        write = csv.writer(f)
        for item in data:
            write.writerow([item])


except serial.SerialException as e:
    print(f"Serial error: {e}")

finally:
    controller.close()