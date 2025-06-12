from serial_controller import *

import serial
import math as m
import matplotlib.pyplot as plt


try:
    # Initialize controller (change port as needed)
    controller = SerialController(port='COM3', debug=True)
    controller.start_listener_thread()

    # Test usage
    maximum = 15000
    nextNum = 100
    flip = False
    pulseList = []
    for i in range(0, maximum):
        pulseList.append(int(math.floor(nextNum)))

        if flip == False:
            nextNum += 0.05
        elif flip == True:
            nextNum -= 0.05
        
        if nextNum >= 200:
            flip = True
        elif nextNum <= 1:
            flip = False
        # if i%2 == 0:
        #     pulseList.append(50)
        # else:
        #     pulseList.append(150)
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


except serial.SerialException as e:
    print(f"Serial error: {e}")

finally:
    controller.close()