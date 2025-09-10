from serial_controller import *

import serial
import math as m
import matplotlib.pyplot as plt
import csv

#blah blah

try:
    # Initialize controller (change port as needed)
    controller = SerialController(port='COM3', debug=True)
    controller.start_listener_thread()

    # Test usage
    maximum = 15999
    periods = 4
    nextNum = 100
    flip = True
    pulseList = []
    for i in range(0, maximum):
        pulseList.append(100)
        #pulseList.append(int(math.floor(((50 * math.sin(math.radians(360 * (i/maximum) * periods)))) + 100)))
    #print(f"sendingData={pulseList}")
    print("Sending PWM pulses...")
    controller.send_pwm_pulses(pulseList)

    for i in range(3):
        response = controller.get_response(timeout=2.0)
        if response:
            msg_type, data = response
            print(f"Received response {i}: Type={msg_type}, length={len(data)}")
        else:
            print(f"none received {i}")

        if msg_type == MessageType.MSG_RETURN_RX:
            rx = data
        elif msg_type == MessageType.MSG_RETURN_ADC:
            adc = data
        elif msg_type == MessageType.MSG_RETURN_PID:
            pid = data

        time.sleep(0.1)

    plt.plot(rx)
    plt.plot(adc)
    plt.plot(pid)
    plt.show()

    data_rows = zip(rx, adc, pid)

    with open('playground_output.csv', 'w', newline='') as f:
        write = csv.writer(f)

         # Write header row
        write.writerow(['rx', 'adc', 'pid'])
        
        # Write the data rows
        write.writerows(data_rows)


except serial.SerialException as e:
    print(f"Serial error: {e}")

finally:
    controller.close()