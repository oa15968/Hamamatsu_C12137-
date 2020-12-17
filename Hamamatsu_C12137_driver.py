import usb.core
import usb.util
import time
import array as arr
import sys
import numpy as np
from struct import pack, unpack
import matplotlib.pyplot as plt
from tkinter import *
from tkinter import ttk

np.set_printoptions(threshold=sys.maxsize)
counts = {}

def callback():
    counts.clear()
    plt.close()
    dummy = 0

def main():
    root = Tk()
    button = ttk.Button(root, text="RESTART")
    button.pack()
    button.config(command=callback)
    device = usb.core.find(idVendor=0x0661, idProduct=0x2917)
    if device is None:
        raise ValueError('Device not found')
    # print(device)
    device.reset()
    i = device[0].interfaces()[0].bInterfaceNumber
    if device.is_kernel_driver_active(i):
        device.detach_kernel_driver(i)
    # use the first/default configuration
    device.set_configuration()

    # first endpoint
    endpoint = device[0][(0, 0)][0]
    # print(endpoint)
    print('connecting hamamatsu.....')
    # read a data packet
    data = None
    data_new = None
    data_new_pre = None
    sort_list = False
    counts_msec_b1 = {}
    counts_msec_b2 = {}
    G_count_arr = []
    G_counter = 0
    start_time = time.time()
    #counts = {}
    counter = 0
    dummy = 0
    while True:
        try:
            if dummy == 0:
                for q in range(5):
                    data_init = device.read(
                        endpoint.bEndpointAddress, 2112, 100)
                    dummy += 1
                    print('dummy.....', dummy)
                    time.sleep(0.5)
            data = device.read(endpoint.bEndpointAddress, 2112, 100)
            arr_size = len(data)
            if ((arr_size == 2112) and (90 in data)):
                inx = data.index(90)
                for i in range(4):
                    if data[inx+i] == 90:
                        if i == 3:
                            sort_list = True
                dataN = []
                if sort_list:
                    for j in range(2112):
                        if ((inx+j) < 2112):
                            dataN.append(data[inx+j])
                        elif ((inx+j) >= 2112):
                            dataN.append(data[(inx+j)-2112])
            msec_b2 = round(dataN[8])
            msec_b1 = round(dataN[9])
            G_count_b2 = round(dataN[4])
            G_count_b1 = round(dataN[5])
            Temp_b2 = round(dataN[10])
            Temp_b1 = round(dataN[11])
            # print(G_count)
            #real_time= ((msec_b2 * 25.5) + (msec_b1/10))
            real_time_b = ((msec_b2 << 8) | msec_b1)/10

            if msec_b1 not in counts_msec_b1:
                counts_msec_b1[msec_b1] = 1
                #print('Real_time: ' + str(real_time) + 'Sec')
                print('Real_time: ' + str(real_time_b) + 'Sec')
                # print(dataN)
                #G_byte = G_count_b1 + (G_count_b2 * 255)
                G_byte = ((G_count_b2 << 8) | G_count_b1)
                G_count_arr.append(G_byte)
                G_len = len(G_count_arr)
                if G_len < 10:
                    G_count_cps = round((sum(G_count_arr)/G_len)*10)
                    print('Gamma_CPS: ' + str(G_count_cps))
                elif G_len == 10:
                    G_count_arr.pop(0)
                    G_count_cps = (sum(G_count_arr))
                    print('Gamma_CPS: ' + str(G_count_cps))
                #temp = 188.686 - (0.00348*((Temp_b2*255)+(Temp_b1)))
                temp_b = ((Temp_b2 << 8) | Temp_b1)
                temp = 188.686 - (0.00348*temp_b)
                print('Temperature: ' + str(float('%.2f' % temp)) + '[C]')
                adc_appd = []
                for l in range(16, 2016, 2):
                    # print(l)
                    adc_b2 = dataN[l]
                    adc_b1 = dataN[l+1]
                    #print(adc_b2, adc_b1)
                    if (adc_b1 or adc_b2 > 0):
                        #print(adc_b2, adc_b1)
                        adc_val = ((adc_b2*255) + adc_b1)  # KeV
                        adc_appd.append(adc_val)
                        counter += 1
                        # print(adc_val)
                        buff_swap = ((adc_b1 << 8) | adc_b2)
                        channel = (buff_swap >> 4) & 0x0FFF
                        if channel not in counts:
                            counts[channel] = 1
                            print(str(channel) + '     ' +
                                  str(counts[channel]))
                        else:
                            counts[channel] += 1
                            print(str(channel) + '     ' +
                                  str(counts[channel]))
                elapsed_time = time.time() - start_time
                if (elapsed_time > 1.0):
                    print('Gamma_cps:  ' + str(counter))
                    counter = 0
                    start_time = time.time()
                    spectrum = [(counts[i] if i in counts else 0)
                                for i in range(4096)]
                    ln, = plt.plot(spectrum, markersize=2, color="red")
                    axes = plt.gca()
                    axes.set_xlim([0, 4096])
                    axes.autoscale_view(None, False, True)
                    plt.ylabel('ereignisse')
                    plt.xlabel('Channel')
                    # plt.autoscale(enable=True, axis='both', tight=None)
                    plt.pause(0.001)
                    ln.remove()
                    # plt.show(block=True)
                    plt.draw()

            if msec_b2 not in counts_msec_b2:
                counts_msec_b2[msec_b2] = 1
                counts_msec_b1 = {}
                if msec_b2 == 255:
                    counts_msec_b2 = {}

            sort_list = False
            time.sleep(0.01)

        except usb.core.USBError as e:
            data = None
            if e.args == ('Operation timed out',):

                continue


if __name__ == '__main__':
    main()

