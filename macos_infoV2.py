#MAC OS M1 system status info gethering script
#version 2.2, by zell, 2023.9.19
#improved uart send functions added lite mode for testing
#version 2.1, by zell, 2023.9.16
#updated CPU GPU freq info

#sudo python3 macos_infoV2.py
# Install the following Python modules:
# pip install pyserial
# pip install psutil
#
# or pip install -r requirements_mac.txt

# On Mac's with the CPU and GPU in the same dye, only the CPU temperature is available
# so pass the CPU temp to GPU temp

import serial
import os
import time
import psutil

updateTime = 2 #number of seconds between each update
condition = 1

def ReadData():
    try:
        #connection = serial.Serial('/dev/tty.MyDisplay-ESP32SPP')
        connection = serial.Serial('/dev/tty.usbserial-0203CE1B',115200, timeout=1)
        #data = temp + ',' + rpm + ',' + str(free_mem) + ',' + str(free_disk) + ',' + gpu + ',' + str(procs) + '/'
        tmp_buf=connection.read(24)
        #connection.write(data.encode())
        print(">>: Data read", tmp_buf.decode())
        #connection.close  
    except Exception as e:
        print(e)
        condition =0
        
def sendData_lite(GPU_F, ECPU_F, PCPU_F, free_disk, free_mem, proc_counter):
    try:
        #connection = serial.Serial('/dev/tty.MyDisplay-ESP32SPP')
        connection = serial.Serial('/dev/tty.usbserial-0203CE1B',115200, timeout=1)
        data = GPU_F + ',' + ECPU_F + ',' + PCPU_F + ',' +str(free_mem) + 'MB/n'
        connection.write(data.encode())
        print("#>: Data written", data.encode())
        #connection.close  
    except Exception as e:
        print(e)
        condition =0
        
def sendData(GPU_F, ECPU_F, PCPU_F, free_disk, free_mem, proc_counter):
    try:
        #connection = serial.Serial('/dev/tty.MyDisplay-ESP32SPP')
        connection = serial.Serial('/dev/tty.usbserial-0203CE1B',115200, timeout=1)
        data = GPU_F + ',' + ECPU_F + ',' + PCPU_F + ',' +str(free_mem) + 'G,' + str(free_disk) + 'G,' + str(proc_counter) + '/'
        connection.write(data.encode())
        print("#>: Data written", data.encode())
        connection.close  
    except Exception as e:
        print(e)
        condition =0


def sendData_uart(temp, rpm, gpu, free_disk, free_mem, procs):
    try:
        connection = serial.Serial('/dev/tty.usbmodemUiFlow2_1')
        data = temp + ',' + rpm + ',' + str(free_mem) + ',' + str(free_disk) + ',' + gpu + ',' + str(procs) + '/'
        connection.write(data.encode())
        print("Data written", data.encode())
        connection.close  
    except Exception as e:
        print(e)
        condition=0

#smc not supported for my test
def CPU_Temp():
    cpu_temp = [each.strip() for each in (os.popen('sudo powermetrics --samplers smc -i1 -n1')).read().split('\n') if each != '']
    for line in cpu_temp:
        if 'CPU die temperature' in line:
            return line.strip('CPU die temperature: ').rstrip(' C')
    return 'n/a'

def GPU_Temp():
    # Will not work with Integrated Intel GPU
    gpu_temp = [each.strip() for each in (os.popen('sudo powermetrics --samplers smc -i1 -n1')).read().split('\n') if each != '']
    for line in gpu_temp:
        if 'GPU die temperature' in line:
            return line.strip('GPU die temperature: ').rstrip(' C')
    return 'n/a'

def FAN_Speed():
    gpu_temp = [each.strip() for each in (os.popen('sudo powermetrics --samplers smc -i1 -n1')).read().split('\n') if each != '']
    for line in gpu_temp:
        if 'Fan' in line:
            return line.strip('Fan: ').rstrip(' rpm')
    return 'n/a'
    
def ECPU_Freq():
    CPU_freq = [each.strip() for each in (os.popen('sudo powermetrics --samplers cpu_power -i1 -n1')).read().split('\n') if each != '']
    for line in CPU_freq:
        if 'E-Cluster HW active frequency' in line:
            return line.strip('E-Cluster HW active frequency: ')
    return 'n/a'
    
def PCPU_Freq():
    CPU_freq = [each.strip() for each in (os.popen('sudo powermetrics --samplers cpu_power -i1 -n1')).read().split('\n') if each != '']
    for line in CPU_freq:
        if 'P-Cluster HW active frequency' in line:
            return line.strip('P-Cluster HW active frequency: ')
    return 'n/a'
    
    
def GPU_Freq():
    GPU_freq = [each.strip() for each in (os.popen('sudo powermetrics --samplers gpu_power -i1 -n1')).read().split('\n') if each != '']
    for line in GPU_freq:
        if 'GPU HW active frequency' in line:
            return line.strip('GPU HW active frequency: ')
    return 'n/a'


print("This is a python3 script to gether PC status info(CPU,Mem..,) and send over serial port ")
condition=1
while(condition):
    GPU_F= GPU_Freq()
    ECPU_F = ECPU_Freq()
    PCPU_F = PCPU_Freq()
    #temp = CPU_Temp()
    #rpm = FAN_Speed()
    obj_Disk = psutil.disk_usage('/')
    free_disk = int(obj_Disk.free / (1000.0 ** 3))
    free_mem = int(int(psutil.virtual_memory().total - psutil.virtual_memory().used)/ (1024 * 1024)) 
    print("E-Cluster CPU frequency="+str(ECPU_F))
    print("P-Cluster CPU frequency="+str(PCPU_F))
    print("GPU HW active frequency="+str(GPU_F))
    #print("CPU_Temp="+str(temp))
    #print("FAN_RPM="+str(rpm))
    print("free_disk="+str(free_disk)+" G")
    #print("free_disk="+str(free_disk)+"G, disk_usage="+str(obj_Disk))
    print("free_mem="+str(free_mem)+"MB")
    print("")
    proc_counter = 0
    for proc in psutil.process_iter():
        proc_counter += 1
    #sendData_uart(temp, rpm, temp, free_disk, free_mem, proc_counter)
    sendData_lite(GPU_F, ECPU_F, PCPU_F, free_disk, free_mem, proc_counter)
    #sendData(GPU_F, ECPU_F, PCPU_F, free_disk, free_mem, proc_counter)
    ReadData()
    time.sleep(updateTime)
    
#lingzhou@192 host_python % python3 list_serial_ports.py                        
#['/dev/tty.wlan-debug', '/dev/tty.EDIFIERNeoBudsPro', '/dev/tty.Bluetooth-Incoming-Port', '/dev/tty.usbmodemUiFlow2_1']
