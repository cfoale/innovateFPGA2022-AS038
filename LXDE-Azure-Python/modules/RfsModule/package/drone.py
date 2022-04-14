#drone.py
# Copyright (C) 2021 Intel Corporation 
# Licensed under the MIT license. See LICENSE file in the project root for
# full license information.

from package.sensorcontroller import *
from package.utility import *
from package.basecomnios import *

FPGA_DSP_BYTE_ADDR = 0x00000040 #this is the only drone address
print('FPGA_DSP_BYTE_ADDR ' + str(FPGA_DSP_BYTE_ADDR))

class Drone(SensorController):
    def __init__(self,name="Drone", real=True, offset=0x0) -> None:
        self.name = name
        #They are dummies
        self.modules = {
            'X'        : Sensor('X', 4, real,        0x0),
            'Y'        : Sensor('Y', 4, real,        0x0),
            'LOC'      : Sensor('LOC', 4, real,     0x0),
        }
        self.real = real

    '''
        Method get_telemetries(self, bridge)
        Arguments:
            bridge              : data bridge can be accessed
        Return Value:
            dict : Response Telemetry Message dict 
            or
            False : if module is dummy
    '''
    def get_telemetries(self,bridge=None):
        #print('drone.get_telemetries')
        data = {}
        if self.real :
            data[self.name] = self.get_fpga()  #self.adxl345.get_axes()
            #print('drone.get_telemetries ' + str(data))
            return data
        else : 
            #logger.debug('{} sensor is dummy and if you want values to test, please use generate_dummy_value in threshold class.'.format(self.name))
            data = {0,0, self.get_fpga()} #cmf thinks a list is expected
            print('dummy - actually real - drone.get_telemetries ' + str(data))
            return False

    def get_fpga(self):
        #print('Drone.get_fpga')
        bridges = get_nios_status(hostname)
        bridges[1].seek(FPGA_DSP_BYTE_ADDR)
        d0 = bridges[1].read_byte()
        if d0 > 123:
            dict = {'WARNING': d0, 'WARNING': d0, 'LOC IMMINENT!': d0}
        else:
            dict = {'SAFE': d0, 'CONDITIONS': d0, 'OK': d0}
        return dict

