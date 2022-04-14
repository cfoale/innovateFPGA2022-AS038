# Copyright (C) 2021 Intel Corporation 
# Licensed under the MIT license. See LICENSE file in the project root for
# full license information.
testing = 0 #cmf - set to 1 fwhen testing in the package folder

import asyncio
import struct
if not testing:
    from package.utility import *
else:
    from utility import *
import logging
import os
import mmap
import socket

'''
    hostname is global variable to check what's device for this application
'''
hostname = os.getenv('IOTEDGE_GATEWAYHOSTNAME',socket.gethostname()) 

'''
    BaseComNIOS class is base communication class with the NIOS II processor in FPGA.
    In this design, the way to access data(sensor/threshold) is same(Of course, offset is different) and that's why this class is created.
    It will be extended to Sensor or Threshold class.
'''

class BaseComNIOS :

    def __init__(self, type:str='float', real=False) -> None:
        self.real:bool = real #If it is a real device, please set it True
        self.type = type #float or int

    '''
        Method read(self, bridge, offset)
        Arguments:
            bridge : data bridge can be accessed
            offset : Address offset to read value
        Return Value:
            False : Fails due to some reasons
            or 
            <Value>(float/int): Read Value
    '''
    def read(self, bridge, offset) :
        print('basecomnios.read')
        try : 
            if self.real == False :
                return False
            if offset == None : 
                raise Exception('Error: Offset value is None!')
            bridge.seek(offset)
            if self.type == 'int' :
                _data = bridge.read_byte()
                return _data
            elif self.type == 'float' :
                _data = bridge.read_byte()
                return _data
            else:
                _data = bridge.read_byte()
                return _data
        except Exception as e: 
            logger.debug(e)
            return False


    '''
        Method write(self, bridge, offset, data=None)
        Arguments:
            bridge : data bridge can be accessed.
            offset : Address offset to write value
            data   : Requested value to write
        Return Value:
            False : Fails due to some reasons
            or 
            True  : Complete
    '''
    def write(self, bridge, offset, data=None) -> bool:
        print('basecomnios.write called')
        try : 
            if self.real == False :
                return False
            if offset == None : 
                raise Exception('Error: Offset value is None!')
            bridge.seek(offset)
            if self.type == 'int' :
                _bytedata = data.to_bytes(4, byteorder='little')
                bridge.write(_bytedata)
                return True
            elif self.type == 'float' :
                _bytedata = struct.pack('<f', data)
                bridge.write(_bytedata)
                return True
            else :
                raise Exception('Error: {} is illegal and type should be int or float.'.format(type(data)))
        except Exception as e :
            logger.debug(e)
            return False


'''
    Below functions will be used in main.py or test codes.
    It is because they are initialization/tracking methods to check HW design working correctly.
'''

# Base Address
LW_BRIDGE_BASE_ADDR = 0xff200000 #points to sysid_qsys shared_memory.s1 from soc_system.html
LW_BRIDGE_SIZE = 0x100000
REG_BYTE_SIZE = 4 # the number of byte to read/write (default=REG_BYTE_SIZE=4=size(int)=size(float))
#cmf adds
FPGA_DSP_BYTE_ADDR = 0x00000010


'''
    Method get_nios_status(hostname)
    Arguments:
        hostname : hostname string.
    Return Value:
        mmap.mmap, mmap.mmap : control bridge for nios watchdog, and data bridge for data communications
        or 
        None, None : if it fails
'''
def get_nios_status(hostname:str) :
    print('basecomnios.get_nios_status')
    control_bridge = None
    data_bridge    = None
    if (hostname in 'de10nano'):
        try :
            f = os.open('/dev/mem', os.O_RDWR|os.O_SYNC)
            control_bridge = mmap.mmap(f, LW_BRIDGE_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE, offset=LW_BRIDGE_BASE_ADDR)
            data_bridge = mmap.mmap(f, LW_BRIDGE_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE, offset=LW_BRIDGE_BASE_ADDR)
    
        except Exception as e:
            logger.debug(getattr(e, 'message', repr(e)))
            control_bridge = None
            data_bridge = None
    else :
            logger.debug('Warning::: The hostname({}) is not supported.'.format(hostname))
    
    return control_bridge, data_bridge


'''
    Method watchdog_nios(bridge, interval)
    Arguments:
        bridge : control bridge to access NIOS
        interval : interval time to the next loop
    Note:
        It is watchdog function to monitor whether NIOS in FPGA works
'''
global fake_watch_dog
fake_watch_dog = 0x7fffffff  #cmf= 0x7fffffff  #cmf

async def watchdog_nios(bridge, interval) :
    print('basecomnios.watchdog_nios')
    loop = 0
    while True:
        temp = read_loop_value(bridge)
        if (loop == temp) or (temp is None): 
            logging.debug('Error::: NIOS maybe stuck!!!Loop Count{}.'.format(loop))
            break
        else :
            loop = temp
        await asyncio.sleep(interval)

'''
    Method read_loop_value(bridge)
    Arguments:
        bridge : control bridge to access NIOS
    Return Value:
        integer : the number of loop stored in NIOS
        or
        None : it fails
    Note:
        read_loop_value is helper for watchdog_nios method
'''

def read_loop_value(bridge) : 
    try :
        bridge.seek(0x0)
        _data = bridge.read(REG_BYTE_SIZE)
        _data = int.from_bytes(_data, byteorder='little')
        if (_data >> 31) : 
            logging.debug('Error::: NIOS maybe not working.')
        return (_data & 0x7fffffff)
    except Exception as e:
        logger.debug(getattr(e, 'message', repr(e)))
        return None

if testing:
    control_bridge, data_bridge = get_nios_status(hostname)
    base=BaseComNIOS(real=True)
    print(base.read(data_bridge,FPGA_DSP_BYTE_ADDR))
    #watchdog_task = asyncio.create_task(watchdog_nios(control_bridge, 1)) #
    #control_bridge.close()
    #data_bridge.close()


