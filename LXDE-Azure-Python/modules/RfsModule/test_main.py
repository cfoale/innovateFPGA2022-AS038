#test_main.py
import time
import sys
import asyncio
from six.moves import input

from azure.iot.device.aio import IoTHubModuleClient, IoTHubDeviceClient, ProvisioningDeviceClient

from package.utility import *
from package.basecomnios import *
from package.gsensor import *
from package.rfssensor import *
from package.thresholdcontroller import *
from package.drone import *

#Test without Azure IoT Edge
isEdge = False
#Test without FPGA design
isReal = True

#Model is in public now and it can be searched.
#https://github.com/Azure/iot-plugandplay-models/tree/main/dtmi/terasic/fcc
model_id ="dtmi:Terasic:FCC:DE10_Nano;1" 
useComponent = True

#Global Instances at first to support simulations
control_bridge = None
data_bridge    = None
ds = Drone(name="Drone", real=True)

bridges = get_nios_status(hostname)

logger.debug('DEBUG ::: The FPGA is ready!')
isReal = True      
control_bridge = bridges[0]
data_bridge = bridges[1]
    #Send initial values to Azure 
init_patch= {
         'thresholdProperty': {
             '__t': 'c',
                'LOC': { 'min': 0, 'max': 255}
            },
            "$version": 1
        }
thc = ThresholdController(real=False,bridge=data_bridge,offset=0x0) #the min, max values are not stored in the fpga, so False
init_dict=thc.update_component_property(data_bridge,init_patch)
#print(init_dict)
        