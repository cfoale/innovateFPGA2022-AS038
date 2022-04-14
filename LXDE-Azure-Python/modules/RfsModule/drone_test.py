# Copyright (C) 2021 Intel Corporation 
# Licensed under the MIT license. See LICENSE file in the project root for
# full license information.

import sys
sys.path.append('../')

import unittest
from package.drone import *
from package.basecomnios import *

class TestDrone(unittest.TestCase):
    def setUp(self):
        bridges = get_nios_status(hostname)
        self.drone_dummy = Drone(real=False,offset=0x0)
        self.drone = None
        self.bridge = None
        if (bridges[0] is not None) :
            self.bridge = bridges[1]
            self.drone = Drone(real=True,offset=0x0)

    def test_0_get_telemetries(self):
        res = self.drone_dummy.get_telemetries()
        self.assertFalse(res)
        if (self.drone is not None) :
            values = self.drone.get_telemetries(self.bridge)
            #Cannot be validated
            print(values)
            self.assertIsNot(values,False)

if __name__ == "__main__":
    unittest.main()