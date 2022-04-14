#!/bin/bash
export IOTHUB_DEVICE_SECURITY_TYPE='DPS'
export IOTHUB_DEVICE_DPS_ID_SCOPE='0ne004B48BF'
export IOTHUB_DEVICE_DPS_DEVICE_ID='de10-nano-drone'
export IOTHUB_DEVICE_DPS_DEVICE_KEY='22xC2oeFU+LKX+bFjISjFTdaRUMWGtexo/dYm1Tb5i+G7oqOOSJsq8rYL8Jsc9zNDbmMPXcEkgo7u6AXHIzhew=='

overlay_dir="/sys/kernel/config/device-tree/overlays/socfpga"
overlay_dtbo="rfs-overlay.dtbo"
overlay_rbf="soc_system.rbf"

if [ -d $overlay_dir ];then
    rmdir $overlay_dir
fi

cp $overlay_dtbo /lib/firmware/
cp $overlay_rbf /lib/firmware/

mkdir $overlay_dir

echo $overlay_dtbo > $overlay_dir/path

cd ../

#thonny &
python3.7 -u ./main1.py
