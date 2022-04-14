#!/bin/bash
export IOTHUB_DEVICE_SECURITY_TYPE='DPS'
export IOTHUB_DEVICE_DPS_ID_SCOPE='0ne0046176E'
export IOTHUB_DEVICE_DPS_DEVICE_ID='DE10-Nano-1'
export IOTHUB_DEVICE_DPS_DEVICE_KEY='vsh4nKKwEKS4ESVG8W4Hw/8USGdnXKLdLvFbvqprOXFonjsVVOd6RIMKmoFxECFDioKO/vULM/JwV9uFKsj+iQ=='

overlay_dir="/sys/kernel/config/device-tree/overlays/socfpga"
overlay_dtbo="rfs-overlay.dtbo"
overlay_rbf="Module5_Sample_HW.rbf"

if [ -d $overlay_dir ];then
    rmdir $overlay_dir
fi

cp $overlay_dtbo /lib/firmware/
cp $overlay_rbf /lib/firmware/

mkdir $overlay_dir

echo $overlay_dtbo > $overlay_dir/path

cd ../
python3.7 -u ./main0.py



