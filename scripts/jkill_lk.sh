#! /bin/bash

echo 'Shutdown lk cell'
jailhouse cell shutdown lk

echo 'Destroy lk cell'
jailhouse cell destroy lk

echo 'Disabling Jailhouse root cell'
jailhouse disable

echo 'Rmmod jailhouse.ko'
rmmod jailhouse.ko
