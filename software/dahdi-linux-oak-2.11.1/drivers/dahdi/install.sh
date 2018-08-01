#!/bin/sh
rmmod pitdm
rmmod pidma
insmod pidma.ko
insmod pitdm.ko
dahdi_cfg
