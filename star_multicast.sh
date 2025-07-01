#! /bin/bash

echo "ubuntu"|sudo -S ifconfig enp88s0 multicast
echo "ubuntu"|sudo -S route add -net 224.0.0.0 netmask 240.0.0.0 dev enp88s0