#!/bin/bash

echo  ScriptBegins!


sudo systemctl stop dnsmasq
sudo systemctl stop hostapd

rm /etc/dhcpcd.conf
cp dhcp.conf.orig  /etc/dhcpcd.conf

sudo service dhcpcd restart

rm /etc/dnsmasq.conf
cp dnsmasq.conf.orig /etc/dnsmasq.conf

sudo systemctl start dnsmasq

rm /etc/hostapd/hostapd.conf
cp hostapd.conf.orig  /etc/hostapd/hostapd.conf

rm /etc/default/hostapd
cp hostapd.orig  /etc/default/hostapd

sudo systemctl mask hostapd
sudo systemctl disable hostapd
sudo systemctl stop   hostapd


rm /etc/sysctl.conf 
cp sysctl.conf.orig  /etc/sysctl.conf
 
rm /etc/rc.local
cp rc.local.orig  /etc/rc.local

echo Script finished


