#!/bin/bash

echo  ScriptBegins!


sudo systemctl stop dnsmasq
sudo systemctl stop hostapd

rm /etc/dhcpcd.conf
cp dhcpcd.conf /etc/dhcpcd.conf

sudo service dhcpcd restart

rm /etc/dnsmasq.conf
cp dnsmasq.conf /etc/dnsmasq.conf

sudo systemctl start dnsmasq

rm /etc/hostapd/hostapd.conf
cp hostapd.conf /etc/hostapd/hostapd.conf

rm /etc/default/hostapd
cp hostapd /etc/default/hostapd

sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl start hostapd



rm /etc/sysctl.conf 
cp sysctl.conf  /etc/sysctl.conf

sudo iptables -t nat -A  POSTROUTING -o eth0 -j MASQUERADE

sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"

 
rm /etc/rc.local
cp rc.local /etc/rc.local

echo Script finished


