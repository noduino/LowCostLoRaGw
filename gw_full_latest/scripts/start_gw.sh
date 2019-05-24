#!/bin/sh

cd /home/pi/lora_gateway

#run script for the shutdown button
#python /home/pi/lora_gateway/scripts/piShutdown.py &

#create the gw id so that a newly installed gateway is always configured with a correct id
#/home/pi/lora_gateway/scripts/create_gwid.sh

###
### Start Internet access with 3G dongle
############################################

if [ -f 3GDongle/use_3GDongle_internet_on_boot.txt ]
then
	echo "Start Internet with 3G Dongle"
	cd 3GDongle
	# the script will turn on the modem and launch pppd
	# it will keep re-launching wvdial is case wvdial crashes
	sudo ./wvdial_wrapper.sh Dongle &
	sleep 10
	cd ..
###
### Start Internet access with Loranga board
############################################
elif [ -f 3GDongle/loranga/use_loranga_internet_on_boot.txt ]
then
	echo "Start Internet with Loranga board"
	cd 3GDongle/loranga
	# the script will turn on the modem and launch pppd
	./start-internet.sh &
	sleep 10
	cd ../../
elif [ -f 3GDongle/loranga/use_loranga_SMS_on_boot.txt ]
then
	echo "Use Loranga board for SMS"
	cd 3GDongle/loranga	
	# just turn on the modem
	if [ -f loranga3G.txt ]
	then
		sudo python wake-3G.py
	else	
		sudo python wake-2G.py
	fi
	cd ../..
fi
############################################

###
### Start Node-Red if needed
############################################

#try to find whether NodeRed cloud is enabled or not
i=`jq '.clouds[].script|index("CloudNodeRed")|values' clouds.json`
#echo $i
j=`jq '.clouds[].script|index("CloudNodeRed")' clouds.json | grep -n "$i" | cut -d ":" -f1`
#echo $j
nodered=`jq ".clouds[$j-1].enabled" clouds.json`
#echo $nodered
if [ $nodered = "true" ]
then
	echo "CloudNodeRed is enabled, start Node-Red"
	sudo -u pi node-red-start &
fi
############################################

###
### Start GPS web interface nodejs if needed
############################################

#try to find whether CloudGpsFile cloud is enabled or not
i=`jq '.clouds[].script|index("CloudGpsFile")|values' clouds.json`
#echo $i
j=`jq '.clouds[].script|index("CloudGpsFile")' clouds.json | grep -n "$i" | cut -d ":" -f1`
#echo $j
gpsfile=`jq ".clouds[$j-1].enabled" clouds.json`
#echo $gpsfile
if [ $gpsfile = "true" ] && [ -f gps/server.js ]
then
	echo "CloudGpsFile is enabled, start GPS nodejs web interface"
	cd gps
	node server.js &
	cd ..
fi
############################################

###
### Internet routing for WiFi access point
############################################

#check if the gateway is an access point
#if yes, then enable IP forwarding to give internet connectivity to connected devices, e.g. smartphone, tablets,...
if [ -f /etc/network/interfaces_not_ap ]
then
		sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
		sudo sed -i 's/^#net.ipv4.ip_forward=1/net.ipv4.ip_forward=1/g' /etc/sysctl.conf

		if [ -f 3GDongle/use_3GDongle_internet_on_boot.txt ]
		then
				sudo iptables -t nat -A POSTROUTING -o ppp0 -j MASQUERADE
				sudo iptables -A FORWARD -i ppp0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
				sudo iptables -A FORWARD -i wlan0 -o ppp0 -j ACCEPT
		else
				sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
				sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
				sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
		fi
fi
############################################

###
### Last item, run the gateway
############################################

#run the gateway
if [ $# -eq 0 ]
then
	python start_gw.py &
fi

############################################

#leave some time for mongo to start, otherwise there will be a problem
sleep 10

#repair if needed the mongodb database connection
#/home/pi/lora_gateway/scripts/mongo_repair.sh
