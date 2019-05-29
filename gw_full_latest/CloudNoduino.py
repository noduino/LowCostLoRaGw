#!/usr/bin/python

#-------------------------------------------------------------------------------
# Copyright 2019, Maike Labs.

# Author: Jack Tan <jack.tan@jackslab.org>
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the program.  If not, see <http://www.gnu.org/licenses/>.
#"-------------------------------------------------------------------------------

# python CloudNoduino.py "Vbat/3.22/Press/-46.87" "1,16,8,56,24,7,-50" "125,5,12,433299" "2019-05-29T13:13:15.015+08:00" "MKLRB827EB8E763900"

import time
import datetime
import httplib
import random
import urllib
import urllib2
import ssl
import socket
import sys
import re

# get key definition from external file to ease
# update of cloud script in the future
import key_NoduinoAPI

try:
	key_NoduinoAPI.source_list
except AttributeError:
	key_NoduinoAPI.source_list=[]

base_url = '/dev/quark?'

# didn't get a response from grovestreams server?
connection_failure=False
    
# function to check connection availability with the server
def test_network_available():
	connection = False
	iteration = 0
	response = None
	
	# we try 4 times to connect to the server.
	while(not connection and iteration < 4) :
		try:
			# 3sec timeout in case of server available but overcrowded
			response=urllib2.urlopen('http://api.noduino.org/', timeout=3)
			connection = True
		except urllib2.URLError, e: pass
		except socket.timeout: pass
		except ssl.SSLError: pass

	    # if connection_failure == True and the connection with the server is unavailable, don't waste more time, exit directly
		if(connection_failure and response is None):
			print('NoduinoAPI: the server is still unavailable')
			iteration = 4
			# print connection failure
		elif(response is None) :
			print('NoduinoAPI: server unavailable, retrying to connect soon...')
			# wait before retrying
			time.sleep(1)
			iteration += 1
	return connection

# send a data to the server
def send_data(data, devid, key):
	#10seconds timeout in case we lose connection right here, then do not waste time
	conn = httplib.HTTPConnection('api.noduino.org', timeout=10)
	
	#Upload the feed
	try:

		url = base_url + "devid=" + devid
		
		#add in the url the channels and the data
		i = 0
		while i < len(data) :
			url += "&"+ key[i] + "=" + data[i]
			i += 1
            
		headers = {"Connection" : "close", "Content-type": "application/json", "APIKey" : key_NoduinoAPI.api_key}

		print('NoduinoAPI: Uploading data to: ' + url)

		conn.request("POST", url, "", headers)

		#Check for errors
		response = conn.getresponse()
		status = response.status

		if status != 200 and status != 201:
			try:
				if (response.reason != None):
					print('NoduinoAPI: HTTP Failure Reason: ' + response.reason + ' body: ' + response.read())
				else:
					print('NoduinoAPI: HTTP Failure Body: ' + response.read())
			
			except Exception:
				print('NoduinoAPI: HTTP Failure Status: %d' % (status) )

	except Exception as e:
		print('NoduinoAPI: HTTP Failure: ' + str(e))

	finally:
		if conn != None:
			conn.close()	
	
	
def push_data(key, data, devid):
	
	connected = test_network_available()
	
	# if we got a response from the server, send the data to it
	if(connected):
		#len(key) == len(data)
		print("NoduinoAPI: uploading")
		send_data(data, devid, key)
	else:
		print("NoduinoAPI: not uploading")
		
	#update grovestreams_connection_failure value
	global connection_failure
	connection_failure = not connected


def main(ldata, pdata, rdata, tdata, gwid):

	# this is common code to process packet information provided by the main gateway script (i.e. post_processing_gw.py)
	# these information are provided in case you need them
	arr = map(int,pdata.split(','))

	dst = arr[0]

	ptype = arr[1]				

	src = arr[2]

	seq = arr[3]
	datalen = arr[4]

	snr = arr[5]
	rssi = arr[6]

	# tdata: 2019-05-29T13:13:15.015+08:00
	ar = time.strptime(tdata[:-6]+"000", "%Y-%m-%dT%H:%M:%S.%f")
	ts = int(time.mktime(ar))

	#LoRaWAN packet
	if dst == 256:
		src_str = "%0.8X" % src
	else:
		src_str = "%02X" % src	# 254 is FE

	if (src_str in key_NoduinoAPI.source_list) or (len(key_NoduinoAPI.source_list)==0):

		ldata = ldata.replace(' ', '')		# strstrip(ldata)
				
		# this part depends on the syntax used by the end-device
		# we use: TC/22.4/HU/85...
		#
		# but we accept also a_str#b_str#TC/22.4/HU/85... for compatibility with ThingSpeak
		# or simply 22.4 in which case, the nomemclature will be DEF

		# get number of '#' separator
		nsharp = ldata.count('#')		 		
		nslash = 0

		#will field delimited by '#' in the MongoDB
		data = ['','']
				
		#no separator
		if nsharp == 0:
			# get number of '/' separator on ldata
			nslash = ldata.count('/')

			#contains ['', '', "s1", s1value, "s2", s2value, ...]
			data_array = data + re.split("/", ldata)		
		else:				
			data_array = re.split("#", ldata)
		
			# only 1 separator
			if nsharp == 1:
				# get number of '/' separator on data_array[1]
				nslash = data_array[1].count('/')
		
				# then reconstruct data_array
				data_array = data + re.split("/", data_array[1])	

			# we have 2 separators
			if nsharp == 2:
				# get number of '/' separator on data_array[2]
				nslash = data_array[2].count('/')
		
				# then reconstruct data_array
				data_array = data + re.split("/", data_array[2])
		
		#just in case we have an ending CR or 0
		data_array[len(data_array)-1] = data_array[len(data_array)-1].replace('\n', '')
		data_array[len(data_array)-1] = data_array[len(data_array)-1].replace('\0', '')	
	
		#test if there are characters at the end of each value, then delete these characters
		i = 3
		while i < len(data_array) :
			while not data_array[i][len(data_array[i])-1].isdigit() :
				data_array[i] = data_array[i][:-1]
			i += 2
																		
		key = []
		# data to send	
		value = []
	
		if nslash == 0:
			# old syntax without nomenclature key, so insert only one key
			# we use DEF
			key.append("DEF")
			value.append(data_array[2])
		else:
			#completing key and data
			i = 2
			while i < len(data_array)-1 :
				key.append(data_array[i])
				value.append(data_array[i+1])
				i += 2

		key.append("snr")
		value.append(str(snr))

		key.append("rssi")
		value.append(str(rssi))

		key.append("ts")
		value.append(str(ts))
	
		#upload data to grovestreams
		push_data(key, value, gwid[0:-2] + src_str)		
	else:
		print "Source is not is source list, not sending with CloudNoduino.py"			

if __name__ == "__main__":
	main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
