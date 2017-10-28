# -*- coding: utf-8 -*-
"""
Created on Sat Sep 23 12:54:47 2017

@author: mtkes
"""

import socket
import struct # for packing stuctures for socket options

# IP Addresses are used by the network hardware to route traffic
# It is just like having a house address or zip code
# In networks the interface hardware is assigned an IP address to
# make the interface unique within the network.
# In the IPv4 standard IP addresses contain 4 bytes representing
# layers to the local network. Usually, the address is represented
# as a string containing '.' as a delimeter for readability
#
# Examples:
#   Some addresses are special, defined in various standards to mean
#   specific things to the network (https://en.wikipedia.org/wiki/Reserved_IP_addresses)
#
#   The standard loopback address (talk to self) is '127.0.0.1'
#
#   A typical Ethernet router (gateway) at '192.168.0.1' will assign your 
#   PC something like '192.168.0.2', '192.168.0.3', etc. although sometimes the
#   '10.x,x,x' range is used.
##
#   '255.255.255.255' is reserved to broadcast to all nodes on the network
#
#   '224.0.0.0' to '239.255.255.255' is reserved for multicast
#
IP_ANY = ''                     # Sometimes we don't care about the actual IP Address
IP_LOOPBACK = '127.0.0.1'       # Sometimes we need to test applictions with ourselves
IP_LOCAL = IP_LOOPBACK          # The local address is typicaly the IP address associated
                                # with the physical network interface on the machine
                                # If the client does not know this address it will
                                # be unable to request connection or otherwise
                                # communicate... this is what makes UDP/IP
                                # with multicast 'simpler' in that the destination
                                # to "stream" information does not require
                                # knowledge of the interface assignments, but
                                # rather a 'multicast group' address that many
                                # can 'listen' to
IP_MULTICAST_GROUP_A = '230.0.0.1'

# Ports are a way of distinguishing the type of information traffic on a
# network. The port is simply a logical distinction that allows traffic to
# be routed to one or more sockets that care about the information.
#
# Like IP Addressses, some ports are reserved (https://en.wikipedia.org/wiki/List_of_TCP_and_UDP_port_numbers)
# and should not be arbitrarily used for other purposes as this could create
# confusion in the applications that are expecting the traffic.
#
# 0 to 1023 are "well known" (e.g., 80 is HTTP, 22 is FTP, don't use them for other purposes or you'll just confuse people)
# 1024 to 49151 are "registered" (i.e., if you use them you may have a conflict with other applications)
# 49152 to 65535 are "dynamic" and may be assigned by user applications
#
# We just need to pick a port to start listening for connections
# then we will let the system assign connections to a new port to
# keep traffic separated
PORT = 53421

while (True):
    # A UDP socket is an object we will use to send data to an address
    mySocket = socket.socket(family=socket.AF_INET,
                                 type=socket.SOCK_DGRAM, 
                                 proto=socket.IPPROTO_UDP)
    
    if (mySocket == None):
        print("Unable to create mySocket socket")
        break
    
    # Allow the address/port pair to be reused by other processes
    mySocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Set the time-to-live for messages to 1 so they do not go past the
    # local network segment. If we need to let the message go further,
    # multicasting to many networks we can set the TTL value as high
    # as 255
    ttl = struct.pack('b', 1)
    mySocket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)
    
    # 'binding' a socket just means associating it with an
    # interface address and a port on that address with the socket
    # object. All network traffic to the address and port will signal
    # the socket each time a message comes in.
    # The interface address can be a specific IP address associated
    # with network interface hardware or we can request that the
    # first interface found is used (IP_ANY). In some environments
    # IP_ANY is '0.0.0.0', in this environment the empty string has
    # the same effect.
    #
    # Ports are a way of identifying packets to be routed to specific
    # user path on an interface
    print("Binding...")
    mySocket.bind((IP_ANY, 0))
    
    # Now we just start sending
    print("Sending...")

    i = 0
    while (True):
        try:
            i = i + 1
            s = 'String = ' + str(i + 1)
            print(s)
            # Now we send to known port of all multicast group members
            mySocket.sendto(str.encode(s), (IP_MULTICAST_GROUP_A, PORT))
        except Exception as e:
            # Just print the excepts and try to create a new socket (start over)
            print(e)
            break

        
    