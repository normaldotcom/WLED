#!/usr/bin/env python3
from zeroconf import ServiceBrowser, Zeroconf, ServiceListener
import socket
import os
from rich import print as print

devices = []

class WLEDListener(ServiceListener):
    def remove_service(self, zeroconf, type, name):
        print(f"Service {name} removed")

    def update_service(self, zeroconf, type, name):
        pass

    def add_service(self, zeroconf, type, name):
        #print("name is", name)
        if name.startswith("wled"):
            info = zeroconf.get_service_info(type, name)
            if info:
                addresses = info.addresses
                ip_addresses = [socket.inet_ntoa(addr) for addr in addresses]
                print(f"[green]   WLED Device Found: {name} - IP: {ip_addresses[0]}")
                devices.append((name, ip_addresses[0]))


zeroconf = Zeroconf()
listener = WLEDListener()
browser = ServiceBrowser(zeroconf, "_wled._tcp.local.", listener)

fw_version = os.popen('git describe --tags').read().strip()

try:
    result = os.popen(f'curl -s -F "update=@./.pio/build/lumapxl/firmware.bin" http:/192.168.11.152/update').read()
    if 'Update successful!' in result:
        print('[green]   Update OK')
    else:
        print('[red]   Update FAILED')

finally:
    zeroconf.close()
