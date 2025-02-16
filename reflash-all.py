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
        print("name is", name)
        if name.startswith("wled"):
            info = zeroconf.get_service_info(type, name)
            if info:
                addresses = info.addresses
                ip_addresses = [socket.inet_ntoa(addr) for addr in addresses]
                print(f"WLED Device Found: {name} - IP: {ip_addresses[0]}")
                devices.append((name, ip_addresses[0]))


zeroconf = Zeroconf()
listener = WLEDListener()
browser = ServiceBrowser(zeroconf, "_wled._tcp.local.", listener)

try:
    input("Press Enter to update all...\n")
    for device in devices:
        print(f"[blue]Updating: {device[0]} - {device[1]}")
        #os.system(f'curl -s -F "update=@./.pio/build/esp32dev_poefusion_ethernet/firmware.bin" {device[1]}/update')
        result = os.popen(f'curl -s -F "update=@./.pio/build/esp32dev_poefusion_ethernet/firmware.bin" {device[1]}/update').read()
        if 'Update successful!' in result:
            print('[green]   Update OK')
        else:
            print('[red]   Update FAILED')

finally:
    zeroconf.close()
