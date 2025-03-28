#!/usr/bin/env python3
from zeroconf import ServiceBrowser, Zeroconf, ServiceListener
import socket
import os
from rich import print as print

import argparse

parser = argparse.ArgumentParser(description="Flashes single wled board")
parser.add_argument("ip", help="IP address of target")
args = parser.parse_args()

fw_version = os.popen('git describe --tags').read().strip()
print(f"Flashing {args.ip}")
result = os.popen(f'curl -s -F "update=@./.pio/build/lumapxl/firmware.bin" http://{args.ip}/update').read()
if 'Update successful!' in result:
    print('[green]   Update OK')
else:
    print('[red]   Update FAILED')

