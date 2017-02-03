#!/usr/bin/env python

import sys, getopt

from pymodbus.client.sync import ModbusTcpClient as ModbusClient

HOST = "localhost"
PORT = 502
VALUE = 0

def print_help():
    print("-h, --host <addr>    : connect to Go Motion on address <addr>, default " + str(HOST))
    print("-p, --port <port>    : connect to Go Motion on port <addr>, default " + str(PORT))
    print("-v, --value <value>  : write <value> to the output")
    print("-?, --help           : print this help message")

try:
    opts, args = getopt.getopt(sys.argv[1:], "h:p:v:?", ["host=", "port=", "value=", "help"])
except getopt.GetoptError, err:
    print("modbus: " + str(err))
    sys.exit(1)

for o, a in opts:
    if o in ("-h", "--host"):
        HOST = a
    elif o in ("-p", "--port"):
        try: PORT = int(a)
        except: 
            print_help()
            sys.exit(1)
    elif o in ("-v", "--value"):
        try: VALUE = float(a)
        except: 
            print_help()
            sys.exit(1)
    elif o in ("-?", "--help"):
        print_help()
        sys.exit(0)

try:
    client = ModbusClient(HOST, PORT)
except:
    print("modbus: can't create client")
    sys.exit(1)


if not client.connect():
    print("modbus: can't connect to " + str(HOST) + " on port " + str(PORT))
    sys.exit(1)

if VALUE >= 0:
    valout = int(3276.7 * VALUE)
    if valout < 0:
        valout = 0
    elif valout > 32767: 
        valout = 32767
else:
    valout = 3276.7 + 65535.0
    if valout < 32768:
        valout = 32768
    elif valout > 65535: 
        valout = 65535

if not client.write_register(0x8000, valout):
    print("modbus: can't write " + str(valout))

sys.exit(0)
