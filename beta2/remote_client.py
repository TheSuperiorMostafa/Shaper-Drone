#!/usr/bin/env python3
"""
Simple client for remote control interface.
Usage: python3 remote_client.py <pi_ip_address> <command>
Commands: emergency_stop, status
"""

import sys
from remote_control import RemoteControl

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 remote_client.py <pi_ip> <command>")
        print("Commands: emergency_stop, status")
        sys.exit(1)
    
    host = sys.argv[1]
    command = sys.argv[2]
    port = 8888
    
    remote = RemoteControl()
    response = remote.send_command(host, port, command)
    print(f"Response: {response}")

