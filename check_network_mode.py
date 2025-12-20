#!/usr/bin/env python3
"""Check if container is using host networking"""

import socket
import os

def check_network_mode():
    """Check various indicators of network mode"""
    print("Checking network configuration...")
    print("=" * 60)
    
    # Check hostname
    try:
        hostname = os.uname().nodename
        print(f"Hostname: {hostname}")
        if hostname == "docker-desktop" or "container" in hostname.lower():
            print("  ⚠ Hostname suggests container networking (not host mode)")
        else:
            print("  ✓ Hostname suggests host networking")
    except:
        pass
    
    # Check if we can bind to 0.0.0.0 (all interfaces)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", 0))
        addr = s.getsockname()
        s.close()
        print(f"Can bind to 0.0.0.0: {addr}")
        print("  ✓ Can bind to all interfaces")
    except Exception as e:
        print(f"  ✗ Cannot bind to all interfaces: {e}")
    
    # Check network interfaces
    try:
        interfaces = socket.if_nameindex()
        print(f"\nNetwork interfaces ({len(interfaces)} found):")
        wifi_interfaces = [name for idx, name in interfaces if 'wlan' in name or 'wifi' in name or 'wl' in name]
        if wifi_interfaces:
            print(f"  WiFi interfaces found: {wifi_interfaces}")
            print("  ✓ WiFi interfaces visible (suggests host networking)")
        else:
            print("  ⚠ No WiFi interfaces visible")
            print("  This might indicate container networking")
    except Exception as e:
        print(f"  Error checking interfaces: {e}")
    
    # Try to get local IP addresses
    try:
        import subprocess
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            ips = result.stdout.strip().split()
            print(f"\nLocal IP addresses: {ips}")
            tello_network = [ip for ip in ips if ip.startswith('192.168.10.')]
            if tello_network:
                print(f"  ✓ Found IP on Tello network: {tello_network}")
            else:
                print("  ⚠ No IP on 192.168.10.x network")
                print("  Make sure you're connected to Tello WiFi")
        else:
            print("  Cannot determine IP addresses")
    except:
        print("  Cannot check IP addresses (hostname command not available)")
    
    print("\n" + "=" * 60)
    print("Recommendations:")
    print("1. If hostname is 'docker-desktop', rebuild devcontainer")
    print("2. Ensure you're connected to Tello WiFi network")
    print("3. Verify devcontainer.json has: \"runArgs\": [\"--network=host\"]")

if __name__ == "__main__":
    check_network_mode()

