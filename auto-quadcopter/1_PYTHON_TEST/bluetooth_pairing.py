import bluetooth

def pair_controller():
    # Get the Bluetooth adapter
    adapter_list = bluetooth.discover_address(lookup_names=True)
    adapter = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    # Discover nearby Bluetooth devices
    nearby_devices = bluetooth.discover_devices()

    # Prompt the user to select a device to pair with
    print("Select a device to pair with:")
    for i, device in enumerate(nearby_devices):
        print(f"{i+1}. {device[1]}")
    choice = int(input("Enter the number of the device: ")) - 1

    # Pair with the selected device
    device_address = nearby_devices[choice][0]
    device_name = nearby_devices[choice][1]
    print(f"Pairing with {device_name} ({device_address})...")
    adapter.connect((device_address, 1))
    adapter.close()
    print(f"Paired successfully with {device_name} ({device_address})")

if __name__ == '__main__':
    pair_controller()