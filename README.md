## Step 1: Setting up the CAN Interface (Linux)

The CANable device appears as a serial port (e.g., `/dev/ttyACM0`). To use it with `python-can`'s SocketCAN backend, you must first create a virtual network interface (e.g., `can0`).

### 1.1. Find Your Device Path

You need to find the correct path to your CANable device. There are two ways to do this.

#### Option A: The Stable Method (Recommended)

Linux creates persistent symbolic links based on the physical USB port the device is plugged into. This path will not change when you unplug and replug the device.

1.  Connect your CANable device.
2.  Run the following command:
    ```bash
    ls -l /dev/serial/by-path/
    ```
3.  Look for an entry that points to a `ttyACM` device. The output will look something like this:
    ```
    # Example Output
    lrwxrwxrwx 1 root root 13 Oct 15 17:51 pci-0000:02:00.0-usb-0:7:1.0 -> ../../ttyACM2
    ```
    The stable path you should use is `/dev/serial/by-path/pci-0000:02:00.0-usb-0:7:1.0`.

#### Option B: The Dynamic Method

You can find the device name directly, but be aware that the number (`0`, `1`, `2`, etc.) might change each time you reconnect the device.

1.  Connect your CANable device.
2.  Run `dmesg` and look for the last `ttyACM` entry:
    ```bash
    dmesg | grep ttyACM
    # Example Output: [ 3400.786502] cdc_acm 1-7:1.0: ttyACM2: USB ACM device
    ```
    In this case, the device is `/dev/ttyACM2`.

### 1.2. Create and Activate the Interface

1.  **Run `slcand`** to create the `can0` interface from your device. Use the path you found above.

    ```bash
    # Usage: sudo slcand -o -c -s<speed_code> <device_path> <interface_name>
    # Using the stable path from our example:
    sudo slcand -o -c -s6 "/dev/serial/by-path/pci-0000:02:00.0-usb-0:7:1.0" can0
    ```

    **Bitrate Codes (`-s` flag):** The bitrate must match your CAN network and the setting in the Python script.
    - `-s5`: 250 kbit/s
    - `-s6`: 500 kbit/s (Default for the script)
    - `-s8`: 1 Mbit/s

2.  **Activate the `can0` interface**:
    ```bash
    sudo ip link set up can0
    ```

3.  **(Optional) Verify the interface** is up and running:
    ```bash
    ip link show can0
    ```
    You should see `state UP` in the output.
