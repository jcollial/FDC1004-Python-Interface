import pathlib
import sys
import time

import pandas as pd
import serial

# --------------------------------------------------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------------------------------------------------
DATA_ACQUISITION_DURATION = 60  # Duration of data acquisition in seconds

# ESP32 variables:
CAPDAC = 0

# Serial port variables:
port = "COM3"
baudRate_serial = 115200  # Measure of data speed
timeout_serial = 2  # in seconds  # Number of seconds to wait for serial data

DataFileName = "myFile"

# --------------------------------------------------------------------------------------------------------------------
# Private Constants
# --------------------------------------------------------------------------------------------------------------------
_CAP_SENSOR_SAMPLING_RATE = 200  # in Hz


# --------------------------------------------------------------------------------------------------------------------
# Functions
# --------------------------------------------------------------------------------------------------------------------
def enhancedReadSerial(serialPort: serial, nBytes, timeout=40):
    """
    Reads a specified number of bytes from a serial port with a timeout mechanism.

    Args:
        serialPort: The serial port object.
        nBytes: Number of bytes to read.
        timeout: Number of attempts before timing out.

    Returns:
        A bytearray containing the received data.
    """
    buf = bytearray()
    cont = 0

    while True:
        # Read available bytes (at least 1, at most nBytes)
        ii = max(1, min(nBytes, serialPort.in_waiting))
        data = serialPort.read(ii)
        if not data:
            cont += 1
            if cont == timeout:
                print("Error: timeout in enhanced read serial")
                sys.exit(1)
        else:
            buf.extend(data)

        if len(buf) >= nBytes:
            return buf


def getDevAck(serialPort: serial, comm2send, comm2rec, timeout=40):
    """
    Sends a command to the ESP32 and waits for acknowledgment.

    Args:
        serialPort: The serial port object.
        comm2send: The command to send (string or convertible to string).
        comm2rec: The expected acknowledgment byte.
        timeout: Number of attempts before timing out.
    """
    if not isinstance(comm2send, str):
        comm2send = str(comm2send)

    esp32Timeout = 0  # Counter for timeout attempts

    # Send the expected acknowledgment code to ESP32
    serialPort.write(comm2rec.to_bytes(1, "big"))

    # Wait for the correct acknowledgment byte
    while esp32Timeout <= 20:
        if serialPort.in_waiting > 0:
            x = serialPort.read()
            if int.from_bytes(x, "big") == comm2rec:
                esp32Timeout = 0
                break
            else:
                print("\nError (ESP32 Communication): Failed to receive correct command from the device \n")
                sys.exit(1)  # stop program execution if error found

        esp32Timeout += 1
        if esp32Timeout == timeout:
            print("\nError (ESP32 Communication): Timeout when communicating with the device \n")
            sys.exit(1)  # stop program execution if error found
        time.sleep(0.5)

    # Send the actual command to ESP32
    serialPort.write(comm2send.encode("utf-8"))

    # Wait for 'O' (OK) response from ESP32
    while esp32Timeout <= 20:
        if serialPort.in_waiting > 0:
            x = serialPort.read().decode("utf-8")
            if x == "O":
                esp32Timeout = 0
                break
            else:
                print("\nError (ESP32 Communication): Failed to receive 'O' command from the device \n")
                sys.exit(1)  # stop program execution if error found

        esp32Timeout += 1
        if esp32Timeout == timeout:
            print("\nError (ESP32 Communication): Timeout when communicating with the device \n")
            sys.exit(1)  # stop program execution if error found
        time.sleep(0.5)


def build_data_headers(headers: dict, custom_metadata: dict = None) -> dict:
    """
    Builds a forceHeaders dictionary by combining metadata and headers.

    Parameters:
    - headers (dict): A dictionary where keys are column letters and values are header names.
    - custom_metadata (dict): Optional. A dictionary with specific metadata lists for certain columns.

    Returns:
    - dict: A dictionary where each key maps to a list of metadata + header.
    """
    if custom_metadata is None:
        custom_metadata = {}

    header_spacing = 1  # spacing between metadata and headers

    # Determine the maximum metadata length
    max_meta_len = max([len(v) for v in custom_metadata.values()], default=0) + header_spacing

    # Pad default metadata
    padded_default_metadata = [None] * (max_meta_len)

    # Pad all custom metadata entries
    padded_custom_metadata = {col: meta + [None] * (max_meta_len - len(meta)) for col, meta in custom_metadata.items()}

    # Build the final dictionary
    return {col: padded_custom_metadata.get(col, padded_default_metadata) + [header] for col, header in headers.items()}


if __name__ == "__main__":
    # Calculate number of samples to read from ESP32
    sampsToGet = int(_CAP_SENSOR_SAMPLING_RATE * DATA_ACQUISITION_DURATION)

    # Each sample from ESP32 is 8 bytes (4 bytes timestamp + 2 bytes cap sensor + 2 bytes padding or unused)
    nBytes_to_receive = 8

    # Open serial communication
    try:  # Try lets a block of code be tested for errors
        # When a port is given, the serial communication starts automatically
        # otherwhise we have to call serial.Open()
        serialPort = serial.Serial(port, baudrate=baudRate_serial, timeout=timeout_serial)
    except serial.SerialException:
        print("\nError (Serial Communication): Check the communication port \n")
        sys.exit(1)  # stop program execution if error found

    serialPort.reset_input_buffer()

    # Set CAPDAC value
    CAPDAC = max(0, min(31, CAPDAC))

    getDevAck(serialPort, CAPDAC, 0)

    # Send number of samples to get to the ESP32
    getDevAck(serialPort, sampsToGet, 1)

    # Send start signal to ESP32
    getDevAck(serialPort, "S", 2)

    # Read the incoming serial data
    serialData = enhancedReadSerial(serialPort, sampsToGet * nBytes_to_receive)

    # Check if data received matches expected size
    (
        print(f"Total data received is: {len(serialData)//nBytes_to_receive}")
        if len(serialData) % nBytes_to_receive == 0
        else print(f"Possible data loss. Total data received is: {len(serialData)/nBytes_to_receive}")
    )

    # Split data into timestamp and cap sensor parts
    pairs = [(elements[:4], elements[4:]) for elements in [serialData[ii : ii + nBytes_to_receive] for ii in range(0, len(serialData), nBytes_to_receive)]]

    esp32_timestamp_bytes = []
    cap_sensor_bytes = []

    for x, y in pairs:
        esp32_timestamp_bytes.append(x)
        cap_sensor_bytes.append(y)

    # Convert Cap sensor bytes to capacitance values (from page 16 on FDC1004 datasheet). Use little as byteorder as the ESP32 sends byte in little endian
    capData = [round(((int.from_bytes(bytes_data, byteorder="little", signed=True) / 524288.0) + (CAPDAC * 3.125)), 4) for bytes_data in cap_sensor_bytes]

    # Convert timestamp bytes to integers
    _esp32_timestamp = [int.from_bytes(bytes_data, byteorder="little") for bytes_data in esp32_timestamp_bytes]
    esp32_timestamp = [x - _esp32_timestamp[0] for x in _esp32_timestamp]

    serialPort.close()

    # ------------------------------------------------------------------------------------------------------------------
    print(f"\nSaving data, please wait...")

    # Check if Force Data folder exists. If it does not, then create the folder to store data
    dataFolderNamePath = pathlib.Path(__file__).parent.joinpath("Force Data")
    if not dataFolderNamePath.is_dir():
        dataFolderNamePath.mkdir()

    cap_data_num_samples = list(range(1, len(esp32_timestamp) + 1))

    _dataHeaders = {
        "A": "Sample No.",
        "B": "Timestamp (us)",
        "C": "Capacitance (pF)",
    }

    # Metadata
    dataMetadata = {
        "A": ["Data Collection Duration (s):", "Capacitive Sensor Sample rate (Hz):"],
        "B": [DATA_ACQUISITION_DURATION, _CAP_SENSOR_SAMPLING_RATE],
    }

    # Data content
    dataBody = {
        "A": cap_data_num_samples,
        "B": esp32_timestamp,
        "C": capData,
        # Add more columns here as needed
    }

    dataHeaders = build_data_headers(_dataHeaders, dataMetadata)

    # Create a DataFrame for the headers
    header_df = pd.DataFrame(dataHeaders)

    # Convert dataBody into a dataframe
    dataBody_df = pd.DataFrame(dataBody)

    # Concatenate the header and data DataFrames
    capacitanceData_df = pd.concat([header_df, dataBody_df], ignore_index=True)

    capacitanceData_df.to_csv(
        dataFolderNamePath.joinpath(DataFileName + ".csv"),
        index=False,
        header=False,
    )

    print(f"\nDone saving data")
