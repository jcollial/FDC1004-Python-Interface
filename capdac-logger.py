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
_CAP_SENSOR_SAMPLING_RATE = 80  # in Hz (this must match the TIME_INTERVAL variable in the ESP32 code)


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
    # Calculate the total number of samples to acquire from the ESP32
    sampsToGet = int(_CAP_SENSOR_SAMPLING_RATE * DATA_ACQUISITION_DURATION)

    # Each sample from the ESP32 consists of 8 bytes:
    # 4 bytes for timestamp and 4 bytes for capacitive sensor data
    nBytes_to_receive = 8

    # Initialize serial communication with the ESP32
    try:
        # If a port is specified, the serial connection opens automatically
        serialPort = serial.Serial(port, baudrate=baudRate_serial, timeout=timeout_serial)
    except serial.SerialException:
        print("\nError (Serial Communication): Check the communication port \n")
        sys.exit(1)  # Exit the program if the serial connection fails

    # Clear any existing data in the input buffer
    serialPort.reset_input_buffer()

    # Ensure CAPDAC value is within the valid range [0, 31]
    CAPDAC = max(0, min(31, CAPDAC))

    # Send CAPDAC configuration to the ESP32
    getDevAck(serialPort, CAPDAC, 0)

    # Send the number of samples to acquire to the ESP32
    getDevAck(serialPort, sampsToGet, 1)

    # Send the start signal to begin data acquisition
    getDevAck(serialPort, "S", 2)

    # Read the expected number of bytes from the serial port
    serialData = enhancedReadSerial(serialPort, sampsToGet * nBytes_to_receive)

    # Verify that the received data size matches the expected number of samples
    (
        print(f"Total data received is: {len(serialData)//nBytes_to_receive}")
        if len(serialData) % nBytes_to_receive == 0
        else print(f"Possible data loss. Total data received is: {len(serialData)/nBytes_to_receive}")
    )

    # Split the raw data into timestamp and capacitive sensor byte pairs
    pairs = [(elements[:4], elements[4:]) for elements in [serialData[ii : ii + nBytes_to_receive] for ii in range(0, len(serialData), nBytes_to_receive)]]

    esp32_timestamp_bytes = []
    cap_sensor_bytes = []

    for x, y in pairs:
        esp32_timestamp_bytes.append(x)
        cap_sensor_bytes.append(y)

    # Convert capacitive sensor bytes to capacitance values (in pF)
    # Formula from FDC1004 datasheet (page 16), using little-endian byte order as that is the format from the ESP32
    capData = [round(((int.from_bytes(bytes_data, byteorder="little", signed=True) / 524288.0) + (CAPDAC * 3.125)), 4) for bytes_data in cap_sensor_bytes]

    # Convert timestamp bytes to relative time (in microseconds)
    _esp32_timestamp = [int.from_bytes(bytes_data, byteorder="little") for bytes_data in esp32_timestamp_bytes]
    esp32_timestamp = [x - _esp32_timestamp[0] for x in _esp32_timestamp]

    # Close the serial port after data acquisition
    serialPort.close()

    # ------------------------------------------------------------------------------------------------------------------
    print(f"\nSaving data, please wait...")

    # Create the output folder if it doesn't exist
    dataFolderNamePath = pathlib.Path(__file__).parent.joinpath("Capacitance Data")
    if not dataFolderNamePath.is_dir():
        dataFolderNamePath.mkdir()

    # Generate sample numbers for each data point
    cap_data_num_samples = list(range(1, len(esp32_timestamp) + 1))

    # Define column headers for the CSV file
    _dataHeaders = {
        "A": "Sample No.",
        "B": "Timestamp (us)",
        "C": "Capacitance (pF)",
    }

    # Define metadata to include at the top of the CSV file
    dataMetadata = {
        "A": ["Data Collection Duration (s):", "Capacitive Sensor Sample rate (Hz):"],
        "B": [DATA_ACQUISITION_DURATION, _CAP_SENSOR_SAMPLING_RATE],
    }

    # Organize the main data content
    dataBody = {
        "A": cap_data_num_samples,
        "B": esp32_timestamp,
        "C": capData,
    }

    # Build the full header section with metadata
    dataHeaders = build_data_headers(_dataHeaders, dataMetadata)

    # Create DataFrames for headers and data
    header_df = pd.DataFrame(dataHeaders)
    dataBody_df = pd.DataFrame(dataBody)

    # Combine header and data into a single DataFrame
    capacitanceData_df = pd.concat([header_df, dataBody_df], ignore_index=True)

    # Save the DataFrame to a CSV file
    capacitanceData_df.to_csv(
        dataFolderNamePath.joinpath(DataFileName + ".csv"),
        index=False,
        header=False,
    )

    print(f"\nDone saving data")
