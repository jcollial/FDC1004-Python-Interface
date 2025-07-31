# FDC1004 Python Interface

This repository provides a Python interface to collect and log data from an **FDC1004 capacitive sensor** connected to an **ESP32** microcontroller. The ESP32 sends data via serial communication, which is then processed and saved using Python.

---

### Features

- Communicates with ESP32 over serial port
- Configurable CAPDAC and sampling parameters
- Parses and logs timestamped capacitance data
- Saves data to CSV with metadata and headers

---

### Installation

```bash
# Clone the repository
git clone https://github.com/jcollial/FDC1004-Python-Interface.git

# Navigate into the project directory
cd <your-path>/FDC1004-Python-Interface

# Create a virtual environment
python -m venv venv
```

#### Activate the virtual environment

- **Windows (Command Prompt)**:
  ```bash
  venv\Scripts\activate
  ```

- **Windows (PowerShell)**:
  ```bash
  .\venv\Scripts\Activate.ps1
  ```

#### Install dependencies

```bash
python -m pip install -r requirements.txt
```

To deactivate the virtual environment:
```bash
deactivate
```

---

### Usage

1. **Upload the firmware**  
   Flash the `FDC1004-Serial-Python.ino` sketch (located in `hardware/FDC1004-Serial-Python/`) to your ESP32.

2. **Connect the ESP32**  
   Ensure the ESP32 is connected to your PC via USB and note the COM port (e.g., `COM3`).

3. **Run the logger**  
   Edit the `capdac-logger.py` file if needed to set:
   - `port` (e.g., `"COM3"`)
   - `CAPDAC` value (0–31)
   - `DATA_ACQUISITION_DURATION` (in seconds)

   Then run:
   ```bash
   python capdac-logger.py
   ```

4. **Output**  
   - Data is saved in the `Force Data/` folder as a CSV file.
   - The CSV includes metadata, timestamps, and capacitance values.

---

### Output Format

The CSV file includes:

- **Metadata**: Duration, sampling rate
- **Headers**: Sample number, timestamp (µs), capacitance (pF)
- **Data**: One row per sample

---

### Notes

- The ESP32 must acknowledge each command before data acquisition begins.
- Each sample includes a 4-byte timestamp and a 4-byte capacitance value.
- The CAPDAC value adjusts the sensor's measurement range.

---

### References

- [FDC1004 Datasheet](https://www.ti.com/lit/ds/symlink/fdc1004.pdf)
- ESP32 Documentation
