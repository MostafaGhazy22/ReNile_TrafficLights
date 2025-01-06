
# Traffic Light Control System with Pedestrian Crossing (ReNile)

## Overview
This project implements a traffic light control system for a four-way intersection using an STM32 microcontroller. The system includes pedestrian crossing functionality, with requests triggered via a push button. The project features a Finite State Machine (FSM) to handle traffic light states and pedestrian crossings, ensuring safety and smooth transitions.

---

## Features
1. **Traffic Light Control**:
   - North-South (NS) and East-West (EW) traffic lights with green, yellow, and red states.
2. **Pedestrian Crossing**:
   - Controlled via a push buttons.
   - Pedestrian green light activates after ensuring all traffic lights are red.
3. **Finite State Machine**:
   - Efficient state transitions based on timers and button requests.
4. **Non-blocking Timers**:
   - Uses `HAL_GetTick()` for precise timing.

---

## Hardware Requirements
1. **Microcontroller**: STM32F401.
2. **Traffic Light LEDs**:
   - 3 LEDs for North: Green, Yellow, Red.
   - 3 LEDs for South: Green, Yellow, Red.
   - 3 LEDs for East: Green, Yellow, Red.
   - 3 LEDs for West: Green, Yellow, Red.
   - 4 RGB LEDs for pedestrian signals.
3. **Push Button**:
   - 4 button for pedestrian crossing request.
4. **Breadboard and Jumper Wires** for connections.

---

## Setup Instructions

### 1. Hardware Setup
#### Traffic Lights
| Traffic Light | GPIO Pin  |
|---------------|-----------|
| NS Green LED  | PA2       |
| NS Yellow LED | PA1       |
| NS Red LED    | PA0       |
| EW Green LED  | PA5       |
| EW Yellow LED | PA6       |
| EW Red LED    | PA7       |
| Ped Green LED | PA3       |
| Ped Red LED   | PA4       |

#### Push Button
| Button                | GPIO Pin |
|-----------------------|----------|
| Pedestrian Button     | PA8      |

#### Connections
1. Connect LEDs to the respective GPIO pins with resistors 220Ω .
2. Connect the push button:
   - One terminal to PA8 (pedestrian button pin).
   - The other terminal to GND.
3. Ensure the STM32 board is powered via USB or external power supply.

---

### 2. STM32CubeIDE Configuration
1. **Clock Configuration**:
   - Set the system clock to 48 MHz using the HSI oscillator.
2. **GPIO Configuration**:
   - Configure LED pins (PA0–PA7) as **Output**.
   - Configure the pedestrian button pin (PA8) as **Input with EXTI (Interrupt)**.
3. **Interrupts**:
   - Enable EXTI interrupts for the pedestrian button (falling edge trigger).
4. **Generate Code**:
   - Generate the initialization code and open it in STM32CubeIDE.

---

### 3. STM32CubeIDE Setup
1. Open the project in STM32CubeIDE.
2. Copy and paste the FSM implementation code into the `main.c` file.
3. Add the following functions in `main.c`:
   - `trafficLightFSM()`: Manages state transitions.
   - `updateLights()`: Updates LEDs based on the current state.
   - `HAL_GPIO_EXTI_Callback()`: Handles pedestrian button interrupts.
4. Build the project and flash the firmware to the STM32 board.

---

## Testing
1. **Initial State**:
   - NS Green and EW Red lights are active by default.
2. **Timer Transitions**:
   - Observe the lights transitioning between green, yellow, and red based on configured timings.
3. **Pedestrian Request**:
   - Press the pedestrian button.
   - All lights turn red, and the pedestrian green light activates.

---

### Traffic Light States
1. **NS_GREEN_EW_RED**: NS green, EW red.
2. **NS_YELLOW_EW_RED**: NS yellow, EW red.
3. **NS_RED_EW_GREEN**: NS red, EW green.
4. **NS_RED_EW_YELLOW**: NS red, EW yellow.
5. **NS_RED_EW_RED**: All red.
6. **PED_CROSSING**: Pedestrian green light.

---

## Customization
1. **Timing Adjustments**:
   - Modify `GREEN_TIME`, `YELLOW_TIME`, `ALL_RED_TIME`, and `PED_TIME` macros in `main.c` to change light durations.
2. **Additional Features**:
   - Integrate IoT functionality to monitor and control the system remotely.

---