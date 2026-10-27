# Buck Converter Design and Cascaded PI Control

Design and control of a buck converter for battery-based energy systems, with cascaded current and voltage PI loops, plus a voltage and current measurement board.

[TODO: confirm supervisor approval before making this repository public.]

<!-- ![Overview](images/hero.png) -->

## Overview

[TODO: 2 to 3 sentences on the goal and the main result.]

## Control design

[TODO: converter specifications, inner current loop, outer voltage loop, tuning method.]

## Measurement board

Specifications, from the project report:

- Measurement range: 0 to 40 V and 0 to 40 A
- Current sensing with a Hall sensor
- Voltage divider and optocoupler stage
- Output signal: 0 to 3 V
- Board size: 120 x 100 mm

[TODO: schematic, PCB layout and bill of materials, designed in EasyEDA.]

## Results

[TODO: step responses, Bode plots, root locus and steady-state error.]

## Repository structure

```
simulation/                 PSIM models
analysis/                   MATLAB scripts and data
hardware/measurement-board/ schematic, PCB, gerbers, bill of materials
images/                     plots and photos
```

## How to reproduce

[TODO: tools, versions, simulation and analysis steps.]

## Author

Felipe Martins, Mechatronics Engineering student at UTFPR, double degree at ENSTA Bretagne.
[LinkedIn](https://www.linkedin.com/in/felipe-m-k-m-martins)
