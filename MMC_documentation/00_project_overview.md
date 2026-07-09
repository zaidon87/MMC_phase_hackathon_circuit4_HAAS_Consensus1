# Project Overview

## Research theme

The project investigates decentralized control for Modular Multilevel Converters (MMCs), focusing on local submodule decisions and neighbor-only capacitor-voltage balancing.

## Main idea

Instead of a centralized sorting controller that receives all capacitor voltages and sends all insertion commands, each submodule receives only local information:

- its own capacitor voltage,
- previous neighbor capacitor voltage,
- next neighbor capacitor voltage,
- local or arm-level current direction,
- common arm modulation reference.

This creates a distributed balancing mechanism that is easier to map to modular hardware.

## Current model scope

The uploaded model is a research prototype. It contains repeated submodule/hardware-oriented blocks and MATLAB Function logic for local balancing. It should be treated as a working base for refactoring and validation, not yet as a final publication model.
