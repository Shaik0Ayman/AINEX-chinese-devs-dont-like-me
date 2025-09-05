# Action Group Controller Guide

This file describes a tiny controller that plays "action groups" using predefined multi-servo motions from a SQLite file. It sends targets to a servo control board.

## Overview

This controller manages and executes action groups by reading from a designated SQLite file and controlling servos via an external board interface.

### Assumptions

The servo controller object (`board`) exposes:

- `bus_servo_set_position(duration_s, data)`: where `data` is a list like `[[id, position], ...]`
- `stopBusServo([ids...])`: stops servos immediately

### Class: ActionGroupController

#### State

- `self.board`: External board interface (injected).
- `self.running_action`: Indicates if an action is currently playing.
- `self.stop_running`: Flag to stop after the current step.
- `self.action_path`: Folder path containing the `ActionGroups/` directory.

#### Methods

- `stop_servo()`: Immediately stops all bus servos.

  - **Usage**: Calls `board.stopBusServo` for servo IDs 1 to 24.

- `stop_action_group()`: Requests a graceful stop.

  - **Function**: Sets `self.stop_running = True`, breaking the playback loop after the current step.

- `action_finish()`: Checks if an action is running.

  - **Return**: `True` if running, `False` otherwise.

- `run_action(actNum)`: Loads and plays an action group file.

  - **Flow**:
    - Resolve `.d6a` file path.
    - Set `stop_running = False`.
    - If no action is running:
      - Open SQLite connection to the `.d6a` file.
      - Select all rows from `ActionGroup`.
      - Parse and execute each row.
      - Call `board.bus_servo_set_position(duration_s, data)`.
      - Sleep to wait for completion.
    - Close DB and set `running_action = False`.
    - If file missing, log a message.

## Action Group File Format (.d6a as SQLite)

### Expected Layout

- SQLite DB with a table `ActionGroup`.
- Row layout:
  - Column 0: (unused)
  - Column 1: Duration in milliseconds
  - Columns 2..N: Servo positions for IDs 1 to (N-1)

### Important Behaviors and Caveats

- **Blocking Call**: `run_action` sleeps between steps; execute in a separate thread for responsiveness.
- **Stop Semantics**:
  - `stop_action_group()`: Graceful stop after current step.
  - `stop_servo()`: Immediate hard stop.
- **Single-Run Guard**: Prevents multiple actions from running simultaneously.
- **No Return Status**: Use `action_finish()` to check if running.

### Minimal Usage Example

Create an instance and play an action:

```python

import action_group_controller as controller
controller.runAction('left_shot')
controller.stop_action_group()      
controller.stop_servo()             