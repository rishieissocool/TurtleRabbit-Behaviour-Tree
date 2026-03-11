# Behaviour Tree

Multi-agent behaviour tree for RoboCup SSL robot control. Built on top of [py_trees](https://github.com/splintered-reality/py_trees).

Each robot runs its own subtree inside a parallel composite, so all robots tick simultaneously. The tree handles game state transitions (running, stopped, halted) and dispatches motor commands through a shared queue.

## Structure

```
behaviour_tree/
├── main_tree.py        # root tree - spawns per-robot subtrees in parallel
├── common_trees.py     # shared behaviours (world updates, ball position, sending commands)
├── test_tree.py        # go-to-ball test tree with look-at, move-to, dribble/kick
├── goalie_tree.py      # goalie - trajectory prediction + blocking
├── halt_sequence.py    # halt state handler (stop all motors)
├── stop_sequence.py    # stop state handler (stop + move away from ball)
├── cmd_mgr.py          # wraps RobotCommand creation and queue dispatch
├── velocity.py         # linear/angular velocity calculations with speed modes
├── move_away.py        # move robot away from a target position
└── run_bt_process.py   # entry point - runs the tree in a separate process
```

## Install

```bash
pip install -e .
```

Requires the `TeamControl` package for world model, networking, and robot movement modules.

## Usage

The tree is meant to run as a subprocess alongside the main TeamControl loop:

```python
from multiprocessing import Queue, Event
from behaviour_tree import run_bt_process

is_running = Event()
is_running.set()

dispatcher_q = Queue()
run_bt_process(is_running, wm, dispatcher_q)
```

Or build a tree directly:

```python
import py_trees
from behaviour_tree import MainTree

root = MainTree(wm, dispatch_q, logger)
bt = py_trees.trees.BehaviourTree(root)
bt.setup(timeout=15)
bt.tick()
```

## Game States

The tree checks `GameState` from the game controller each tick and activates the corresponding subtree per robot:

- **RUNNING** - normal play (go to ball, pass, kick, etc.)
- **STOPPED** - stop motors + move away from ball
- **HALTED** - full stop, no movement

## Dependencies

- `py_trees`
- `numpy`
- `TeamControl` (world model, networking, robot movement)
