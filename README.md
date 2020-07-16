## CoppeliaSim multiprocessing

# warning / notes:

You must set the environment variable `COPPELIASIM_MODEL_PATH` to the `3d_models` directory such the the code knows where are the tap, button, lever, and kuka.

Some methods of the `SimulationConsumer` return tuples (see `get_data` for instance).
When using the `SimulationPool`, calling `my_pool.get_data()` will return a list for each process of tuples, instead of a tuple of lists for each process:

```
print(my_pool.get_data())

[
  (state_proc0, current_goal_proc0),
  (state_proc1, current_goal_proc1),
  (state_proc2, current_goal_proc2),
  ...
]
```

Naturally one would rather expect

```
(
  [state_proc0, state_proc1, state_proc2, ...],
  [current_goal_proc0, current_goal_proc1, current_goal_proc2, ...]
)
```

Have fun
