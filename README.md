## CoppeliaSim multiprocessing

# warning / notes:

Some methods of the `SimulationConsumer` return tuples (see `get_data` for instance).
When using the `SimulationPool`, calling `my_pool.get_data()` will return a list for each process of tuples, instead of a tuple of lists for each process.

Have fun
