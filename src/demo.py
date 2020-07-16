from simulation import SimulationProducer, SimulationPool, MODEL_PATH
import time
import numpy as np


def test1():
    simulation = SimulationProducer(gui=True)
    simulation.add_arm()
    simulation.add_button(position=(1, 0, 0))
    simulation.add_lever(position=(0, 1, 0))
    simulation.add_tap(position=(-1, 0, 0))
    simulation.start_sim()

    for i in range(200):
        simulation.step_sim()

    simulation.set_stateful_objects_states([1, 1, 1])

    for i in range(200):
        simulation.step_sim()

    simulation.stop_sim()
    simulation.close()


def test2():
    simulation = SimulationProducer(
        scene=MODEL_PATH + '/custom_timestep.ttt',
        gui=False
    )
    simulation.create_environment('one_arm_2_buttons_1_levers_1_tap')
    dt = 0.05
    simulation.set_simulation_timestep(dt)
    simulation.set_control_loop_enabled(False)
    simulation.start_sim()
    upper_limits = simulation.get_joint_upper_velocity_limits()
    n_joints = simulation.get_n_joints()

    N = 1000

    frequencies = np.random.randint(
        low=5 / dt, high=8 / dt, size=n_joints)[np.newaxis]
    x = np.arange(N)[:, np.newaxis]
    velocities = np.sin(x / frequencies * 2 * np.pi) * upper_limits

    t0 = time.time()

    for i in range(N):
        simulation.step_sim()
        simulation.set_joint_target_velocities(velocities[i])
        a = simulation.get_joint_forces()

    t1 = time.time()

    print("{} iteration in {:.3f} sec ({:.3f} it/sec)".format(
        N,
        t1 - t0,
        N / (t1 - t0)
    ))
    simulation.stop_sim()
    simulation.close()


def test3():
    pool_size = 4
    simulations = SimulationPool(
        pool_size,
        scene=MODEL_PATH + '/custom_timestep.ttt',
        guis=[]
    )
    simulations.create_environment('one_arm_2_buttons_1_levers_1_tap')
    dt = 0.05
    simulations.set_simulation_timestep(dt)
    simulations.set_control_loop_enabled(False)
    simulations.start_sim()
    with simulations.specific(0):
        upper_limits = simulations.get_joint_upper_velocity_limits()[0]
        n_joints = simulations.get_n_joints()[0]

    N = 1000

    frequencies = np.random.randint(
        low=5 / dt, high=8 / dt, size=n_joints)[np.newaxis]
    x = np.arange(N)[:, np.newaxis]
    velocities = np.sin(x / frequencies * 2 * np.pi) * upper_limits

    t0 = time.time()

    for i in range(N):
        simulations.step_sim()
        simulations.set_joint_target_velocities(velocities[i])
        a = simulations.get_data()
        print(a)

    t1 = time.time()

    print("{} iteration in {:.3f} sec ({:.3f} it/sec)".format(
        N * pool_size,
        t1 - t0,
        N * pool_size / (t1 - t0)
    ))
    simulations.stop_sim()
    simulations.close()


if __name__ == '__main__':
    import sys
    func_name = sys.argv[1]

    if func_name == 'test1':
        test1()
    elif func_name == 'test2':
        test2()
    elif func_name == 'test3':
        test3()
