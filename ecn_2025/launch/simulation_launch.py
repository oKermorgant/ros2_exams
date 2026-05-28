#!/usr/bin/env python3

from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    sl.include('baxter_simple_sim', 'sim_launch.py',
               launch_arguments={'rviz': False, 'lab': 'none', 'zero_joints': True})

    sl.rviz(sl.find('ecn_2025', 'config.rviz'))
    sl.node('ecn_2025', 'ball.py')

    return sl.launch_description()
