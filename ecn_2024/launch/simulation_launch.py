#!/usr/bin/env python3

from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    sl.include('map_simulator', 'simulation2d_launch.py',
               launch_arguments={'map': sl.find('map_simulator', 'void.yaml'),
                                 'map_server': False,
                                 'display': False,
                                 'rate': 50})

    sl.node('ecn_2024', 'letter2path')

    sl.rviz(sl.find('ecn_2024', 'config.rviz'))

    for name, (r,g,b) in (('r2d1', [1,0,1]),
                      ('r2d2', [1,0,0]),
                      ('r2d3', [0,1,0]),
                      ('r2d4', [0,0,1]),
                      ('r2d5', [1,1,0])):

        with sl.group(ns = name):

            sl.robot_state_publisher('map_simulator', 'r2d2.xacro',
                                     xacro_args={'rgb': f'"{r} {g} {b}"', 'prefix': name+'/'})

            sl.call_service('/simulator/spawn',
                            {'robot_namespace': '/'+name,
                             'x': 10*float(name[-1]),
                             'y': 10.,
                             'static_tf_odom': True,
                             'force_scanner': False})

    return sl.launch_description()
