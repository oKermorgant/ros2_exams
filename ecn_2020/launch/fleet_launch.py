from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    for robot, target in (('bb8', 'r2d2'),
                          ('d0','bb8')):

        sl.node('ecn_2020', 'control', name = robot, namespace = robot,
                parameters = {'base_frame': robot+'/base_link',
                              'target_frame': target+'/base_link'})


    
    return sl.launch_description()
