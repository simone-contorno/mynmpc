from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    # Params
    robot = sl.declare_arg('robot', default_value='rosbot', description='rosbot')
    path = sl.declare_arg('path', default_value='fix', description='fix / dyn')
    
    # Map 
    sl.include('mobro_sim', 'sim_rosbot_amcl.py')

    # Nodes
    sl.node('mobro_sim', 'simulate', arguments = [robot, path])  # paths
    sl.node('mynmpc', 'obstacle')                                # obstacle 
    
    return sl.launch_description()