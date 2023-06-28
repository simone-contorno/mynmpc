from simple_launch import SimpleLauncher
 
def generate_launch_description():
    sl = SimpleLauncher()

    # Params
    robot = sl.declare_arg('robot', default_value='bike', description='r2d2 / bike')
    path = sl.declare_arg('path', default_value='circle', description='circle')

    # Map 
    sl.include('mobro_sim', 'sim_multi.py', launch_arguments = {'robot': robot})  

    # Nodes
    sl.node('mobro_sim', 'simulate', arguments = [robot, path])    # path
    sl.node('mynmpc', 'obstacle1')                                 # obstacle 1
    sl.node('mynmpc', 'obstacle2')                                 # obstacle 2
   
    return sl.launch_description()