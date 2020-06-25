import os
import time


def setup_sim():
    print("==========================")
    print("     RESET SIMULATION     ")
    print("==========================")

    os.system("rosservice call /gazebo/pause_physics")
    print("Unpausing Physics...")
    os.system("rosservice call /gazebo/unpause_physics")
    print("Placing quadrotor...")
    os.system(
        "rosservice call /gazebo/set_model_state "
        "'{model_state: { model_name: delta, pose: { position: { x: 0.0, y: 0.0 ,z: 0.2 }, "
        "orientation: {x: 0, y: 0, z: 0.0, w: 1.0 }}, "
        "twist:{ linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 }}, "
        "reference_frame: world } }'")
    time.sleep(2)
    # start quadrotor
    # os.system("timeout 1s rostopic pub /hummingbird/bridge/arm std_msgs/Bool 'True'")
    # os.system("timeout 1s rostopic pub /hummingbird/autopilot/start std_msgs/Empty")