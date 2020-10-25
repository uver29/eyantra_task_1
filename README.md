    # navigate to the workspace
    cd ~/catkin_ws/src/
    
    # clone the repo
    git clone https://github.com/uver29/ebot_controller.git
    
    # build the package
    cd ..
    catkin build
    
    # make the script executable
    chmod +x ~/catkin_ws/src/ebot_description/controller.py
    
    # launch the obstacle course and spawn the bot
    roslaunch ebot_description task1.launch
    
    # run the controller script
    rosrun ebot_controller controller.py

