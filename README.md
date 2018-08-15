# gazebo_python_swig
gazebo server interface (SWIG) to control simulation in python 

# Compile
    git clone https://github.com/lgesing/gazebo_python_swig.git
    cd gazebo_python_swig
    cd build
    cmake ../
    make

# Run
    cd gazebo_python_swig
    # start server
    python test_gazebo_sim.py
    # start gazebo-gui
    gzclient
    
