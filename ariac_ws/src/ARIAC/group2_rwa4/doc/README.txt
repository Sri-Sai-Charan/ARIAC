README:

#### HOW TO RUN THE CODE ###

Build the package
    • Extract the Package
    • paste group2_rwa4 in the src folder in ariac_ws
    • catkin clean -y && catkin build
    • source devel/setup.bash

Commands to run the package
    • roslaunch group2_rwa4 ariac.launch

Note:
    • As long as the real time factor is below 0.8,  the package shouldn't throw any error.
    • If the RTF is below 0.8, please adjust the step size in the nist_ gear / worlds / ariac.template file.
