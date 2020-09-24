2 files:
listener.py
runner.py

change run_with_start_ros = False var to True
first start runner.py
then start listener.py
listener.py will issue the start command, which runner.py subscribe to and will start the simulator. 

the runner will publish vehicle position in a dictionary which contains the vehicle id and current position of the vheicles within set distance of the intersection.

