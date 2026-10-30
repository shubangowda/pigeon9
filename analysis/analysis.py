# analysis.py
# Post-flight analysis script. It loads the flight log from the SD card and replays the data through the same Kalman filter logic used on the rocket.
# It is used to tune the filter noise values (Q and R) against real data and to plot actual flight results against OpenRocket predictions.
# Everything lives in this one file until it gets too big to read comfortably.
