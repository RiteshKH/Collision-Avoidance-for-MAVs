## [Collision-Avoidance-for-MAVs](https://github.com/RiteshKH/Collision-Avoidance-for-MAVs)
A guidance strategy for a fixed wing Micro Air Vehicle(MAV) to avoid collision with
obstacles using only bearing measurements. The objective of the robot is to keep the obstacle at a constant
desired bearing angle in the sensor field of view. Robot proceeds in this way until it crosses the obstacle and
then proceeds towards the destination.The control strategies are developed using Lyapunov-based sliding
mode control, and it safely manoeuvres the MAV achieving collision avoidance.

### Instructions 

* Run `main.m` in this folder for simulations. 
* In [`main.m`](https://github.com/RiteshKH/Collision-Avoidance-for-MAVs/blob/master/main.m), uncomment the corresponding lines of code to run the corresponding simulation described.

![field_mult](https://user-images.githubusercontent.com/38212000/60295546-9f892500-9941-11e9-999b-b393281d9b06.JPG)
![horzWall](https://user-images.githubusercontent.com/38212000/60295707-fc84db00-9941-11e9-94db-aec2f151ee8e.JPG)
![rangenbearing](https://user-images.githubusercontent.com/38212000/60295731-11fa0500-9942-11e9-9091-de2f5a18f044.JPG)
![square](https://user-images.githubusercontent.com/38212000/60295763-263e0200-9942-11e9-8e98-1ddfb3c8b4bd.JPG)
