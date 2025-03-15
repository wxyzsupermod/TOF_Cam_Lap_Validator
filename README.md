# RotorHazardLidar
This is a plugin for Rotorhazard to help reject unwanted laps allowing a lowering of the requirements for the vtx's to be calibrated as sensitively as they are currently.
On race start the plugin will start scanning with the lidar and watch the start finish gate.
On a lap event being registered the plugin will compare the lap time with the crossing detected by lidar if the crossing time is essentially equal we have registered a true lap and it will be allowed otherwise the lap will be deleted.
