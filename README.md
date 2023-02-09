# PoolTechClimateControl
This is AiK's Arduino project to control the ventilation and water level in the pool tech shaft.

<add a picture of the PooLTechShaft>

The base for this project is https://github.com/MakeMagazinDE/Taupunktluefter. This project caught my attention because it calculates the dew point from the outside and the room (or in my case a dark and wet shaft) you want to get dry and decides based on that if it makes sense to activate the fan to suck "dryer" air from the outside.

I adapted it a little bit, but the base idea/concept is unchanged.

Here is the list of my adoptions (on top of the advanced version with SD card logging):
    * translated it from German to English
    * added a second pair of sensors (now 2 inside and 2 outside) to detect defect sensors and false measures
    * store 3 measures to average the results (again to compensate false measures)
    * added a push button to activate the display
    * added a sensor if there is water to activate a water pump


<add a picture of the PTCC> 
