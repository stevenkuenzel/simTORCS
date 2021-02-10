# simTORCS
A lightweight Kotlin 2-d racing car environment with an abstract physics engine, aiming at training AI controllers for the application in [TORCS SCR][1].

## Features
* Lightweight 2-d racing car environment highly influenced by the TORCS Simulated Car Racing Championship
* Train AI racing car controllers fast (compared to the approach in [TORCS SCR][1])
* Import original TORCS tracks into a 2-d equivalent in simTORCS
* (Rudimentary) Visual observation of the race to verify the trained controllers

## Limitations
* The physics engine does not reach the degree of detail that TORCS have. For example the cars are accelerating with constant torque (however, that constant was determined in order to match the behaviour of **car1-trb1** as close as possible). Also,
* Only a limited number of sensors are supported (due to the abstract physics engine):
   * Distance raced on the track,
   * Angle to the track axis,
   * Distance to the track axis,
   * Driving speed, and
   * Track edge sensors
(* Althought TORCS SCR provides much more information to the controller, this subset of sensor values is sufficient to create a competitive racing car controller.)
* The track import is not perfectly working yet, however, five example tracks are provided: Brondehach, Forza, G-Track-1, G-Track-3 and Wheel-2.
* Cars are disqualified from the race when the leave the track or try driving backwards on the track. (This could also be mentioned as feature as it allows to train AI controllers that conduct neither of the two mentioned mistakes.)

## Credits

The source code of the physics engine is based on [this JavaScript implementation][2] by *spacejack*. The implementation hence foots on the [article by *M. Monster*][3].

## Quick Start
1. Checkout the project from GitHub and import it into your IDE
2. Run the application via the **main** method of the class **simtorcs.Main** (it provides the implementation of an exemplary race with two cars controlled by naive controllers).
3. Your own AI controller has to extend the **simtorcs.car.control.CarController** class.
4. Retrieve fitness information from the **simtorcs.car.SensorInformation** instance of your car. See the method **getFitness** in the class **simtorcs.Main** for an example.

## Future Work
* Improve the import function or provide all default TORCS tracks as working variants
* Consider other racing cars in the simulation
* Provide more sensor values

## Further Reading
Please refer to my dissertation which is available under: TO BE PUBLISHED SOON.


[1]: https://arxiv.org/pdf/1304.1672.pdf
[2]: https://github.com/spacejack/carphysics2d/tree/master/public/js
[3]: https://asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
