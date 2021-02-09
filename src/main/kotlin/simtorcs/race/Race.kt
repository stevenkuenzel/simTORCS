package simtorcs.race

import simtorcs.car.Car
import simtorcs.track.Track

/**
 * A race instance.
 */
class Race(val track: Track, val noise: Boolean, val tMax: Int = 6000) {
    companion object {
        val FPS = 50
        val DT = 1.0 / FPS.toDouble()


        // Maximum and minimum speed considered.
        val SPEED_MIN = 50.0 / 3.6 // in [m/s]

        /**
         * The maximum speed of the car. Based on the approx. max. speed of car1-trb1 in TORCS.
         */
        val SPEED_MAX = 330.0 / 3.6 // in [m/s]

        /**
         * The maximum range covered by the track edge sensors.
         */
        val SENSOR_RANGE = 200.0 // in [m]eters.

        /**
         * The maximum deviation of the track edge sensor values in percent.
         */
        val MAX_RANDOM_DEVIATION = 0.05

        /**
         * FITNESS RELATED CONSTANTS.
         */

        /**
         * The maximum possible turn speed is multiplied with that constant. If the car is driving faster through the turn than the resulting value, it positively contributes to the corresponding fitness value.
         */
        val TURN_SPEED_MULTIPLIER = 0.95

        /**
         * The distance at which the possible speed is linearly reduced to the turn speed. In meters.
         */
        val TURN_SPEED_DISTANCE_MAX = 50.0

        /**
         * The maximum inclination of the steering wheel to be the car considered as driving straight. (In percent of STEER_MAX radians).
         */
        val THRESHOLD_DRIVING_STRAIGHT = 0.05

    }

    val cars = mutableListOf<Car>()
    var tNow = 0
    var raceFinished = false


    fun run() {
        while (tNow < tMax && !raceFinished) {
            update()
        }

        if (!raceFinished) raceEnd()
    }

    fun createCar(): Car {
        val car = Car(this, noise)
        cars.add(car)

        return car
    }

    fun update() {
        for (car in cars) {
            car.update(DT)
        }

        if (cars.all { it.disqualified }) raceEnd()

        tNow++
    }

    fun raceEnd() {
        raceFinished = true

        cars.forEach { it.raceEnd() }
    }
}