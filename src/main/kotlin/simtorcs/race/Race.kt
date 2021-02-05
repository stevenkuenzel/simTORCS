package simtorcs.race

import simtorcs.car.Car
import simtorcs.track.Track

/**
 * A race instance.
 */
class Race(val track: Track, val noise : Boolean, val tMax : Int = 6000) {
    companion object
    {
        val FPS = 50
        val DT = 1.0 / FPS.toDouble()

    }

    val cars = mutableListOf<Car>()
    var tNow = 0
    var raceFinished = false


    fun run()
    {
        while (tNow < tMax && !raceFinished)
        {
            update()
        }

        if (!raceFinished) raceEnd()
    }

    fun createCar() : Car
    {
        val car = Car(this, noise)
        cars.add(car)

        return car
    }

    fun update()
    {
        for (car in cars) {
            car.update(DT)
        }

        if (cars.all { it.disqualified }) raceEnd()

        tNow++
    }

    fun raceEnd()
    {
        raceFinished = true

        cars.forEach { it.raceEnd() }
    }
}