package simtorcs.race

import simtorcs.car.Car
import simtorcs.track.Track

class Race(val track: Track, val tMax : Int = 6000) {

    val fps = 50
    val dt = 1.0 / fps.toDouble()
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

    fun update()
    {
        for (car in cars) {
            car.update(dt)
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