package simtorcs

import simtorcs.car.Car
import simtorcs.car.control.TestController
import simtorcs.race.Race
import simtorcs.track.Track
import simtorcs.visualization.RaceAWTComponent
import javax.swing.JFrame
import kotlin.math.max
import kotlin.math.min

class Main {
    companion object {
        @JvmStatic

        fun main(args: Array<String>) {

            // Place an '!' before the track name to load the track with turns being inverted: Brondehach --> !Brondehach.
            // All tracks to be accessed have to be put into the 'input/tracks' directory.
            val trackBrondehach = Track.load("Brondehach", 1)

            // Create a race on Brondehach without noise. End the race after 120 [s]econds.
            val race = Race(trackBrondehach, false, Race.FPS * 120)

            // Create two cars with simple controllers. Note that the cars do not collide with each other.
            // I suggest using only a single car.
            val car = race.createCar()
            val car2 = race.createCar()
            car.setController(TestController(15.0))
            car2.setController(TestController(20.0))

            val jFrame = JFrame("simTORCS Race on ${race.track.name}")
            jFrame.add(RaceAWTComponent(race, 850))
            jFrame.setSize(900, 900)
            jFrame.isVisible = true

            // Speed up the visualization by a certain factor.
            val timeMultiplier = 8.0

            while (race.tNow < race.tMax && !race.raceFinished) {
                // Use the update-call to proceed step-wise. race.run() would process the whole race at once.
                race.update()

                // If all cars are disqualified, stop the race.
                if (race.cars.all { it.disqualified }) break

                jFrame.repaint()
                Thread.sleep((1000.0 / (timeMultiplier * Race.FPS)).toLong())
            }

            println("Fitness of car: " + getFitness(car).contentToString())
            println("Fitness of car2: " + getFitness(car2).contentToString())
        }


        /**
         * Example: The fitness functions described in my dissertation.
         */
        private fun getFitness(car: Car): Array<Double> {
            // Define the required constants:
            val metersPerTickAt180KMH = (180.0 / 3.6) * Race.DT
            val distanceAt180KMH = metersPerTickAt180KMH * car.race.tMax.toDouble()
            val distanceOnStraightsAt180KMH = distanceAt180KMH * (car.track.straightLength / car.track.length)

            // ... and the fitness functions. See my dissertation for the description.
            val fDistance = 1.0 - min(1.0, car.sensorInformation.distanceRaced / distanceAt180KMH)
            val fTurnSpeed = if (car.sensorInformation.ticksInOrBeforeTurns == 0) 1.0 else
                (car.sensorInformation.tooLowTurnSpeed / (car.sensorInformation.ticksInOrBeforeTurns * Race.FPS).toDouble()).coerceIn(
                    0.0,
                    1.0
                )

            val fDrivenStraight =
                1.0 - min(1.0, car.sensorInformation.lengthDrivenStraight / distanceOnStraightsAt180KMH)

            return arrayOf(fDistance, fTurnSpeed, fDrivenStraight)
        }
    }
}