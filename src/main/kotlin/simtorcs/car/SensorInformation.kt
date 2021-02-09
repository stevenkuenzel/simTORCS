package simtorcs.car

import org.apache.commons.rng.simple.RandomSource
import simtorcs.race.Race
import simtorcs.track.Track

class SensorInformation(val noise: Boolean, val track: Track, numOfSensors: Int) {


    /**
     * Use the Mersenne Twister random number generator.
     */
    private val random by lazy { RandomSource.create(RandomSource.MT)!! }

    /**
     * Attributes in TORCS SCR to: distRaced.
     */
    var roundsFinished = 0
    var lapPosition = 0.0
    var segmentPosition = 0.0

    /**
     * Equivalent in TORCS SCR: angle.
     */
    var angleToTrackAxis = 0.0

    /**
     * Equivalent in TORCS SCR: trackPos.
     */
    var distanceToTrackAxis = 0.0

    /**
     * Equivalent in TORCS SCR: speed.
     */
    var absoluteVelocity = 0.0

    /**
     * Equivalent in TORCS SCR: track.
     */
    var trackEdgeSensors = Array(numOfSensors) { 0.0 }



    val distanceRaced get() = roundsFinished.toDouble() * track.length + lapPosition + segmentPosition

    var ticksInOrBeforeTurns = 0
    var tooLowTurnSpeed = 0.0
    var lengthDrivenStraight = 0.0

    fun perturbIfNecessary() {
        if (noise) {
//            var totalPerturbation = 0.0

            for (index in trackEdgeSensors.indices) {
                val actualValue = trackEdgeSensors[index]

                var perturbedValue = actualValue + Race.MAX_RANDOM_DEVIATION * (random.nextDouble() - 0.5) * 2.0

                if (perturbedValue > 1.0) perturbedValue = 1.0
                if (perturbedValue < 0.0) perturbedValue = 0.0

//                totalPerturbation += abs(actualValue - perturbedValue)

                trackEdgeSensors[index] = perturbedValue
            }
        }
    }
}