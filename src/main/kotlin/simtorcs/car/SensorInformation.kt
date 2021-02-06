package simtorcs.car

import org.apache.commons.rng.UniformRandomProvider
import org.apache.commons.rng.simple.RandomSource
import kotlin.math.abs

class SensorInformation(val noise: Boolean, numOfSensors: Int) {
    companion object {
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
    }

    /**
     * Use the Mersenne Twister random number generator.
     */
    private val random by lazy { RandomSource.create(RandomSource.MT)!! }

    /**
     * Attributes in TORCS SCR to: distRaced.
     */
    var roundsFinished = 0
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

    fun perturbIfNecessary() {
        if (noise) {
//            var totalPerturbation = 0.0

            for (index in trackEdgeSensors.indices) {
                val actualValue = trackEdgeSensors[index]

                var perturbedValue = actualValue + MAX_RANDOM_DEVIATION * (random.nextDouble() - 0.5) * 2.0

                if (perturbedValue > 1.0) perturbedValue = 1.0
                if (perturbedValue < 0.0) perturbedValue = 0.0

//                totalPerturbation += abs(actualValue - perturbedValue)

                trackEdgeSensors[index] = perturbedValue
            }
        }
    }
}