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


    /**
     * Attributes in TORCS SCR to: distRaced.
     */
    var roundsFinished = 0
    var lapPosition = 0.0
    var segmentPosition = 0.0

    /**
     * Equivalent in TORCS SCR: distRaced.
     */
    val distanceRaced get() = roundsFinished.toDouble() * track.length + lapPosition + segmentPosition

    /**
     * The summed speed difference in or before turns (to the respective maximum possible speed). See my dissertation for the description.
     */
    var ticksInOrBeforeTurns = 0

    /**
     * The number of ticks driven in or before turns. See my dissertation for the description.
     */
    var tooLowTurnSpeed = 0.0

    /**
     * The summed distance in [m]eters driven without steering (steering wheel moved below certain threshold). See my dissertation for the description.
     */
    var lengthDrivenStraight = 0.0

    fun perturbIfNecessary() {
        if (noise) {
            for (index in trackEdgeSensors.indices) {
                val actualValue = trackEdgeSensors[index]

                var perturbedValue = actualValue + Race.MAX_RANDOM_DEVIATION * (random.nextDouble() - 0.5) * 2.0

                if (perturbedValue > 1.0) perturbedValue = 1.0
                if (perturbedValue < 0.0) perturbedValue = 0.0

                trackEdgeSensors[index] = perturbedValue
            }
        }
    }
}