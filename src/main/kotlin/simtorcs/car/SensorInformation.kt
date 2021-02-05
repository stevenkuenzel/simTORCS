package simtorcs.car

import org.apache.commons.rng.UniformRandomProvider
import org.apache.commons.rng.simple.RandomSource
import kotlin.math.abs

class SensorInformation(val noise : Boolean, numOfSensors : Int) {
    val random : UniformRandomProvider = RandomSource.create(RandomSource.MT)!! // Use the Mersenne Twister random number generator.
//    val minSpeed = 50.0 / 3.6
    val maxSpeed = 330.0 / 3.6
    val sensorRange = 200.0

    var angleToTrackAxis = 0.0
//    var angleToTrackIdeal = 0.0

    var roundsFinished = 0
    var segmentPosition = 0.0

    var distanceToTrackAxis = 0.0
//    var distanceToTrackIdeal = 0.0

    var absoluteVelocity = 0.0

    var sensorData = Array(numOfSensors) {0.0}

    fun finish()
    {
        if (noise)
        {
            perturb()
        }
    }

    fun perturb()
    {
        val maxDeviation = 0.05

        var totalPerturbation = 0.0

        for (index in sensorData.indices) {
            val actualValue = sensorData[index]

            var perturbedValue = actualValue + maxDeviation * (random.nextDouble() - 0.5) * 2.0

            if (perturbedValue > 1.0) perturbedValue = 1.0
            if (perturbedValue < 0.0) perturbedValue = 0.0

            totalPerturbation += abs(actualValue - perturbedValue)

            sensorData[index] = perturbedValue
        }
    }
}