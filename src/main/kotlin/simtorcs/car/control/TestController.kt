package simtorcs.car.control

import simtorcs.car.SensorInformation
import kotlin.math.exp

/**
 * A simple controller driving at _targetSpeed_ always steering towards the track axis.
 */
class TestController(private val targetSpeed: Double) : CarController() {
    override fun control(si: SensorInformation): CarInput {
        val targetSteer = si.angleToTrackAxis - si.distanceToTrackAxis * 0.5

        val accelerationAndBrake = 2.0 / (1.0 + exp(si.absoluteVelocity - targetSpeed)) - 1.0

        return CarInput(
            if (targetSteer > 0.0) targetSteer else 0.0,
            if (targetSteer < 0.0) -targetSteer else 0.0,
            if (accelerationAndBrake > 0.0) accelerationAndBrake else 0.0,
            if (accelerationAndBrake < 0.0) -accelerationAndBrake else 0.0
        )
    }
}