package simtorcs.car.control

import simtorcs.car.SensorInformation
import kotlin.math.exp

class TestController(val initialTurnSpeed: Double) : CarController() {
    override fun control(si: SensorInformation): CarInput {
        val targetSteer = si.angleToTrackAxis - si.distanceToTrackAxis * 0.5

        val accel_and_brake = 2.0 / (1.0 + exp(si.absVel - initialTurnSpeed)) - 1.0

        return CarInput(
            if (targetSteer > 0.0) targetSteer else 0.0,
            if (targetSteer < 0.0) -targetSteer else 0.0,
            if (accel_and_brake > 0.0) accel_and_brake else 0.0,
            if (accel_and_brake < 0.0) -accel_and_brake else 0.0
        )
    }
}