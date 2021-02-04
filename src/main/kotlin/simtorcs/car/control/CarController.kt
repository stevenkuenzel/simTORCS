package simtorcs.car.control

import simtorcs.car.SensorInformation

abstract class CarController {
    abstract fun control(si : SensorInformation) : CarInput
}