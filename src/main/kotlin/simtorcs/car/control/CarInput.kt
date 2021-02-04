package simtorcs.car.control

data class CarInput(var left : Double, var right : Double, var throttle : Double, var brake : Double) {
    constructor() : this(0.0, 0.0, 0.0, 0.0)
}