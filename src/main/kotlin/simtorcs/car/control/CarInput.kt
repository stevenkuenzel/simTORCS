package simtorcs.car.control

data class CarInput(var left : Double = 0.0, var right : Double = 0.0, var throttle : Double = 0.0, var brake : Double = 0.0)