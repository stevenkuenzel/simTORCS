//package simtorcs.car.control
//
//import elements.phenotype.NetworkPhenotype
//import simtorcs.car.SensorInformation
//import kotlin.math.exp
//import kotlin.math.max
//import kotlin.math.min
//
//class ANNController(val network: NetworkPhenotype) : CarController() {
//
//    override fun control(si: SensorInformation): CarInput {
//        val input = DoubleArray(2 + si.sensorData.size)
//        var inputIndex = 0
//        input[inputIndex++] = min(1.0, si.absVel / si.maxSpeed)
//        input[inputIndex++] = (min(1.0, max(-1.0, si.distanceToTrackAxis)) + 1.0) / 2.0
//
//        for (d in si.sensorData) {
//            input[inputIndex++] = d
//        }
//
//        val output = network.update(input)
//        val forceLeft = output[0]
//        val forceRight = output[1]
//
//        val targetSpeed = si.minSpeed + output[2] * (si.maxSpeed - si.minSpeed)
//        val accel_and_brake = 2.0 / (1.0 + exp(si.absVel - targetSpeed)) - 1.0
//
//        return CarInput(
//            forceLeft,
//            forceRight,
//            if (accel_and_brake > 0.0) accel_and_brake else 0.0,
//            if (accel_and_brake < 0.0) -accel_and_brake else 0.0
//        )
//    }
//}