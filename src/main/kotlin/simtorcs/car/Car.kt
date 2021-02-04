package simtorcs.car

import simtorcs.car.control.CarController
import simtorcs.geometry.LineSegment
import simtorcs.geometry.Vector2
import simtorcs.race.Race
import simtorcs.track.Segment
import simtorcs.util.GeometryUtils
import kotlin.math.*

class Car(val race: Race, val noise: Boolean) {
    var printDebug = false
    var isTraining = true

    private var controller: CarController? = null

    fun setController(controller: CarController) {
        this.controller = controller
    }

    fun raceEnd() {
        distanceRaced = si.roundsFinished.toDouble() * race.track.length

        if (currentSegment != null) {
            distanceRaced += lastValidSegment!!.totalTrackLength + si.segmentPosition
        } else if (lastValidSegment != null) {
            distanceRaced += lastValidSegment!!.totalTrackLength
        }

        if (printDebug) {
            println("Avg Speed = ${3.6 * totalVel / (currentStep.toDouble() / race.fps.toDouble())} km/h")
            println("Distance raced = $distanceRaced m")
        }
    }

    val track = race.track

    var sensors: Array<Vector2>? = null
    var indicesToConsider = Array(0) { 0 }
    var previousSegment: Segment? = null
    var wasOffTrack = false
    var visitedSegments = 0
    var distanceRaced = 0.0

    var totalDistanceToIdeal = 0.0

    var speedReachedMax = 0.0

    var vTarget = 0


    val maxSteer = 0.366519  // Maximum steering angle in radians

    val sensorValues = defineSensorAngles(-45, 45, 5)
//    val eds = EngineDataSampler(0.05, EngineDataSampler.EDSMode.Deceleration)

    var currentSegment: Segment? = null
    var lastValidSegment: Segment? = null

    var currentStep = 0
    var totalDistanceFromTrack = 0.0
    var totalSpeed = 0.0

    var disqualified = false

    var heading = track.startingDirection// 0.0
    var position = track.startingPoint.copy() // Vector2(1.0, 0.0)


    var velocity = Vector2()
    var velocity_c = Vector2()
    var accel = Vector2()
    var accel_c = Vector2()
    var absVel = 0.0
    var yawRate = 0.0

    var throttle = 0.0
    var brake = 0.0
    var steerAngle = 0.0


    // Densitity: https://www.sunocoracefuels.com/tech-article/specific-gravity-matter#:~:text=For%20fuels%2C%20specific%20gravity%20can,%3A%205.95%20%2F%208.325%20%3D%200.715.
    val fuelDensity = 0.59 // kg/l
    var fuel = 94.0

    // Car configuration.
    val gravity = 9.81  // m/s^2
    val mass = 1150.0 + fuel * fuelDensity  // kg
    val inertiaScale = 1.0  // Multiply by mass for inertia
    val halfWidth = 0.97 // Centre to side of chassis (metres)
    val cgToFront = 2.3504 // Centre of gravity to front of chassis (metres)
    val cgToRear = 2.1696   // Centre of gravity to rear of chassis
    val cgToFrontAxle = 0.97  // Centre gravity to front axle
    val cgToRearAxle = 0.97  // Centre gravity to rear axle
    val cgHeight = 0.25  // Centre gravity height


//    val wheelRadius = 0.2796  // Includes tire (also represents height of axle)

    val tireGrip = 2.0 // 1.75 // 2.0  // How much grip tires have
//    val lockGrip = 0.7  // % of grip available when wheel is locked


    // ANNRACER:
    // acc: 8.579664582906522 * x + 72.34940739001144
    // dec: -90.13773558596135 * x + 191.35322287960696

    val engineForce = 11900.0 //3060.0 with dragconstant=0.0625
    val brakeForce = 26300.0 // 28620.0 with dragconstant=0.0625

    val weightTransfer = 0.2  // How much weight is transferred during acceleration/braking
    val cornerStiffnessFront = 5.0
    val cornerStiffnessRear = 5.2
    val airResist = 2.5    // air resistance (* vel)
    val rollResist = 8.0   // rolling resistance force (* vel)

    val inertia = mass * inertiaScale
    val wheelBase = cgToFrontAxle + cgToRearAxle
    val axleWeightRatioFront = cgToRearAxle / this.wheelBase // % car weight on the front axle
    val axleWeightRatioRear = cgToFrontAxle / this.wheelBase // % car weight on the rear axle


    /**
     * ASSISTANT SYSTEMS.
     */

    var lastBrakeLoosened = true

    fun resetABS() {
        lastBrakeLoosened = true
    }

    /**
     * A naive ABS.
     */
    fun filterABS(brake: Double): Double {
        if (brake >= 0.5) {
            val result = if (lastBrakeLoosened) {
                brake
            } else {
                0.0
            }

            lastBrakeLoosened = !lastBrakeLoosened

            return result
        }

        return brake
    }

    val constantTurnSpeed = 0.95
    val constantTurnDistanceConsidered = 50.0
    val constantStraightDrivingThreshold = 0.05

    var si = SensorInformation(noise, sensorValues.size)

    fun update(dt: Double) {
        si = updateGameState()
        val input = controller!!.control(si)

        val targetSteer = input.left - input.right


//        val steeringDirection = sign(targetSteer).toInt()
//        if (steeringDirection != lastSteeringDirection) {
//            if (lastSteeringDirection != 0) steerChangesLastSec++
//            lastSteeringDirection = steeringDirection
//        }
//
//        if (currentStep % race.fps == 0) {
//            steerChangesTotal += steerChangesLastSec
//            steerChangesLastSec = 0
//        }


        if (currentSegment != null) {
            val countsToFitness: Boolean

            val possibleSpeed = if (currentSegment!!.inTurn != null) {
                countsToFitness = true
                currentSegment!!.inTurn!!.maxSpeed
            } else {
                val nextTurn = currentSegment!!.nextTurn!!

                if (nextTurn.maxSpeed >= si.maxSpeed) {
                    countsToFitness = false
                    si.maxSpeed
                } else {
                    val distance = min(
                        constantTurnDistanceConsidered, if (nextTurn.segments.first().id < currentSegment!!.id)
                            track.length - (currentSegment!!.totalTrackLength + si.segmentPosition) + nextTurn.segments.first().totalTrackLength
                        else nextTurn.segments.first().totalTrackLength - (currentSegment!!.totalTrackLength + si.segmentPosition)
                    )

                    countsToFitness = distance < constantTurnDistanceConsidered

                    nextTurn.maxSpeed + (si.maxSpeed - nextTurn.maxSpeed) * (distance / constantTurnDistanceConsidered)
                }
            } * constantTurnSpeed

            if (countsToFitness) {
                tooLowTurnSpeed += possibleSpeed - absVel
//                println("${possibleSpeed - absVel}")
                ticksInOrBeforeTurns++
            }
        }

        if (abs(targetSteer) <= constantStraightDrivingThreshold) {
            lengthDrivenStraight += absVel * dt
        }


        throttle = input.throttle
        brake = filterABS(input.brake)
        steerAngle = targetSteer * maxSteer


//        val crtl = eds.control(absVel * 3.6)
//        inputs.throttle = crtl.first
//        inputs.brake = crtl.second
//        steerAngle = (angleToTrackAxis - distanceToTrackAxis * 0.5)
//
//        eds.sample(absVel * 3.6)

        doPhysics(dt)
    }

    var lengthDrivenStraight = 0.0
    var penaltyPoints = 0
    var tooLowTurnSpeed = 0.0
    var ticksInOrBeforeTurns = 0


//    val thresholdSteerChangesPerSecond = 2
//    var steerChangesLastSec = 0
//    var steerChangesTotal = 0
//    var lastSteeringDirection = 0


    private fun defineSensorAngles(from: Double, to: Double, number: Int): Array<Double> {
        val angles = mutableListOf<Double>()

        val total = abs(from) + abs(to)
        val step = total / (number - 1).toDouble()

        var value = from

        angles.add(-value)

        for (i in 1 until number) {
            value += step
            angles.add(-value)

        }

        return angles.toTypedArray()
    }

    private fun defineSensorAngles(from: Int, to: Int, stepSize: Int): Array<Double> {
        val angles = mutableListOf<Double>()

        for (x in from..to step stepSize) {
            angles.add(-x.toDouble() / (180.0) * PI)
        }

        return angles.toTypedArray()
    }

    /**
     * Update the sensor direction vectors.
     */
    private fun updateSensors(): Array<Vector2> {
        return Array(sensorValues.size) { i ->
            var target = heading + sensorValues[i]
            if (target > PI) target -= 2.0 * PI
            if (target < -PI) target += 2.0 * PI

            Vector2.fromAngleInRad(target)
        }
    }

//    private fun extend(values: Array<Double>): Array<Double> {
//        val newSize = values.size + (values.size - 1) * 2
//
//        val result = Array(newSize) { i ->
//            0.0
//        }
//
//        var indexCounter = 0
//
//        for (index in 0 until values.size - 1) {
//            val v1 = values[index]
//            val v2 = (2.0 * values[index] + 1.0 * values[index + 1]) / 3.0
//            val v3 = (1.0 * values[index] + 2.0 * values[index + 1]) / 3.0
//
//            result[indexCounter++] = v1
//            result[indexCounter++] = v2
//            result[indexCounter++] = v3
//        }
//
//        result[indexCounter] = values[values.size - 1]
//
//        return result
//    }


    private fun determineIndicesToConsider(min: Int, max: Int): Array<Int> {
        val indices = Array(-min + max + 1) { 0 }
        var inputIndex = 0
        val segmentIndex = if (currentSegment != null) currentSegment!!.id else 0

        for (i in min..max) {
            val index = segmentIndex + i
            indices[inputIndex++] = when {
                index < 0 -> {
                    track.segments.size + index
                }
                index >= track.segments.size -> {
                    index - track.segments.size
                }
                else -> {
                    index
                }
            }
        }

        return indices
    }


    private fun findCurrentSegment(): Segment? {
        if (wasOffTrack) return null
        indicesToConsider = determineIndicesToConsider(-1, 3)

        for (segmentIndex in indicesToConsider) {
            val segment = track.segments[segmentIndex]
            if (GeometryUtils.isWithin(segment, position)) {
                return segment
            }
        }

        return null
    }


    private fun updateSensorData(): Array<Double> {
        if (wasOffTrack) return Array(0) { 0.0 }

        sensors = updateSensors()

        val data = Array(sensors!!.size) { 1.0 }

        for (index in sensors!!.indices) {
            val sensorLine = LineSegment(position, position.addNew(sensors!![index].multiply(si.sensorRange)))

            var dMin = Double.MAX_VALUE
            var found = false

            var distanceSum = 0.0
            var segmentIndex = currentSegment!!.id

            while (distanceSum <= si.sensorRange) {
                val segment = track.segments[segmentIndex]
                distanceSum += segment.measuredLength

                for (line in segment.segmentLines) {
                    val intersection = sensorLine.intersect(line)

                    if (intersection != null) {
                        found = true

                        val distance = position.distance(intersection)

                        if (distance < dMin) {
                            dMin = distance
                        }
                    }
                }

                if (++segmentIndex >= track.segments.size) segmentIndex = 0
            }

//            for (segmentIndex in 0 until track.segments.size) {
//                val segment = track.segments[segmentIndex]
//
//                for (line in segment.segmentLines) {
//                    val intersection = sensorLine.intersect(line)
//
//                    if (intersection != null) {
//                        found = true
//
//                        val distance = position.distance(intersection)
//
//                        if (distance < dMin) {
//                            dMin = distance
//                        }
//                    }
//                }
//            }

            if (found) data[index] = (dMin / si.sensorRange)
        }

        return data
    }


    fun getFitness(): Array<Double> {
        /*
        Successful,if
        f1 <= 0.2 -- avg speed >= 144km/h
        f2 <= 0.1 -- in 90 % of all measured points, speed lies at 0.95 x max measured turn speed of more
        f3 <= 0.15 -- more than 85 % of the total straight length of the track are driven with only minor steering wheel movements
         */
        val metersPerTickAt180KMH = (180.0 / 3.6) * race.dt
        val distanceAt180KMH = metersPerTickAt180KMH * race.tMax.toDouble()
        val distanceOnStraightsAt180KMH = distanceAt180KMH * (race.track.straightLength / race.track.length)

        val fDistance = 1.0 - min(1.0, distanceRaced / distanceAt180KMH)
        val fCurveSpeed = if (ticksInOrBeforeTurns == 0) 1.0 else
            max(0.0, min(1.0, tooLowTurnSpeed / (ticksInOrBeforeTurns.toDouble() * race.fps.toDouble())))

        val fStraight = 1.0 - min(1.0, lengthDrivenStraight / distanceOnStraightsAt180KMH)



        return arrayOf(fDistance, fCurveSpeed, fStraight)
    }

//    var roundCounter = 0

//    var finishedTwoRounds = false

    fun updateGameState(): SensorInformation {
        si.absVel = absVel

        currentSegment = findCurrentSegment()

        if (currentSegment != null) {
            lastValidSegment = currentSegment

            if (previousSegment != currentSegment) {
                if (previousSegment == null) {
                    // First update. Or after loosing track.
                    visitedSegments = 0
                } else {
                    if (previousSegment!!.id == track.segments[track.segments.size - 1].id && currentSegment!!.id == 0) {
                        // Finished round.
                        si.roundsFinished++

//                        if (si.roundsFinished == 2)
//                        {
//                            finishedTwoRounds = true
//                        }

                        visitedSegments++
                    } else {
                        val segmentDiff = currentSegment!!.id - previousSegment!!.id
                        when {
                            segmentDiff < 0 -> {
                                disqualified = true
                            }
                            segmentDiff <= 3 -> {
                                // Entering subsequent segment.
                                visitedSegments++
                            }
                            else -> {
                                visitedSegments = 0
                            }
                        }
                    }
                }

                previousSegment = currentSegment
            }

            val segment = currentSegment!!

            si.sensorData = updateSensorData()

            // Sensor: Angle to track axis. Ie. angle necessary to turn towards, to follow track axis.
            si.angleToTrackAxis = segment.segmentAngle - heading
            if (si.angleToTrackAxis < -PI) si.angleToTrackAxis += 2.0 * PI
            if (si.angleToTrackAxis > PI) si.angleToTrackAxis -= 2.0 * PI
            si.angleToTrackIdeal = segment.idealAngle - heading
            if (si.angleToTrackIdeal < -PI) si.angleToTrackIdeal += 2.0 * PI
            if (si.angleToTrackIdeal > PI) si.angleToTrackIdeal -= 2.0 * PI

//            println(angleToTrackAxis)

            val detAxis =
                sign((segment.centreEnd.x - segment.centreStart.x) * (position.y - segment.centreStart.y) - (segment.centreEnd.y - segment.centreStart.y) * (position.x - segment.centreStart.x)).toInt()

            val pAxis = (if (detAxis == 0) position else GeometryUtils.adjPoint(position, segment.axis))!!
            // Sensor: Current position concerning track length.
            si.segmentPosition = segment.centreStart.distance(pAxis)
            // Sensor: Deviation from track centre.
            val ratio = si.segmentPosition / segment.measuredLength
            //REMOVE: As widthStart and widthEnd are squared distances, multiply with 1/4. We are interested in half width, so multiply with 1/8
            val widthAtPoint = segment.widthStart * (1.0 - ratio) + segment.widthEnd * ratio

            si.distanceToTrackAxis = if (detAxis != 0) {
                sign(detAxis.toDouble()) * position.distance(pAxis) / (widthAtPoint * 0.5)
            } else {
                0.0
            }

            val detIdeal =
                sign((segment.idealEnd.x - segment.idealStart.x) * (position.y - segment.idealStart.y) - (segment.idealEnd.y - segment.idealStart.y) * (position.x - segment.idealStart.x)).toInt()
            val pIdeal = (if (detIdeal == 0) position else GeometryUtils.adjPoint(position, segment.ideal))!!


            si.distanceToTrackIdeal = if (detIdeal != 0) {
                sign(detIdeal.toDouble()) * position.distance(pIdeal) / (widthAtPoint * 0.5)
            } else {
                0.0
            }

        } else {
            wasOffTrack = true
            disqualified = true
        }

        if (abs(si.distanceToTrackAxis) > 0.9) {
            totalDistanceFromTrack += abs(si.distanceToTrackAxis)
        }

        if (abs(si.distanceToTrackIdeal) > 0.1) {
            totalDistanceToIdeal += abs(si.distanceToTrackIdeal)
        }


        if (!disqualified) {
            totalSpeed += absVel

            if (absVel > speedReachedMax) speedReachedMax = absVel
        }

        currentStep++

        si.finish()

//        if (!isTraining && finishedTwoRounds)
//        {
//            disqualified = true
//        }

        return si
    }

//    private fun getNextTurn(): Double {
//        val threshold = 0.0 //0.2 // ~11 deg
//        val p = getTrackAngleInRad()
//
////        println(p)
//
//        if (abs(p) < threshold) {
//            return 0.5
//        }
//
//        // 0.0 = right, 1.0 = left
//        return if (p < 0.0) 0.0 else 1.0
//    }
//
//    private fun getTrackAngleInRad(distanceMax: Double = 200.0): Double {
//
//        val indexCentre = (sensorData!!.size - 1) / 2
//        val indexRight = indexCentre - 1
//        val indexLeft = indexCentre + 1
//
//        val sensorRight = sensorData!![indexRight]
//        val sensorFront = sensorData!![indexCentre]
//        val sensorLeft = sensorData!![indexLeft]
//
//        // track is straight and enough far from a turn so goes to max speed
//        return if (sensorFront > distanceMax || sensorFront >= sensorRight && sensorFront >= sensorLeft) {
//            0.0
//        } else { // approaching a turn on right
//
//            val B = sensors!![indexCentre]
//            val C = if (sensorLeft > sensorRight) sensors!![indexLeft] else sensors!![indexRight]
//
//            val factor = sign(sensorLeft - sensorRight)
//
//            val b = C.y - B.y
//            val a = C.subtract(B).magn()
//
//            acos(b / a) * factor
//        }
//    }


    /**
     * Car physics.
     */
    private fun doPhysics(dt: Double) {
        val sn = sin(heading)
        val cs = cos(heading)

        // Get velocity in local car coordinates
        velocity_c.x = cs * velocity.x + sn * velocity.y;
        velocity_c.y = cs * velocity.y - sn * velocity.x;

        // Weight on axles based on centre of gravity and weight shift due to forward/reverse acceleration
        val axleWeightFront =
            mass * (axleWeightRatioFront * gravity - weightTransfer * accel_c.x * cgHeight / wheelBase);
        val axleWeightRear = mass * (axleWeightRatioRear * gravity + weightTransfer * accel_c.x * cgHeight / wheelBase);

        // Resulting velocity of the wheels as result of the yaw rate of the car body.
        // v = yawrate * r where r is distance from axle to CG and yawRate (angular velocity) in rad/s.
        val yawSpeedFront = cgToFrontAxle * this.yawRate;
        val yawSpeedRear = -cgToRearAxle * this.yawRate;

        // Calculate slip angles for front and rear wheels (a.k.a. alpha)
        val slipAngleFront = atan2(velocity_c.y + yawSpeedFront, abs(velocity_c.x)) - sign(velocity_c.x) * steerAngle;
        val slipAngleRear = atan2(velocity_c.y + yawSpeedRear, abs(velocity_c.x));

        val tireGripFront = tireGrip;
        val tireGripRear = tireGrip //* (1.0 - inputs.ebrake * (1.0 - lockGrip)); // reduce rear grip when ebrake is on

        val frictionForceFront_cy =
            clamp(-cornerStiffnessFront * slipAngleFront, -tireGripFront, tireGripFront) * axleWeightFront;
        val frictionForceRear_cy =
            clamp(-cornerStiffnessRear * slipAngleRear, -tireGripRear, tireGripRear) * axleWeightRear;

        //  Get amount of brake/throttle from our inputs
        val brake = this.brake * brakeForce
        val throttle = this.throttle * engineForce

/*
ONLY BASIC PHYSICS MODEL. Important to tune those constants:
-throttle: to low = training applies too much throttle
 to high = training applies too less throttle.

 CONSIDER Sallab, Ahmad El, et al. "Meta learning Framework for Automated Driving." arXiv preprint arXiv:1706.04038 (2017).
 */


        //  Resulting force in local car coordinates.
        //  This is implemented as a RWD car only.
        val tractionForce_cx = throttle - brake * sign(velocity_c.x)
        val tractionForce_cy = 0.0;

        val dragForce_cx = -rollResist * velocity_c.x - airResist * velocity_c.x * abs(velocity_c.x)
        val dragForce_cy = -rollResist * velocity_c.y - airResist * velocity_c.y * abs(velocity_c.y)

        // total force in car coordinates
        val totalForce_cx = dragForce_cx + tractionForce_cx;
        val totalForce_cy =
            dragForce_cy + tractionForce_cy + cos(steerAngle) * frictionForceFront_cy + frictionForceRear_cy;

        // acceleration along car axes
        accel_c.x = totalForce_cx / mass;  // forward/reverse accel
        accel_c.y = totalForce_cy / mass;  // sideways accel

        // acceleration in world coordinates
        accel.x = cs * accel_c.x - sn * accel_c.y;
        accel.y = sn * accel_c.x + cs * accel_c.y;

        // update velocity
        velocity.x += accel.x * dt;
        velocity.y += accel.y * dt;

        absVel = velocity.magn()


        // calculate rotational forces
        var angularTorque =
            (frictionForceFront_cy + tractionForce_cy) * cgToFrontAxle - frictionForceRear_cy * cgToRearAxle;

        //  Sim gets unstable at very slow speeds, so just stop the car
        if (abs(absVel) < 0.5 && throttle == 0.0) {
            velocity.x = 0.0
            velocity.y = 0.0
            absVel = 0.0
            angularTorque = 0.0
            yawRate = 0.0
        }

        val angularAccel = angularTorque / this.inertia

        yawRate += angularAccel * dt;
        heading += yawRate * dt;

        //  finally we can update position
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;

        totalVel += absVel * dt
    }

    var totalVel = 0.0

    private fun clamp(n: Double, vMin: Double, vMax: Double): Double {
        return min(vMax, max(vMin, n))
    }

}