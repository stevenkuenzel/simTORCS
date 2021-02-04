package simtorcs.car

import simtorcs.car.control.CarController
import simtorcs.geometry.LineSegment
import simtorcs.geometry.Vector2
import simtorcs.race.Race
import simtorcs.track.Segment
import simtorcs.util.GeometryUtils
import kotlin.math.*

class Car(private val race: Race, noisySensors: Boolean) {

    companion object {
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
    }

    private var controller: CarController? = null

    val track = race.track

    var sensors: Array<Vector2>? = null
    var indicesToConsider = Array(0) { 0 }
    var previousSegment: Segment? = null
    var wasOffTrack = false


    var visitedSegments = 0
    var distanceRaced = 0.0
    var totalDistanceToIdeal = 0.0
    var speedReachedMax = 0.0


    val maxSteer = 0.366519  // Maximum steering angle in radians

    val sensorAngles = defineSensorAngles(-45, 45, 5)

    var currentSegment: Segment? = null
    var lastValidSegment: Segment? = null

    var currentStep = 0
    var totalDistanceFromTrack = 0.0
    var totalSpeed = 0.0

    var disqualified = false

    var heading = track.startingDirection
    var position = track.startingPoint.copy()


    var velocity = Vector2()
    var velocity_c = Vector2()
    var accel = Vector2()
    var accel_c = Vector2()
    var absVel = 0.0
    var yawRate = 0.0

    var throttle = 0.0
    var brake = 0.0
    var steerAngle = 0.0


    // Car configuration.
    val gravity = 9.81  // m/s^2
    val mass = 1150.0  // kg
    val inertiaScale = 1.0  // Multiply by mass for inertia
    val halfWidth = 0.97 // Centre to side of chassis (metres)
    val cgToFront = 2.3504 // Centre of gravity to front of chassis (metres)
    val cgToRear = 2.1696   // Centre of gravity to rear of chassis
    val cgToFrontAxle = 0.97  // Centre gravity to front axle
    val cgToRearAxle = 0.97  // Centre gravity to rear axle
    val cgHeight = 0.25  // Centre gravity height

    val tireGrip = 2.0  // How much grip tires have

    // ANNRACER:
    // acc: 8.579664582906522 * x + 72.34940739001144
    // dec: -90.13773558596135 * x + 191.35322287960696

    val engineForce = 11900.0
    val brakeForce = 26300.0

    val weightTransfer = 0.2  // How much weight is transferred during acceleration/braking
    val cornerStiffnessFront = 5.0
    val cornerStiffnessRear = 5.2
    val airResist = 2.5    // air resistance (* vel)
    val rollResist = 8.0   // rolling resistance force (* vel)

    val inertia = mass * inertiaScale
    val wheelBase = cgToFrontAxle + cgToRearAxle
    val axleWeightRatioFront = cgToRearAxle / this.wheelBase // % car weight on the front axle
    val axleWeightRatioRear = cgToFrontAxle / this.wheelBase // % car weight on the rear axle


    val constantTurnSpeed = 0.95
    val constantTurnDistanceConsidered = 50.0
    val constantStraightDrivingThreshold = 0.05

    var sensorInformation = SensorInformation(noisySensors, sensorAngles.size)


    fun setController(controller: CarController) {
        this.controller = controller
    }

    fun raceEnd() {
        distanceRaced = sensorInformation.roundsFinished.toDouble() * race.track.length

        if (currentSegment != null) {
            distanceRaced += lastValidSegment!!.totalTrackLength + sensorInformation.segmentPosition
        } else if (lastValidSegment != null) {
            distanceRaced += lastValidSegment!!.totalTrackLength
        }
    }


    fun update(dt: Double) {
        sensorInformation = updateGameState()

        val input = controller!!.control(sensorInformation)
        val targetSteer = input.left - input.right

        if (currentSegment != null) {
            val countsToFitness: Boolean

            val possibleSpeed = if (currentSegment!!.inTurn != null) {
                countsToFitness = true
                currentSegment!!.inTurn!!.maxSpeed
            } else {
                val nextTurn = currentSegment!!.nextTurn!!

                if (nextTurn.maxSpeed >= sensorInformation.maxSpeed) {
                    countsToFitness = false
                    sensorInformation.maxSpeed
                } else {
                    val distance = min(
                        constantTurnDistanceConsidered, if (nextTurn.segments.first().id < currentSegment!!.id)
                            track.length - (currentSegment!!.totalTrackLength + sensorInformation.segmentPosition) + nextTurn.segments.first().totalTrackLength
                        else nextTurn.segments.first().totalTrackLength - (currentSegment!!.totalTrackLength + sensorInformation.segmentPosition)
                    )

                    countsToFitness = distance < constantTurnDistanceConsidered

                    nextTurn.maxSpeed + (sensorInformation.maxSpeed - nextTurn.maxSpeed) * (distance / constantTurnDistanceConsidered)
                }
            } * constantTurnSpeed

            if (countsToFitness) {
                tooLowTurnSpeed += possibleSpeed - absVel
                ticksInOrBeforeTurns++
            }
        }

        if (abs(targetSteer) <= constantStraightDrivingThreshold) {
            lengthDrivenStraight += absVel * dt
        }

        throttle = input.throttle
        brake = filterABS(input.brake)
        steerAngle = targetSteer * maxSteer

        doPhysics(dt)
    }

    var lengthDrivenStraight = 0.0
    var tooLowTurnSpeed = 0.0
    var ticksInOrBeforeTurns = 0


    /**
     * Determines the track segment, the car is currently located on.
     */
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

    /**
     * Finds the indices of the track segments to consider for determining the current segment.
     */
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


    /**
     * Updates the sensor information: Distance of the sensors to the track edges.
     */
    private fun updateTrackEdgeSensors(): Array<Double> {
        // The car is disqualified.
        if (wasOffTrack) return Array(0) { 0.0 }

        sensors = updateSensorTargetVectors()

        val data = Array(sensors!!.size) { 1.0 }

        for (index in sensors!!.indices) {
            val sensorLine =
                LineSegment(position, position.addNew(sensors!![index].multiply(sensorInformation.sensorRange)))

            var dMin = Double.MAX_VALUE
            var found = false

            var totalDistanceToSegment = 0.0
            var segmentIndex = currentSegment!!.id

            // Iterate over the next segments, break if the max. sensor range is exceeded
            while (totalDistanceToSegment <= sensorInformation.sensorRange) {
                val segment = track.segments[segmentIndex]
                totalDistanceToSegment += segment.measuredLength

                // Check all segment lines for intersections and return the minimum value.
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

            // Replace the sensor value.
            if (found) data[index] = (dMin / sensorInformation.sensorRange)
        }

        return data
    }


    /**
     * Update the sensor direction vectors.
     */
    private fun updateSensorTargetVectors(): Array<Vector2> {
        return Array(sensorAngles.size) { i ->
            var target = heading + sensorAngles[i]
            if (target > PI) target -= 2.0 * PI
            if (target < -PI) target += 2.0 * PI

            Vector2.fromAngleInRad(target)
        }
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


    fun updateGameState(): SensorInformation {
        sensorInformation.absVel = absVel

        currentSegment = findCurrentSegment()

        if (currentSegment != null) {
            lastValidSegment = currentSegment

            // Count the number of visited segments.
            if (previousSegment != currentSegment) {
                if (previousSegment == null) {
                    // First update. Or after loosing track.
                    visitedSegments = 0
                } else {
                    if (previousSegment!!.id == track.segments[track.segments.size - 1].id && currentSegment!!.id == 0) {
                        // Finished round.
                        sensorInformation.roundsFinished++

                        visitedSegments++
                    } else {
                        val segmentDiff = currentSegment!!.id - previousSegment!!.id
                        when {
                            segmentDiff < 0 -> {
                                // The car is driving backwards. Disqualify it.
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

            // Update Sensors: Track Edge.
            sensorInformation.sensorData = updateTrackEdgeSensors()

            val segment = currentSegment!!

            // Update Sensors: Angle to track axis, i.e. angle necessary to turn towards, to follow track axis.
            sensorInformation.angleToTrackAxis = segment.segmentAngle - heading
            if (sensorInformation.angleToTrackAxis < -PI) sensorInformation.angleToTrackAxis += 2.0 * PI
            if (sensorInformation.angleToTrackAxis > PI) sensorInformation.angleToTrackAxis -= 2.0 * PI

            // Update Sensors: Angle to segment ideal line. Attention: Has to be determined in advance.
            sensorInformation.angleToTrackIdeal = segment.idealAngle - heading
            if (sensorInformation.angleToTrackIdeal < -PI) sensorInformation.angleToTrackIdeal += 2.0 * PI
            if (sensorInformation.angleToTrackIdeal > PI) sensorInformation.angleToTrackIdeal -= 2.0 * PI


            val detAxis = sign(
                (segment.centreEnd.x - segment.centreStart.x) * (position.y - segment.centreStart.y) -
                        (segment.centreEnd.y - segment.centreStart.y) * (position.x - segment.centreStart.x)
            ).toInt()

            val pAxis = (if (detAxis == 0) position else GeometryUtils.adjPoint(position, segment.axis))!!
            // Update Sensors: Current position concerning track length.
            sensorInformation.segmentPosition = segment.centreStart.distance(pAxis)

            // Determine the relative position on the track and its width at that position.
            val ratio = sensorInformation.segmentPosition / segment.measuredLength
            val widthAtPoint = segment.widthStart * (1.0 - ratio) + segment.widthEnd * ratio

            // Update Sensors: Distance to the center axis of the track.
            sensorInformation.distanceToTrackAxis = if (detAxis != 0) {
                sign(detAxis.toDouble()) * position.distance(pAxis) / (widthAtPoint * 0.5)
            } else {
                0.0
            }

            // Same for ideal line.
            val detIdeal =
                sign((segment.idealEnd.x - segment.idealStart.x) * (position.y - segment.idealStart.y) - (segment.idealEnd.y - segment.idealStart.y) * (position.x - segment.idealStart.x)).toInt()
            val pIdeal = (if (detIdeal == 0) position else GeometryUtils.adjPoint(position, segment.ideal))!!


            sensorInformation.distanceToTrackIdeal = if (detIdeal != 0) {
                sign(detIdeal.toDouble()) * position.distance(pIdeal) / (widthAtPoint * 0.5)
            } else {
                0.0
            }

        } else {
            // The car has left the track. Disqualify it.
            wasOffTrack = true
            disqualified = true
        }

        if (abs(sensorInformation.distanceToTrackAxis) > 0.9) {
            totalDistanceFromTrack += abs(sensorInformation.distanceToTrackAxis)
        }

        if (abs(sensorInformation.distanceToTrackIdeal) > 0.1) {
            totalDistanceToIdeal += abs(sensorInformation.distanceToTrackIdeal)
        }


        if (!disqualified) {
            totalSpeed += absVel

            if (absVel > speedReachedMax) speedReachedMax = absVel
        }

        currentStep++

        sensorInformation.finish()

        return sensorInformation
    }


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


    /**
     * A naive approach on Anti-lock braking system.
     */

    var lastBrakeLoosened = true

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

        if (brake < 0.1) {
            // Reset.
            lastBrakeLoosened = true
        }

        return brake
    }
}