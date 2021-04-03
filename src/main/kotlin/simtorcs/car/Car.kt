package simtorcs.car

import simtorcs.car.control.CarController
import simtorcs.geometry.LineSegment
import simtorcs.geometry.Vector2
import simtorcs.race.Race
import simtorcs.track.Segment
import simtorcs.geometry.GeometryUtils
import kotlin.math.*

class Car(val race: Race, noisySensors: Boolean) {

    companion object {
        val RANGE_TRACK_EDGE_SENSOR_LEFT = -45
        val RANGE_TRACK_EDGE_SENSOR_RIGHT = 45
        val ANGLE_BETWEEN_TRACK_EDGE_SENSORS = 5

        private fun defineSensorAngles(from: Int, to: Int, stepSize: Int): Array<Double> {
            val angles = mutableListOf<Double>()

            for (x in from..to step stepSize) {
                angles.add((-x.toDouble() / 180.0) * PI)
            }

            return angles.toTypedArray()
        }

        /**
         * Maximum steering angle in radians
         */
        private val STEER_MAX = 0.366519


        /**
         * PHYSICS RELATED CONSTANTS.
         */

        // Car configuration. Based on the properties of car1-trb1 in TORCS.
        private val PHYSICS_GRAVITY = 9.81  // m/s^2
        private val PHYSICS_MASS = 1150.0  // kg
        private val PHYSICS_INERTIA_SCALE = 1.0  // Multiply by mass for inertia
        private val PHYSICS_CG_TO_FRONT_AXLE = 0.97  // Centre gravity to front axle
        private val PHYSICS_CG_TO_REAR_AXLE = 0.97  // Centre gravity to rear axle
        private val PHYSICS_CG_HEIGHT = 0.25  // Centre gravity height
        private val PHYSICS_TIRE_GRIP = 2.0  // How much grip tires have
        private val PHYSICS_WEIGHT_TRANSFER = 0.2  // How much weight is transferred during acceleration/braking
        private val PHYSICS_CORNER_STIFFNESS_FRONT = 5.0
        private val PHYSICS_CORNER_STIFFNESS_REAR = 5.2
        private val PHYSICS_AIR_RESIST = 2.5    // air resistance (* vel)
        private val PHYSICS_ROLL_RESIST = 8.0   // rolling resistance force (* vel)
        private val PHYSICS_INERTIA = PHYSICS_MASS * PHYSICS_INERTIA_SCALE
        private val PHYSICS_WHEEL_BASE = PHYSICS_CG_TO_FRONT_AXLE + PHYSICS_CG_TO_REAR_AXLE
        private val PHYSICS_AXLE_WEIGHT_RATIO_FRONT =
            PHYSICS_CG_TO_REAR_AXLE / this.PHYSICS_WHEEL_BASE // % car weight on the front axle
        private val PHYSICS_AXLE_WEIGHT_RATIO_REAR =
            PHYSICS_CG_TO_FRONT_AXLE / this.PHYSICS_WHEEL_BASE // % car weight on the rear axle

        // Constants determined according to the description in my dissertation.
        private val PHYSICS_ENGINE_FORCE = 11900.0
        private val PHYSICS_BRAKE_FORCE = 26300.0

    }

    private var controller: CarController? = null
    val track = race.track

    var totalDistanceFromTrack = 0.0
    var totalSpeed = 0.0

    var disqualified = false

    var sensors: Array<Vector2>? = null
    private val sensorAngles = defineSensorAngles(
        RANGE_TRACK_EDGE_SENSOR_LEFT,
        RANGE_TRACK_EDGE_SENSOR_RIGHT,
        ANGLE_BETWEEN_TRACK_EDGE_SENSORS
    )
    var sensorInformation = SensorInformation(noisySensors, race.track, sensorAngles.size)
    var heading = track.startingDirection
    var position = track.startingPoint.copy()
    var currentSegment: Segment? = null
    var lastValidSegment: Segment? = null
    var previousSegment: Segment? = null

    // Input / Control.
    var throttle = 0.0
    var brake = 0.0
    var steerAngle = 0.0

    // Fitness.
    var distanceRaced = 0.0
    var speedReachedMax = 0.0

    // Physics.
    var velocity = Vector2()
    var velocityLocal = Vector2()
    var acceleration = Vector2()
    var accelerationLocal = Vector2()
    var absoluteVelocity = 0.0
    var yawRate = 0.0

    /**
     * ABS flag.
     */
    var lastBrakeLoosened = true

    fun setController(controller: CarController) {
        this.controller = controller
    }

    private fun updateFitnessRelatedInformation(dt: Double, targetSteer: Double) {
        // Fitness: Turn speed score.
        if (currentSegment != null) {
            val carIsInTurnOrApproaching: Boolean

            val possibleSpeed = if (currentSegment!!.inTurn != null) {
                // The car is currently in a turn.

                carIsInTurnOrApproaching = true
                currentSegment!!.inTurn!!.maxSpeed
            } else {
                val nextTurn = currentSegment!!.nextTurn!!

                if (nextTurn.maxSpeed >= Race.SPEED_MAX) {
                    // The car is on a straight or the next turn is close to straight.
                    carIsInTurnOrApproaching = false
                    Race.SPEED_MAX
                } else {
                    // The next turn is a real "turn". Determine the distance to it.
                    val distance = min(
                        Race.TURN_SPEED_DISTANCE_MAX, if (nextTurn.segments.first().id < currentSegment!!.id)
                            track.length - (currentSegment!!.totalTrackLength + sensorInformation.segmentPosition) + nextTurn.segments.first().totalTrackLength
                        else nextTurn.segments.first().totalTrackLength - (currentSegment!!.totalTrackLength + sensorInformation.segmentPosition)
                    )

                    carIsInTurnOrApproaching = distance < Race.TURN_SPEED_DISTANCE_MAX

                    // Reduce the approx. max. speed linearly in relation to the distance to the turn.
                    nextTurn.maxSpeed + (Race.SPEED_MAX - nextTurn.maxSpeed) * (distance / Race.TURN_SPEED_DISTANCE_MAX)
                }
            } * Race.TURN_SPEED_MULTIPLIER

            if (carIsInTurnOrApproaching) {
                // Update the fitness value. Note: A speed higher than possibleSpeed contributes positively to the fitness value.
                sensorInformation.tooLowTurnSpeed += possibleSpeed - absoluteVelocity
                sensorInformation.ticksInOrBeforeTurns++
            }
        }

        if (abs(targetSteer) <= Race.THRESHOLD_DRIVING_STRAIGHT) {
            sensorInformation.lengthDrivenStraight += absoluteVelocity * dt
        }
    }


    fun update(dt: Double) {
        sensorInformation = updateGameState()

        if (disqualified) return

        val input = controller!!.control(sensorInformation)
        val targetSteer = input.left - input.right

        updateFitnessRelatedInformation(dt, targetSteer)

        throttle = input.throttle
        brake = filterABS(input.brake)
        steerAngle = targetSteer * STEER_MAX

        updatePhysics(dt)
    }


    /**
     * Determines the track segment, the car is currently located on.
     */
    private fun findCurrentSegment(): Segment? {
        if (disqualified) return null
        val segmentsToConsider = determineIndicesToConsider(-1, 3)

        for (segmentIndex in segmentsToConsider) {
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
        if (disqualified) return Array(0) { 0.0 }

        sensors = updateSensorTargetVectors()

        val data = Array(sensors!!.size) { 1.0 }

        for (index in sensors!!.indices) {
            val sensorLine =
                LineSegment(position, position.add(sensors!![index].scale(Race.SENSOR_RANGE)))

            var dMin = Double.MAX_VALUE
            var found = false

            var totalDistanceToSegment = 0.0
            var segmentIndex = currentSegment!!.id

            // Iterate over the next segments, break if the max. sensor range is exceeded
            while (totalDistanceToSegment <= Race.SENSOR_RANGE) {
                val segment = track.segments[segmentIndex]
                totalDistanceToSegment += segment.measuredLength

                // Check all segment lines for intersections and return the minimum value.
                for (line in segment.segmentLines) {
                    val intersection = sensorLine.intersect(line)

                    if (intersection != null) {
                        found = true

                        val distance = position.distanceTo(intersection)

                        if (distance < dMin) {
                            dMin = distance
                        }
                    }
                }

                if (++segmentIndex >= track.segments.size) segmentIndex = 0
            }

            // Replace the sensor value.
            if (found) data[index] = (dMin / Race.SENSOR_RANGE)
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

    private fun updateGameState(): SensorInformation {
        sensorInformation.absoluteVelocity = absoluteVelocity

        currentSegment = findCurrentSegment()

        if (currentSegment != null) {
            lastValidSegment = currentSegment
            sensorInformation.lapPosition = lastValidSegment!!.totalTrackLength

            // Count the number of visited segments.
            if (previousSegment != currentSegment) {
                if (previousSegment == null) {
                    // First update. Or after loosing track.
                } else {
                    if (previousSegment!!.id == track.segments[track.segments.size - 1].id && currentSegment!!.id == 0) {
                        // Finished round.
                        sensorInformation.roundsFinished++
                    } else {
                        val segmentDiff = currentSegment!!.id - previousSegment!!.id

                        if (segmentDiff < 0) disqualified = true // The car is driving backwards. Disqualify it.
                    }
                }

                previousSegment = currentSegment
            }

            // Update Sensors: Track Edge.
            sensorInformation.trackEdgeSensors = updateTrackEdgeSensors()

            val segment = currentSegment!!

            // Update Sensors: Angle to track axis, i.e. angle necessary to turn towards, to follow track axis.
            sensorInformation.angleToTrackAxis = segment.segmentAngle - heading
            if (sensorInformation.angleToTrackAxis < -PI) sensorInformation.angleToTrackAxis += 2.0 * PI
            if (sensorInformation.angleToTrackAxis > PI) sensorInformation.angleToTrackAxis -= 2.0 * PI

            // Update Sensors: Current position concerning track length.
            val detAxis = sign(
                (segment.centreEnd.x - segment.centreStart.x) * (position.y - segment.centreStart.y) -
                        (segment.centreEnd.y - segment.centreStart.y) * (position.x - segment.centreStart.x)
            ).toInt()
            val pAxis = (if (detAxis == 0) position else GeometryUtils.adjPoint(position, segment.axis))
            sensorInformation.segmentPosition = segment.centreStart.distanceTo(pAxis)

            // Determine the relative position on the track and its width at that position.
            val ratio = sensorInformation.segmentPosition / segment.measuredLength
            val widthAtPoint = segment.widthStart * (1.0 - ratio) + segment.widthEnd * ratio

            // Update Sensors: Distance to the center axis of the track.
            sensorInformation.distanceToTrackAxis = if (detAxis != 0) {
                sign(detAxis.toDouble()) * position.distanceTo(pAxis) / (widthAtPoint * 0.5)
            } else {
                0.0
            }
        } else {
            // The car has left the track. Disqualify it.
            disqualified = true
        }

        if (abs(sensorInformation.distanceToTrackAxis) > 0.9) {
            totalDistanceFromTrack += abs(sensorInformation.distanceToTrackAxis)
        }

        if (!disqualified) {
            totalSpeed += absoluteVelocity

            if (absoluteVelocity > speedReachedMax) speedReachedMax = absoluteVelocity
        }

        sensorInformation.perturbIfNecessary()

        return sensorInformation
    }

    /**
     * Car physics.
     */
    private fun updatePhysics(dt: Double) {
        val sn = sin(heading)
        val cs = cos(heading)

        // Get velocity in local car coordinates
        velocityLocal.x = cs * velocity.x + sn * velocity.y;
        velocityLocal.y = cs * velocity.y - sn * velocity.x;

        // Weight on axles based on centre of gravity and weight shift due to forward/reverse acceleration
        val axleWeightFront =
            PHYSICS_MASS * (PHYSICS_AXLE_WEIGHT_RATIO_FRONT * PHYSICS_GRAVITY - PHYSICS_WEIGHT_TRANSFER * accelerationLocal.x * PHYSICS_CG_HEIGHT / PHYSICS_WHEEL_BASE);
        val axleWeightRear =
            PHYSICS_MASS * (PHYSICS_AXLE_WEIGHT_RATIO_REAR * PHYSICS_GRAVITY + PHYSICS_WEIGHT_TRANSFER * accelerationLocal.x * PHYSICS_CG_HEIGHT / PHYSICS_WHEEL_BASE);

        // Resulting velocity of the wheels as result of the yaw rate of the car body.
        // v = yawrate * r where r is distance from axle to CG and yawRate (angular velocity) in rad/s.
        val yawSpeedFront = PHYSICS_CG_TO_FRONT_AXLE * this.yawRate;
        val yawSpeedRear = -PHYSICS_CG_TO_REAR_AXLE * this.yawRate;

        // Calculate slip angles for front and rear wheels (a.k.a. alpha)
        val slipAngleFront =
            atan2(velocityLocal.y + yawSpeedFront, abs(velocityLocal.x)) - sign(velocityLocal.x) * steerAngle;
        val slipAngleRear = atan2(velocityLocal.y + yawSpeedRear, abs(velocityLocal.x));

        val tireGripFront = PHYSICS_TIRE_GRIP;
        val tireGripRear =
            PHYSICS_TIRE_GRIP //* (1.0 - inputs.ebrake * (1.0 - lockGrip)); // reduce rear grip when ebrake is on

        val frictionForceFront_cy =
            clamp(-PHYSICS_CORNER_STIFFNESS_FRONT * slipAngleFront, -tireGripFront, tireGripFront) * axleWeightFront;
        val frictionForceRear_cy =
            clamp(-PHYSICS_CORNER_STIFFNESS_REAR * slipAngleRear, -tireGripRear, tireGripRear) * axleWeightRear;

        //  Get amount of brake/throttle from our inputs
        val brake = this.brake * PHYSICS_BRAKE_FORCE
        val throttle = this.throttle * PHYSICS_ENGINE_FORCE

        /*
        ONLY BASIC PHYSICS MODEL. Important to tune those constants:
        -throttle:
         --to low = training applies too much throttle,
         --to high = training applies too less throttle.

         CONSIDER Sallab, Ahmad El, et al. "Meta learning Framework for Automated Driving." arXiv preprint arXiv:1706.04038 (2017).
         */

        //  Resulting force in local car coordinates.
        //  This is implemented as a RWD car only.
        val tractionForce_cx = throttle - brake * sign(velocityLocal.x)
        val tractionForce_cy = 0.0;

        val dragForce_cx =
            -PHYSICS_ROLL_RESIST * velocityLocal.x - PHYSICS_AIR_RESIST * velocityLocal.x * abs(velocityLocal.x)
        val dragForce_cy =
            -PHYSICS_ROLL_RESIST * velocityLocal.y - PHYSICS_AIR_RESIST * velocityLocal.y * abs(velocityLocal.y)

        // total force in car coordinates
        val totalForce_cx = dragForce_cx + tractionForce_cx;
        val totalForce_cy =
            dragForce_cy + tractionForce_cy + cos(steerAngle) * frictionForceFront_cy + frictionForceRear_cy;

        // acceleration along car axes
        accelerationLocal.x = totalForce_cx / PHYSICS_MASS;  // forward/reverse accel
        accelerationLocal.y = totalForce_cy / PHYSICS_MASS;  // sideways accel

        // acceleration in world coordinates
        acceleration.x = cs * accelerationLocal.x - sn * accelerationLocal.y;
        acceleration.y = sn * accelerationLocal.x + cs * accelerationLocal.y;

        // update velocity
        velocity.x += acceleration.x * dt;
        velocity.y += acceleration.y * dt;

        absoluteVelocity = velocity.magnitude()


        // calculate rotational forces
        var angularTorque =
            (frictionForceFront_cy + tractionForce_cy) * PHYSICS_CG_TO_FRONT_AXLE - frictionForceRear_cy * PHYSICS_CG_TO_REAR_AXLE;

        //  Sim gets unstable at very slow speeds, so just stop the car
        if (abs(absoluteVelocity) < 0.5 && throttle == 0.0) {
            velocity.x = 0.0
            velocity.y = 0.0
            absoluteVelocity = 0.0
            angularTorque = 0.0
            yawRate = 0.0
        }

        val angularAccel = angularTorque / PHYSICS_INERTIA

        yawRate += angularAccel * dt;
        heading += yawRate * dt;

        //  finally we can update position
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;

        totalVel += absoluteVelocity * dt
    }

    var totalVel = 0.0

    private fun clamp(n: Double, vMin: Double, vMax: Double): Double {
        return min(vMax, max(vMin, n))
    }

    /**
     * A naive approach on an Anti-lock braking system.
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

        if (brake < 0.1) {
            // Reset.
            lastBrakeLoosened = true
        }

        return brake
    }
}