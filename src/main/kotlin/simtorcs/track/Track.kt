package simtorcs.track

import de.stevenkuenzel.util.PathUtil
import de.stevenkuenzel.xml.XElement
import simtorcs.car.control.TestController
import simtorcs.race.Race
import simtorcs.geometry.Vector2
import kotlin.math.PI
import kotlin.math.abs

class Track(val name: String = "", private val levelOfDetail: Int = 1) {

    companion object {
        /**
         * Imports the TORCS track with the given name and level of detail.
         */
        fun load(name: String, levelOfDetail: Int): Track {
            val invert = name.startsWith("!")
            val t = Track(name, levelOfDetail)

            t.importFromTORCS((if (invert) name.substring(1) else name).toLowerCase(), invert)
            t.finalize()

            return t
        }

        const val MIN_TURN_SPEED = 5.0
        const val MAX_TURN_SEGMENT_LENGTH_SQR = 2500.0
    }

    val minTurnRad = 0.1 / levelOfDetail.toDouble()
    var startWidth = 13.0

    val segments = mutableListOf<Segment>()
    var prevSegment: CoordinateSegment? = null
    var nextSegmentID = 0

    // The location where a new racing car is spawned, including its heading.
    var startingPoint = Vector2()
    var startingDirection = 0.0

    val turns = mutableListOf<Turn>()

    /**
     * Total length of the track.
     */
    var length = 0.0

    /**
     * Length of the straight segments of the track.
     */
    var straightLength = 0.0


    // Values concerning drawing the track on the screen.
    var xMin = Double.MAX_VALUE
    var xMax = Double.NEGATIVE_INFINITY
    var yMin = Double.MAX_VALUE
    var yMax = Double.NEGATIVE_INFINITY


    /**
     * Adds a turn to the track, composed of two half turns with radii radiusStart and radiusEnd.
     */
    fun addTurn(angleInRad: Double, radiusStart: Double, radiusEnd: Double) {
        val angHalf = angleInRad / 2.0
        val segmentLength1 = radiusStart * abs(angHalf)
        val segmentLength2 = radiusEnd * abs(angHalf)

        // First half.
        for (i in 0 until levelOfDetail) {
            addSegment(segmentLength1 / levelOfDetail.toDouble(), startWidth, angHalf / levelOfDetail.toDouble())
        }

        // Second half.
        for (i in 0 until levelOfDetail) {
            addSegment(segmentLength2 / levelOfDetail.toDouble(), startWidth, angHalf / levelOfDetail.toDouble())
        }
    }

    /**
     * Adds a straight segment with a certain rotation w.r.t. the previous segment. The interface between the two segments is covered by an EdgeSegment.
     */
    fun addSegment(length: Double, width: Double, angle: Double) {
        val newSegment = CoordinateSegment(length, width, angle, prevSegment, this)

        val edgeSegment = newSegment.initialize()

        if (edgeSegment != null) {
            edgeSegment.id = nextSegmentID++
            determineMinAndMaxXYValues(edgeSegment)
            segments.add(edgeSegment)
        }

        newSegment.id = nextSegmentID++

        determineMinAndMaxXYValues(newSegment)
        segments.add(newSegment)

        prevSegment = newSegment
    }

    /**
     * Determines the global min. and max. x- and y-values of the track. Required for drawing the track on the screen.
     */
    private fun determineMinAndMaxXYValues(segment: Segment) {
        val segmentMinMax = segment.getXYMinMax()

        val min = segmentMinMax.first
        val max = segmentMinMax.second

        if (min.x < xMin) xMin = min.x
        if (min.y < yMin) yMin = min.y
        if (max.x > xMax) xMax = max.x
        if (max.y > yMax) yMax = max.y
    }


    /**
     * Creates the last track segment, i.e. a connection between the last and the first segment. Determines relevant meta information about the track.
     */
    fun finalize() {
        // Add the connecting segment.
        val firstSegment = segments[0]
        val lastSegment = segments[segments.size - 1]

        val connectingSegment = ConnectingSegment(lastSegment, firstSegment, this)
        connectingSegment.id = nextSegmentID++

        segments.add(connectingSegment)

        // Determine further track information.
        startingPoint = Segment.getWeightedCentre(firstSegment.centreStart, firstSegment.centreEnd, 0.9)
        startingDirection = firstSegment.segmentAngle

        findTurnSegments()
        findMaxTurnSpeeds()

        for (segment in segments) {
            segment.totalTrackLength = length

            length += segment.measuredLength

            if (segment.inTurn == null) straightLength += segment.measuredLength
        }
    }

    /**
     * Determines the track segments that belong to turns.
     */
    private fun findTurnSegments() {
        var currentTurn: Turn? = null

        var nextTurnID = 0

        for (segment in segments) {
            // Do only consider "ordinary" segments:
            if (segment is CoordinateSegment) {
                // A segment is only considered as a turn, if its arc exceeds a certain threshold (depends on level of detail) and its length is shorter than MAX_TURN_SEGMENT_LENGTH_SQR.
                if (segment.turn != TurnDirection.None && abs(segment.turnAngle) >= minTurnRad && segment.measuredLength <= MAX_TURN_SEGMENT_LENGTH_SQR) {
                    when {
                        currentTurn == null -> {
                            // First segment of a new turn.
                            currentTurn = Turn(nextTurnID++, segment)
                        }

                        currentTurn.direction != segment.turn -> {
                            // The turn direction of the segment is different to the direction of the current turn. Consider the segment as the starting segment of a new turn.
                            turns.add(currentTurn.apply { finalize() })
                            currentTurn = Turn(nextTurnID++, segment)
                        }

                        else -> {
                            // Add the segment to the current turn.
                            currentTurn.add(segment)
                        }
                    }
                } else if (currentTurn != null) {
                    // End the current turn.
                    turns.add(currentTurn.apply { finalize() })
                    currentTurn = null
                }
            } else if (segment is EdgeSegment && currentTurn != null) {
                // Add a connecting edge segment to the current turn, if existing.
                currentTurn.add(segment)
            }
        }

        // Finalize the last turn found.
        if (currentTurn != null) turns.add(currentTurn.apply { finalize() })


        // Assign a reference to the next turn to each segment if the track.
        for (index in segments.indices) {
            val segment = segments[index]

            if (segment.inTurn != null) continue


            for (j_ in index + 1 until (segments.size + index + 1)) {
                val j = if (j_ < segments.size) j_ else j_ % segments.size

                if (segments[j].inTurn != null) {
                    segment.nextTurn = segments[j].inTurn
                    break
                }
            }

        }
    }


    /**
     * Determines the maximum driving speed a turn can be taken with.
     */
    private fun findMaxTurnSpeeds() {
        val iniSpeed = 92.0 // [m/s]
        var nextTurnToFind = 0

        var turn = turns[nextTurnToFind]
        var turnPassed: Boolean
        var speed = iniSpeed

        while (true) {
            // Create a new race.
            val race = Race(this, false)
            val car = race.createCar()
            car.setController(TestController(speed))

            // Find the first turn of the track.
            val firstSegmentID = turn.segments.first().id

            // Place the car on the previous segment of the turn segment.
            val prevSegment = segments[if (firstSegmentID > 0) firstSegmentID - 1 else segments.size - 1]
            car.currentSegment = prevSegment
            car.position =
                prevSegment.centreStart.add(prevSegment.centreEnd.subtract(prevSegment.centreStart).scale(0.1))

            // Set the car's heading in segment axis direction.
            car.heading = prevSegment.segmentAngle

            // "Accelerate" the car into axis direction.
            val headingVector = Vector2.fromAngleInRad(car.heading)
            car.velocity = Vector2(headingVector.x * speed, headingVector.y * speed)

            // Update the race until the car is disqualified or has passed the turn.
            while (race.tNow < race.tMax && !race.raceFinished) {
                race.update()

                if (car.disqualified || car.lastValidSegment!!.id > turn.segments.last().id) break
            }

            turnPassed = car.lastValidSegment!!.id > turn.segments.last().id

            if (turnPassed) {
                // Save the current speed as maximum turn speed.
                turn.maxSpeed = speed
                if (++nextTurnToFind == turns.size) break

                turn = turns[nextTurnToFind]
                speed = iniSpeed
            } else {
                // Reduce the speed and try again.
                speed -= 0.5
            }

            // The turn could not be passed for any reason. Set a default speed.
            if (speed < 0.5) {
                turn.maxSpeed = MIN_TURN_SPEED
                break
            }
        }
    }

    /**
     * Load a TORCS track from an XML file.
     */
    fun importFromTORCS(name: String, invert: Boolean = false) {
        val xElement = XElement.load(PathUtil.inputDir + "tracks/$name.xml")!!

        var xMainTrack: XElement? = null
        var xTrackSegments: XElement? = null
//        var xPits: XElement? = null

        // Find the main section of the XML file.
        for (xSection in xElement.getChildren("section")) {
            if (xSection.getAttributeValue("name") == "Main Track") {
                xMainTrack = xSection
                break
            }
        }

        var trackWidth = 10.0

        // Determine the track width.
        for (xAttNum in xMainTrack!!.getChildren("attnum")) {
            if (xAttNum.getAttributeValue("name") == "width") {
                trackWidth = xAttNum.getAttributeValueAsDouble("val")

                break
            }
        }

        startWidth = trackWidth


//        for (xSection in xMainTrack.getChildren("section")) {
//            if (xSection.getAttributeValue("name") == "Pits") {
//                xPits = xSection
//            }
//        }
//        val pitNames = mutableListOf<String>()
//
//        for (xAttStr in xPits!!.getChildren("attstr")) {
//            pitNames.add(xAttStr.getAttributeValue("val"))
//        }

        // Find the track segment section.
        for (xSection in xMainTrack.getChildren("section")) {
            if (xSection.getAttributeValue("name") == "Track Segments") {
                xTrackSegments = xSection
            }
        }

        // Iterate over all segments.
        for (xTrackSegment in xTrackSegments!!.getChildren("section")) {
            val map = hashMapOf<String, String>()

            // Determine the segment type: str(aight), l(eft)gt, r(ight)gt
            for (xAttStr in xTrackSegment.getChildren("attstr")) {
                val xName = xAttStr.getAttributeValue("name")
                val xValue = xAttStr.getAttributeValue("val")

                if (xName == "type") {
                    map["type"] = xValue
                    break
                }
            }

            // Determine the numeric properties of the segment.
            for (xAttNum in xTrackSegment.getChildren("attnum")) {
                val xName = xAttNum.getAttributeValue("name")
                val xValue = xAttNum.getAttributeValue("val")

                if (xName == "lg") {
                    map["lg"] = xValue
                } else if (xName == "arc") {
                    map["arc"] = xValue
                } else if (xName == "radius") {
                    map["radius"] = xValue
                } else if (xName == "end radius") {
                    map["end radius"] = xValue
                }
            }

            if (map["type"]!! == "str") {
                // Add a straight segment to this track instance.

                val length = map["lg"]!!.toDouble()
                addSegment(length, trackWidth, 0.0)
            } else {
                // Add a turn segment to this track instance.

                val arc = map["arc"]!!.toDouble()
                val leftTurn = map["type"]!! == "lft" // else rgt
                val radiusStart = map["radius"]!!.toDouble()
                val radiusEnd = if (map.containsKey("end radius")) {
                    map["end radius"]!!.toDouble()
                } else {
                    radiusStart
                }

                // Determine the turn angle in [rad]. Left turns have negative sign.
                val angleInRad = arc * (PI / 180.0) * (if (leftTurn) -1.0 else 1.0) * (if (invert) -1.0 else 1.0)

                addTurn(angleInRad, radiusStart, radiusEnd)
            }
        }
    }
}