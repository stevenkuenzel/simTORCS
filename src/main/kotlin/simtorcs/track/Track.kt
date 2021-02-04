package simtorcs.track

import de.stevenkuenzel.util.PathUtil
import de.stevenkuenzel.xml.XElement
import simtorcs.car.Car
import simtorcs.car.control.TestController
import simtorcs.race.Race
import simtorcs.visualization.RaceAWTComponent
import simtorcs.geometry.Vector2
import javax.swing.JFrame
import kotlin.math.PI
import kotlin.math.abs

class Track(val size: Int, val levelOfDetail: Int = 1, val name: String = "") {

    companion object {
        fun load(name: String, levelOfDetail: Int): Track {
            val invert = name.startsWith("!")
            val t = Track(0, levelOfDetail, name)

            t.import((if (invert) name.substring(1) else name).toLowerCase(), invert)
            t.finalize()

            return t
        }
    }


    val segments = mutableListOf<Segment>()

    val minTurnRad = 0.1 / levelOfDetail.toDouble()
    val maxCurveSegmentLengthSqr = 2500.0
    var startWidth = 13.0

    var prevSegment: CoordinateSegment? = null


    var xMin = Double.MAX_VALUE
    var xMax = Double.NEGATIVE_INFINITY
    var yMin = Double.MAX_VALUE
    var yMax = Double.NEGATIVE_INFINITY


    var nextSegmentID = 0

    val turns = mutableListOf<Turn>()

    fun addTurn(angleInRad: Double, radiusStart: Double, radiusEnd: Double) {
        val angHalf = angleInRad / 2.0
        val l1 = radiusStart * abs(angHalf)
        val l2 = radiusEnd * abs(angHalf)

        // First half.
        for (i in 0 until levelOfDetail) {
            addSegment(l1 / levelOfDetail.toDouble(), startWidth, angHalf / levelOfDetail.toDouble())
        }

        // Second half.
        for (i in 0 until levelOfDetail) {
            addSegment(l2 / levelOfDetail.toDouble(), startWidth, angHalf / levelOfDetail.toDouble())
        }
    }

    fun addSegment(length: Double, width: Double, angle: Double) {
        val newSegment = CoordinateSegment(length, width, angle, prevSegment, this)

        val edgeSegment = newSegment.initialize()

        if (edgeSegment != null) {
            edgeSegment.id = nextSegmentID++
            checkMinMaxOfSegment(edgeSegment)
            segments.add(edgeSegment)
        }

        newSegment.id = nextSegmentID++

        checkMinMaxOfSegment(newSegment)
        segments.add(newSegment)

        prevSegment = newSegment
    }

    fun addSegment(length: Int, width: Int, angle: Int) {
        addSegment(length.toDouble(), width.toDouble(), angle.toDouble() / (180.0 / PI))
    }

    fun checkMinMaxOfSegment(segment: Segment) {
        val segmentMinMax = segment.getXYMinMax()

        val min = segmentMinMax.first
        val max = segmentMinMax.second

        if (min.x < xMin) xMin = min.x
        if (min.y < yMin) yMin = min.y
        if (max.x > xMax) xMax = max.x
        if (max.y > yMax) yMax = max.y
    }


    var length = 0.0
    var straightLength = 0.0

    var startingPoint = Vector2()
    var startingDirection = 0.0

    fun finalize() {
        val firstSegment = segments[0]
        val lastSegment = segments[segments.size - 1]

//        val point1 = Segment.getWeightedCentre(firstSegment.p1, lastSegment.p3, 0.5)
//        val point2 = Segment.getWeightedCentre(firstSegment.p2, lastSegment.p4,0.5)
//
//        firstSegment.p1 = point1
//        firstSegment.p2 = point2
//        lastSegment.p3 = point1
//        lastSegment.p4 = point2
//
//        firstSegment.updateSegmentLines()
//        firstSegment.updateCentres()
//        lastSegment.updateSegmentLines()
//        lastSegment.updateCentres()
//

        val connectingSegment = ConnectingSegment(lastSegment, firstSegment, this)
        connectingSegment.id = nextSegmentID++

        segments.add(connectingSegment)


        startingPoint = Segment.getWeightedCentre(firstSegment.centreStart, firstSegment.centreEnd, 0.9)
        startingDirection = firstSegment.segmentAngle

        findTurnSegments()
        findMaxTurnSpeeds()

//        showTrack()

        for (segment in segments) {
            segment.totalTrackLength = length

            length += segment.measuredLength

            if (segment.inTurn == null) straightLength += segment.measuredLength
        }
    }

    fun findMaxTurnSpeeds() {
        val visualize = false
        val iniSpeed = 92.0 // ~ 330.0 / 3.6
        var nextTurnToFind = 0


        var turn = turns[nextTurnToFind]
        var turnOK: Boolean
        var speed = iniSpeed


        while (true) {
            val r = Race(this)
            val c = Car(r, false)
            r.cars.add(c)
            c.setController(TestController(speed))

            val firstSegmentID = turn.segments.first().id

            val prevSegment = segments[if (firstSegmentID > 0) firstSegmentID - 1 else segments.size - 1]
            c.currentSegment = prevSegment
            c.position =
                prevSegment.centreStart.addNew(prevSegment.centreEnd.subtract(prevSegment.centreStart).multiply(0.1))
            c.heading = prevSegment.segmentAngle

            val hv = Vector2.fromAngleInRad(c.heading)
            c.velocity = Vector2(hv.x * speed, hv.y * speed)

            val myFrame: JFrame? = if (visualize) JFrame("RACE") else null
            if (myFrame != null) {
                myFrame.add(RaceAWTComponent(r, 850))
                myFrame.setSize(900, 900)
                myFrame.setVisible(true)
            }



            while (r.tNow < r.tMax && !r.raceFinished) {
                r.update()

                if (c.disqualified) break
                if (c.lastValidSegment!!.id > turn.segments.last().id) break

                if (myFrame != null) {
                    myFrame.repaint()
                    Thread.sleep((1000.0 / (r.fps * 10).toDouble()).toLong())
                }
            }

            myFrame?.dispose()

            turnOK = c.lastValidSegment!!.id > turn.segments.last().id

            if (turnOK) {
                turn.maxSpeed = speed
                if (++nextTurnToFind == turns.size) break

                turn = turns[nextTurnToFind]
                speed = iniSpeed
            } else {
                speed -= 0.5
            }

            if (speed < 0.5) {
                turn.maxSpeed = 5.0 // TODO. Arbitrary Default Value.
                break
            }
        }
    }

    fun showTrack() {
        val r = Race(this)
        val c = Car(r, false)
        r.cars.add(c)
        c.setController(TestController(50.0))

        val myFrame = JFrame("Track: $name")
        myFrame.add(RaceAWTComponent(r, 850))
        myFrame.setSize(900, 900)
        myFrame.setVisible(true)
        r.update()
        myFrame.repaint()
    }

    fun findTurnSegments() {
        var currentTurn: Turn? = null

        var nextID = 0

        for (segment in segments) {
            if (segment is CoordinateSegment) {
                if (segment.turn != TurnDirection.None && abs(segment.turnAngle) >= minTurnRad && segment.measuredLength <= maxCurveSegmentLengthSqr) {
                    when {
                        currentTurn == null -> {
                            currentTurn = Turn(nextID++, segment)
                        }
                        currentTurn.direction != segment.turn -> {
                            turns.add(currentTurn.apply { finalize() })
                            currentTurn = Turn(nextID++, segment)
                        }
                        else -> {
                            currentTurn.add(segment)
                        }
                    }
                } else if (currentTurn != null) {
                    turns.add(currentTurn.apply { finalize() })
                    currentTurn = null
                }
            } else if (segment is EdgeSegment && currentTurn != null) {
                currentTurn.add(segment)
            }
        }

        if (currentTurn != null) turns.add(currentTurn.apply { finalize() })


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

//        updateIdealLine() // TODO: Deactivated as with increasing LOD the ideal line computation becomes worse.
    }

//    fun updateIdealLine() {
//        val filtered = segments.filterIsInstance<CoordinateSegment>()
//
//        for (index in filtered.indices) {
//            val a = filtered[index]
//            val b = filtered[(index + 1) % filtered.size]
//
//            if (a.turn == TurnDirection.None && b.turn != TurnDirection.None) {
//                a.idealEnd = b.idealStart
//                continue
//            }
//
//            if (a.turn != TurnDirection.None && b.turn == TurnDirection.None) {
//                b.idealStart = a.idealEnd
//                continue
//            }
//
//
//            val pt = Segment.getWeightedCentre(a.idealEnd, b.idealStart, 0.5)
//
//            a.idealEnd = pt
//            b.idealStart = pt
//        }
//
//        for (index in segments.indices) {
//            val segment = segments[index]
//
//            if (!(segment is CoordinateSegment)) {
//                val prev = segments[if (index > 0) index - 1 else segments.size - 1]
//                val next = segments[(index + 1) % segments.size]
//
//                segment.idealStart = prev.idealEnd
//                segment.idealEnd = next.idealStart
//            }
//        }
//
//        segments.forEach { it.updateIdeal() }
//    }


    fun import(name: String, invert: Boolean = false) {
        val xElement = XElement.load(PathUtil.inputDir + "tracks/$name.xml")!!

        var xMainTrack: XElement? = null
        var xTrackSegments: XElement? = null
        var xPits: XElement? = null

        for (xSection in xElement.getChildren("section")) {
            if (xSection.getAttributeValue("name") == "Main Track") {
                xMainTrack = xSection
                break
            }
        }

        var trackWidth = 10.0

        for (xAttNum in xMainTrack!!.getChildren("attnum")) {
            if (xAttNum.getAttributeValue("name") == "width") {
                trackWidth = xAttNum.getAttributeValueAsDouble("val")

                break
            }
        }

        startWidth = trackWidth


        for (xSection in xMainTrack.getChildren("section")) {
            if (xSection.getAttributeValue("name") == "Pits") {
                xPits = xSection
            }
        }

        val pitNames = mutableListOf<String>()

        for (xAttStr in xPits!!.getChildren("attstr")) {
            pitNames.add(xAttStr.getAttributeValue("val"))
        }

        for (xSection in xMainTrack.getChildren("section")) {
            if (xSection.getAttributeValue("name") == "Track Segments") {
                xTrackSegments = xSection
            }
        }


        for (xTrackSegment in xTrackSegments!!.getChildren("section")) {
            val map = hashMapOf<String, String>()

            val segmentName = xTrackSegment.getAttributeValue("name")

//            if (pitNames.contains(segmentName)) continue
//            if (segmentName.contains("pit")) continue

            for (xAttStr in xTrackSegment.getChildren("attstr")) {
                val xName = xAttStr.getAttributeValue("name")
                val xValue = xAttStr.getAttributeValue("val")

                if (xName == "type") {
                    map["type"] = xValue
                    break
                }
            }

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

                val lg = map["lg"]!!.toDouble()
                addSegment(lg, trackWidth, 0.0)
            } else {
                val arc = map["arc"]!!.toDouble()
                val leftTurn = map["type"]!! == "lft" // else rgt
                val r1 = map["radius"]!!.toDouble()
                val r2 = if (map.containsKey("end radius")) {
                    map["end radius"]!!.toDouble()
                } else {
                    r1
                }


                val angleInRad = arc * (PI / 180.0) * (if (leftTurn) -1.0 else 1.0) * (if (invert) -1.0 else 1.0)

                addTurn(angleInRad, r1, r2)
            }
        }
    }
}