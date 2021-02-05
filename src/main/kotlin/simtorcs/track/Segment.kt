package simtorcs.track

import simtorcs.geometry.Line
import simtorcs.geometry.LineSegment
import simtorcs.geometry.Vector2
import kotlin.math.*


abstract class Segment(val previous: Segment?, val track: Track) {
    companion object {
        fun getWeightedCentre(a: Vector2, b: Vector2, factor: Double): Vector2 {
            return a.multiply(factor).addNew(b.multiply(1.0 - factor))
        }
    }


    var id = -1

    // The segment coordinates:
    /*
    p3    p4
    |-----|
    |  e  |
    |     |
    |  s  |
    |-----|
    p1   p2
    */
    var p1 = Vector2()
    var p2 = Vector2()
    var p3 = Vector2()
    var p4 = Vector2()

    /**
     * Centre point between p1<->p2
     */
    var centreStart = Vector2()

    /**
     * Centre point between p3<->p4
     */
    var centreEnd = Vector2()

    /**
     *The lines between the points p1 .. p4
     */
    val segmentLines = mutableListOf<LineSegment>()


    /**
     * Next turn on the track.
     */
    var nextTurn: Turn? = null

    /**
     * Is the segment part of a turn?
     */
    var inTurn: Turn? = null

    var axis = LineSegment(Vector2(), Vector2())

    /**
     * Angle in [rad].
     */
    var segmentAngle = 0.0

    /**
     * Angle translated into a 2-d vector.
     */
    var segmentDirection = Vector2()


//    var idealAngle = 0.0
//    var idealDirection = Vector2()


    // Meta information.
    var measuredLength = 0.0
    var totalTrackLength = 0.0
    var widthStart = 0.0
    var widthEnd = 0.0

//    var idealStart = Vector2()
//    var idealEnd = Vector2()
//    var ideal = LineSegment(Vector2(), Vector2())

    /**
     * Creates the segment lines.
     */
    fun updateSegmentLines() {
        segmentLines.clear()
        segmentLines.add(LineSegment(p1, p3))
        segmentLines.add(LineSegment(p2, p4))
    }

    /**
     * Updates the center points and further meta information of the segment.
     */
    fun updateCentres() {
        centreStart = getWeightedCentre(p1, p2, 0.5)
        centreEnd = getWeightedCentre(p3, p4, 0.5)

        axis = LineSegment(centreStart, centreEnd)

        segmentDirection = centreEnd.subtract(centreStart).norm()
        segmentAngle = atan2(segmentDirection.y, segmentDirection.x)

        measuredLength = centreEnd.distance(centreStart)

        widthStart = p1.distance(p2)
        widthEnd = p3.distance(p4)

//        idealStart = centreStart
//        idealEnd = centreEnd

//        updateIdeal()
    }

//    fun updateIdeal() {
//        ideal = LineSegment(idealStart, idealEnd)
//
//        if (idealStart.sqrDistance(idealEnd) == 0.0) return
//
//        idealDirection = idealEnd.subtract(idealStart).norm()
//        idealAngle = atan2(idealDirection.y, idealDirection.x)
//    }

    /**
     * Returns the normalized segment lines. Required for drawing the segment on the screen.
     */
    fun getNormSegmentLines(): List<Line> {
        val result = mutableListOf<Line>()

        for (line in segmentLines) {
            val fromNew = Vector2(
                (line.from.x - track.xMin) / (track.xMax - track.xMin),
                (line.from.y - track.yMin) / (track.yMax - track.yMin)
            )
            val toNew = Vector2(
                (line.to.x - track.xMin) / (track.xMax - track.xMin),
                (line.to.y - track.yMin) / (track.yMax - track.yMin)
            )

            result.add(Line(fromNew, toNew))
        }

        return result
    }

    /**
     * Determines the global min. and max. x- and y-values of the segment. Required for drawing the track on the screen.
     */
    fun getXYMinMax(): Pair<Vector2, Vector2> {
        val min = Vector2(p1.x, p1.y)
        val max = Vector2(p1.x, p1.y)

        val arr = arrayOf(p2, p3, p4)

        for (vector2 in arr) {
            if (vector2.x < min.x) min.x = vector2.x
            if (vector2.x > max.x) max.x = vector2.x
            if (vector2.y < min.y) min.y = vector2.y
            if (vector2.y > max.y) max.y = vector2.y
        }

        return Pair(min, max)
    }

    override fun equals(other: Any?): Boolean {
        if (other is Segment) {
            return other.id == id
        }

        return false
    }

    override fun hashCode(): Int {
        return id
    }


//    fun getNormAxis(): Line {
//        val fromNew = Vector2(
//            (axis.from.x - track.xMin) / (track.xMax - track.xMin),
//            (axis.from.y - track.yMin) / (track.yMax - track.yMin)
//        )
//        val toNew = Vector2(
//            (axis.to.x - track.xMin) / (track.xMax - track.xMin),
//            (axis.to.y - track.yMin) / (track.yMax - track.yMin)
//        )
//
//        return Line(fromNew, toNew)
//    }

//    fun getNormIdeal(): Line {
//        val fromNew = Vector2(
//            (ideal.from.x - track.xMin) / (track.xMax - track.xMin),
//            (ideal.from.y - track.yMin) / (track.yMax - track.yMin)
//        )
//        val toNew = Vector2(
//            (ideal.to.x - track.xMin) / (track.xMax - track.xMin),
//            (ideal.to.y - track.yMin) / (track.yMax - track.yMin)
//        )
//
//        return Line(fromNew, toNew)
//    }

}