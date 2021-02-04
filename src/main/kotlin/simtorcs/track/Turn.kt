package simtorcs.track

import simtorcs.geometry.LineSegment
import simtorcs.geometry.Vector2
import kotlin.math.abs
import kotlin.math.sqrt

class Turn(val id: Int, firstSegment: CoordinateSegment) {
    val direction = firstSegment.turn

    val segments = mutableListOf<Segment>()

    init {
        add(firstSegment)
    }

    var length = 0.0
    var angle = 0.0
    var apexIn = -1
    var apexOut = -1

    var maxSpeed = 100.0

    val indent = 0.3

    var s = Vector2()
    var m1: Vector2? = Vector2()
    var m2: Vector2? = Vector2()
    var e = Vector2()

    fun add(segment: Segment)
    {
        segment.inTurn = this
        segments.add(segment)
    }

    fun findApex(turnIn: Boolean): Int {
        if (segments.size < 3) return -1
        val startIndex = if (turnIn) 0 else segments.size - 1
        var apex = -1

        val lineStart =
            Segment.getWeightedCentre(
                if (turnIn) segments[startIndex].p2 else segments[startIndex].p4,
                if (turnIn) segments[startIndex].p1 else segments[startIndex].p3,
                if (direction == TurnDirection.Left) 1.0 - indent else indent
            )

        val indices = if (turnIn) Array(segments.size - 2) { i -> startIndex + 2 + i }
        else Array(segments.size - 2) { i -> segments.size - (i + 3) }

        for (index in indices) {
            val lineEnd = Segment.getWeightedCentre(
                segments[index].p1,
                segments[index].p2,
//                if (turnIn) segments[index].p1 else segments[index].p3,
//                if (turnIn) segments[index].p2 else segments[index].p4,
                if (direction == TurnDirection.Left) 1.0 - indent else indent
            )
            val line = LineSegment(lineStart, lineEnd)

            var allIntersect = true

            val indicesBetween = if (turnIn) Array(index - 1) { i -> i } else
                Array(startIndex - index) { i -> startIndex - i }

            for (j in indicesBetween) {

                val int = line.intersect(
                    LineSegment(
                        if (turnIn) segments[j].p3 else segments[j].p1,
                        if (turnIn) segments[j].p4 else segments[j].p2
                    )
                )

                if (int == null) {
                    allIntersect = false
                    break
                }
            }

            if (allIntersect) {
                apex = index
            } else {
                break
            }
        }

        return apex
    }

    fun finalize() {

        // Determine length and total angle.
        length = 0.0
        angle = 0.0

        for (segment in segments) {
            if (segment is CoordinateSegment) {
                val len = sqrt(segment.measuredLength)
                val absAng = abs(segment.turnAngle)

                length += len
                angle += absAng
            }
        }


        // Find apex for entering the turn.
        apexIn = findApex(true)

        val noApex = apexIn == segments.size - 1 || apexIn == -1

        apexOut = if (noApex) -1 else findApex(false)

        if (apexIn > apexOut) {
            val mean = (apexIn + apexOut) / 2
            apexIn = mean
            apexOut = mean
        }




        if (!noApex) {
            s = Segment.getWeightedCentre(
                segments[0].p2,
                segments[0].p1,
                if (direction == TurnDirection.Left) indent else 1.0 - indent
            )

            m1 = Segment.getWeightedCentre(
                segments[apexIn].p1,
                segments[apexIn].p2,
                if (direction == TurnDirection.Left) indent else 1.0 - indent
            )

            m2 = Segment.getWeightedCentre(
                segments[apexOut].p1,
                segments[apexOut].p2,
                if (direction == TurnDirection.Left) indent else 1.0 - indent
            )

            e = Segment.getWeightedCentre(
                segments[segments.size - 1].p4,
                segments[segments.size - 1].p3,
                if (direction == TurnDirection.Left) indent else 1.0 - indent
            )


            val lineIn = LineSegment(s, m1!!)
            val lineOut = LineSegment(if (apexOut != -1) m2!! else m1!!, e)

            for (index in segments.indices) {
                val segment = segments[index]

//                if (segment is CoordinateSegment) {
                val int = when {
                    index < apexIn -> lineIn.intersect(LineSegment(segment.p3, segment.p4))
                    index >= apexOut -> lineOut.intersect(LineSegment(segment.p3, segment.p4))
                    else -> lineOut.intersect(LineSegment(segment.p3, segment.p4))
                }

                segment.idealStart = if (index == 0) s else segments[index - 1].idealEnd

                if (int != null) {

                    segment.idealEnd = int
                } else {

                    if (index == segments.size - 1) {
                        segment.idealEnd = e
                    }
                    else {
                        segment.idealEnd = Segment.getWeightedCentre(
                            segment.p4,
                            segment.p3,
                            if (direction == TurnDirection.Left) 1.0 - indent else indent
                        )
                    }
                }
            }
        } else {// if (apexIn == segments.size - 1) {
            val line = LineSegment(segments[0].centreStart, segments[segments.size - 1].centreEnd)

            for (index in segments.indices) {
                val segment = segments[index]
                segment.idealStart = if (index == 0) segment.centreStart else segments[index - 1].idealEnd

                if (index == segments.size - 1) {
                    segment.idealEnd = segment.centreEnd
                } else {
                    val intersection = line.intersect(LineSegment(segment.p3, segment.p4))

                    if (intersection == null) {
                        // TODO: Das ist so nicht richtig.
                        segment.idealEnd = segment.centreEnd
                    } else {
                        segment.idealEnd = intersection
                    }
                }
            }
        }
    }

    override fun toString(): String {
        return "$direction   $length   $angle   $apexIn   $apexOut"
    }
}