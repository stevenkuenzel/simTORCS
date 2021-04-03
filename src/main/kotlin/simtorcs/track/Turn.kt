package simtorcs.track

import kotlin.math.abs
import kotlin.math.sqrt

class Turn(val id: Int, firstSegment: CoordinateSegment) {
    val direction = firstSegment.turn

    /**
     * Track segments belonging to this turn.
     */
    val segments = mutableListOf<Segment>()

    var length = 0.0
    var angle = 0.0
    var maxSpeed = 100.0

    init {
        add(firstSegment)
    }

    fun add(segment: Segment)
    {
        segment.inTurn = this
        segments.add(segment)
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
    }

    override fun toString(): String {
        return "$direction   $length   $angle"
    }
}