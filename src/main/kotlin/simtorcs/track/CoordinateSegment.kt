package simtorcs.track

import simtorcs.geometry.Vector2

/**
Default segment for straights and turns.
 */
class CoordinateSegment(private val length: Double, private val endWidth: Double, val turnAngle: Double, previous: CoordinateSegment?, track: Track) : Segment(previous, track) {

    // Left or right turn, or straight?
    val turn = if (turnAngle < 0.0) TurnDirection.Left else if (turnAngle > 0.0) TurnDirection.Right else TurnDirection.None

    private val startWidth = if (previous != null) previous.endWidth else track.startWidth
    private val halfWidth = startWidth / 2.0

    fun initialize() : EdgeSegment?
    {
        val start1 = previous?.p3?.copy() ?: Vector2(0.0, halfWidth)
        val start2 = previous?.p4?.copy() ?: Vector2(0.0, -halfWidth)

        p1 = start1
        p2 = start2

        if (turn != TurnDirection.None) {

            // Determine the pivot to rotate around.
            val from = if (turn == TurnDirection.Right) p1 else p2
            val to = if (turn == TurnDirection.Right) p2 else p1
            val vec = to.subtract(from)

            val rotVec = from.addNew(vec.rotate(turnAngle))

            // Set the rotated point as one of the starting points.
            if (turn == TurnDirection.Right) {
                p2 = rotVec
            } else {
                p1 = rotVec
            }
        }

        // Determine the global segment direction and a rotated vector accordingly.
        val directionInRad = (previous?.segmentAngle ?: 0.0) + turnAngle
        val directionWithLength = Vector2.fromAngleInRad(directionInRad).multiply(length, length)

        // Determine the rotated end points of the segment.
        p3 = p1.addNew(directionWithLength)
        p4 = p2.addNew(directionWithLength)

        val widthRelation = endWidth / startWidth

        // Move the end points towards each other if the segment gets narrower towards its end, and vice versa accordingly.
        if (widthRelation < 1.0)
        {
            val each = (1.0 - widthRelation) / 2.0

            val direction = p4.subtract(p3)

            direction.mult(each)


            p3 = p3.addNew(direction)

            direction.mult(-1.0)
            p4 = p4.addNew(direction)
        }
        else if (widthRelation > 1.0)
        {
            val each = (widthRelation - 1.0) / 2.0

            val direction = p4.subtract(p3)

            direction.mult(each)


            p4 = p4.addNew(direction)

            direction.mult(-1.0)
            p3 = p3.addNew(direction)
        }

        // Update the segment and center lines.
        updateSegmentLines()
        updateCentres()

        // If required, create an edge segment between the previous and this segment.
        if (previous != null && turn != TurnDirection.None) {
            return EdgeSegment(previous, this, turn, track)
        }

        return null
    }

    override fun toString(): String {
        return "$id   $turn   $turnAngle"
    }
}