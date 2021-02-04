package simtorcs.track

import simtorcs.geometry.LineSegment
import simtorcs.geometry.Vector2

class EdgeSegment(from : Segment, to: Segment, val turn: TurnDirection, track: Track) : Segment(from, track) {

    val thirdPoint : Vector2

    init {
        p1 = from.p3
        p2 = from.p4

        when (turn) {
            TurnDirection.Right -> {
                p3 = p1
                p4 = to.p2

                thirdPoint = p4

                segmentLines.add(LineSegment(p2, p4))
            }
            TurnDirection.Left -> {
                p3 = to.p1
                p4 = p2

                thirdPoint = p3

                segmentLines.add(LineSegment(p1, p3))
            }
            else -> {
                TODO("This case should not happen.")
            }
        }

        updateCentres()
    }

    override fun toString(): String {
        return "$id   $turn"
    }
}