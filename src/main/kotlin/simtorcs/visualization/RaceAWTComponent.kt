package simtorcs.visualization

import simtorcs.race.Race
import simtorcs.track.CoordinateSegment
import simtorcs.track.TurnDirection
import simtorcs.geometry.Vector2
import simtorcs.track.Track
import java.awt.*
import kotlin.math.abs

/**
 * For visualization on a JFrame.
 */
class RaceAWTComponent(val race: Race, val drawSize: Int) : Component() {

    override fun paint(g: Graphics?) {
        val g2d = g!! as Graphics2D
        g2d.setRenderingHints(RenderingHints(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON))

        // Draw the track segments. Size is normalized.
        for (segment in race.track.segments) {
            g2d.color = when (segment) {
                is CoordinateSegment -> when {
                    segment.turn == TurnDirection.Right && abs(segment.turnAngle) >= race.track.minTurnRad && segment.measuredLength <= Track.MAX_TURN_SEGMENT_LENGTH_SQR -> Color.RED
                    segment.turn == TurnDirection.Left && abs(segment.turnAngle) >= race.track.minTurnRad && segment.measuredLength <=  Track.MAX_TURN_SEGMENT_LENGTH_SQR -> Color.BLUE
                    else -> Color.GREEN
                }
                else -> Color.BLACK
            }

            // Draw the left and right border.
            for (line in segment.getNormSegmentLines()) {

                g2d.drawLine(
                    (line.from.x * drawSize.toDouble()).toInt(),
                    (line.from.y * drawSize.toDouble()).toInt(),
                    (line.to.x * drawSize.toDouble()).toInt(),
                    (line.to.y * drawSize.toDouble()).toInt()
                )
            }

            // Draw the ideal line.
//            g2d.color = Color.BLUE
//            val axis = segment.getNormIdeal()
//
//            g2d.drawLine(
//                (axis.from.x * drawSize.toDouble()).toInt(),
//                (axis.from.y * drawSize.toDouble()).toInt(),
//                (axis.to.x * drawSize.toDouble()).toInt(),
//                (axis.to.y * drawSize.toDouble()).toInt()
//            )
        }


        g2d.color = Color.BLUE

        // Draw the cars.
        for (car in race.cars) {
            val x =
                (((car.position.x - race.track.xMin) / (race.track.xMax - race.track.xMin)) * drawSize.toDouble()).toInt()
            val y =
                (((car.position.y - race.track.yMin) / (race.track.yMax - race.track.yMin)) * drawSize.toDouble()).toInt()

            val lookingDir = Vector2.fromAngleInRad(car.heading)
            val endPoint = car.position.add(lookingDir.scale(10.0))

            val xTo =
                (((endPoint.x - race.track.xMin) / (race.track.xMax - race.track.xMin)) * drawSize.toDouble()).toInt()
            val yTo =
                (((endPoint.y - race.track.yMin) / (race.track.yMax - race.track.yMin)) * drawSize.toDouble()).toInt()

            g2d.drawOval(x - 4, y - 4, 8, 8)
            g2d.drawLine(x, y, xTo, yTo)
        }
    }
}