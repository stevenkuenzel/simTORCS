package simtorcs.util

import simtorcs.track.EdgeSegment
import simtorcs.track.Segment
import simtorcs.geometry.LineSegment
import simtorcs.geometry.Vector2
import kotlin.math.sign

class GeometryUtils {
    companion object {
        /**
         * https://de.wikipedia.org/wiki/Punkt-in-Polygon-Test_nach_Jordan
         */
        fun isWithin(segment: Segment, p: Vector2): Boolean {

            val points = if (segment is EdgeSegment)
            {
                arrayOf(segment.thirdPoint, segment.p1, segment.p2, segment.thirdPoint)
            }
            else
            {
                arrayOf(segment.p3, segment.p1, segment.p2, segment.p4, segment.p3)
            }

            var sign = -1

            for (i in 0 until (points.size - 1)) {
                sign *= rightCross(p, points[i], points[i + 1])
//                t *= kpt(p, points[i], points[i + 1])

                if (sign == 0) break
            }

            return sign >= 0
        }

        fun rightCross(q : Vector2, r_ : Vector2, s_ : Vector2) : Int
        {
            val r = if (r_.y > s_.y) s_ else r_
            val s = if (r_.y > s_.y) r_ else s_

            if (q.y <= r.y || q.y > s.y) return 1

            val d = delta(q, r, s)

            return sign(d).toInt()
        }

        private fun delta(q : Vector2, r : Vector2, s : Vector2) : Double
        {
            return (r.x - q.x) * (s.y - q.y) - (r.y - q.y) * (s.x - q.x)
        }


        /**
         * https://stackoverflow.com/a/6853926
         */
        fun adjPoint(p : Vector2, line: LineSegment) : Vector2?
        {
            val A = p.x - line.from.x
            val B = p.y - line.from.y
            val C = line.to.x - line.from.x
            val D = line.to.y - line.from.y

            val dot = A * C + B * D
            val len_sq = C* C + D *D

            val param = if (len_sq > 0.0) dot / len_sq else -1.0

            return when
            {
                param < 0.0 -> Vector2(line.from.x, line.from.y)
                param > 1.0 -> Vector2(line.to.x, line.to.y)
                else -> Vector2(line.from.x + param * C, line.from.y + param * D)
            }
        }
    }


}