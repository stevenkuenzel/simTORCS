package simtorcs.geometry

import simtorcs.track.EdgeSegment
import simtorcs.track.Segment
import kotlin.math.sign

class GeometryUtils {
    companion object {
        /**
         * Is a point within a segment?
         *
         * Source: https://en.wikipedia.org/wiki/Point_in_polygon
         */
        fun isWithin(segment: Segment, p: Vector2): Boolean {

            val points = if (segment is EdgeSegment) {
                arrayOf(segment.thirdPoint, segment.p1, segment.p2, segment.thirdPoint)
            } else {
                arrayOf(segment.p3, segment.p1, segment.p2, segment.p4, segment.p3)
            }

            var sign = -1

            for (i in 0 until (points.size - 1)) {
                sign *= rightCross(p, points[i], points[i + 1])

                if (sign == 0) break
            }

            return sign >= 0
        }

        fun rightCross(q: Vector2, r_: Vector2, s_: Vector2): Int {
            val r = if (r_.y > s_.y) s_ else r_
            val s = if (r_.y > s_.y) r_ else s_

            if (q.y <= r.y || q.y > s.y) return 1

            val d = delta(q, r, s)

            return sign(d).toInt()
        }

        private fun delta(q: Vector2, r: Vector2, s: Vector2): Double {
            return (r.x - q.x) * (s.y - q.y) - (r.y - q.y) * (s.x - q.x)
        }


        /**
         * Find the adjacent point on a line segment to a point _p_.
         *
         * Source: https://stackoverflow.com/a/6853926
         */
        fun adjPoint(p: Vector2, line: LineSegment): Vector2 {
            val a = p.x - line.from.x
            val b = p.y - line.from.y
            val c = line.to.x - line.from.x
            val d = line.to.y - line.from.y

            val dot = a * c + b * d
            val lengthSquared = c * c + d * d

            val param = if (lengthSquared > 0.0) dot / lengthSquared else -1.0

            return when {
                param < 0.0 -> Vector2(line.from.x, line.from.y)
                param > 1.0 -> Vector2(line.to.x, line.to.y)
                else -> Vector2(line.from.x + param * c, line.from.y + param * d)
            }
        }
    }
}