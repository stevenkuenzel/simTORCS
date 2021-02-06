package simtorcs.geometry


/**
 * A line segment between _from_ and _to_. In contrast to a line, its length is not infinity.
 */
class LineSegment(val from: Vector2, val to: Vector2) {

    private val direction by lazy { to.subtract(from) }

    /**
     * Determines the intersection point of two line segments.
     *
     * Source: https://stackoverflow.com/a/565282
     */
    fun intersect(other: LineSegment): Vector2? {
        val qp = other.from.subtract(from)
        val qpr = qp.cross(direction)
        val rs = direction.cross(other.direction)

        // Colinear.
        if (rs == 0.0 && qpr == 0.0) return null
        // Parallel.
        if (rs == 0.0 && qpr != 0.0) return null

        if (rs != 0.0) {
            val u = qpr / rs

            if (u in 0.0..1.0) {
                val qps = qp.cross(other.direction)
                val t = qps / rs

                if (t in 0.0..1.0) {
                    return from.add(direction.scale(t))
                }
            }
        }

        return null
    }
}