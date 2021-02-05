package simtorcs.geometry


class LineSegment(val from : Vector2, val to : Vector2) {
    // p = from
    // p + r = to
    // r = to - from

    private val r = to.subtract(from)

    /**
     * Determines the intersection point of two line segments.
     *
     * Source: https://stackoverflow.com/a/565282
     */
    fun intersect(other : LineSegment) : Vector2?
    {
        val qp = other.from.subtract(from)
        val qpr = qp.cross(r)
        val rs = r.cross(other.r)

        // Colinear.
        if (rs == 0.0 && qpr == 0.0) return null
        // Parallel.
        if (rs == 0.0 && qpr != 0.0) return null

        if(rs != 0.0)
        {
            val u = qpr / rs

            if (u in 0.0..1.0)
            {
                val qps = qp.cross(other.r)
                val t = qps / rs

                if (t in 0.0..1.0)
                {
                   return from.addNew(r.multiply(t))
                }
            }
        }

        return null
    }
}