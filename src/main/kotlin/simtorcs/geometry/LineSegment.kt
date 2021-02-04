package simtorcs.geometry

/**
 * https://stackoverflow.com/a/565282
 */
class LineSegment(val from : Vector2, val to : Vector2) {
    // p = from
    // p + r = to
    // r = to - from

    val r = to.subtract(from)

    fun intersect(other : LineSegment) : Vector2?
    {
        // q = other.from
        // s = other.r

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