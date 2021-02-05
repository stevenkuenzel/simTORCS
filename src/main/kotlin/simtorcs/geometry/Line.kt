package simtorcs.geometry

class Line(val from: Vector2, val to: Vector2) {

    fun intersect(other: Line): Vector2? {
        val v1 = (from.x * to.y - from.y * to.x)
        val v2 = (other.from.x - other.to.x)
        val v3 = (from.x - to.x)
        val v4 = (other.from.x * other.to.y - other.from.y * other.to.x)
        val v5 = (other.from.y - other.to.y)
        val v6 = (from.y - to.y)
        // x
        val vX = v1 * v2 - v3 * v4
        val vY = v1 * v5 - v6 * v4
        val n = v3 * v5 - v6 * v2

        val x = vX / n
        val y = vY / n

        if (x.isInfinite() || x.isNaN() || y.isInfinite() || y.isNaN()) return null

        return Vector2(x, y)
    }
}