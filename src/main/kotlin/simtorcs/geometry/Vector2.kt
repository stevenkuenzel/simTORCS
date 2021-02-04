package simtorcs.geometry

import kotlin.math.*

class Vector2(var x : Double, var y : Double) {
    companion object
    {
        /**
         * https://math.stackexchange.com/a/295827
         */
        fun fromAngleInRad(angInRad : Double) : Vector2
        {
            return Vector2(cos(angInRad), sin(angInRad))
        }
    }
    constructor(x : Int, y : Int) : this(x.toDouble(), y.toDouble())
    constructor() : this (0.0, 0.0)

    fun multiply(x_ : Double, y_ : Double) : Vector2
    {
        return Vector2(x * x_, y * y_)
    }

    fun multiply(factor : Double) : Vector2
    {
        return Vector2(x * factor, y * factor)
    }

    fun mult(factor : Double)
    {
        x *= factor
        y *= factor
    }

    fun addNew(other : Vector2) : Vector2
    {
        return Vector2(x + other.x, y + other.y)
    }

    fun add(other : Vector2)
    {
        x += other.x
        y += other.y
    }

    fun addMultiple(other : Vector2, factor : Double)
    {
        x += other.x * factor
        y += other.y * factor
    }

    fun add(value : Double) : Vector2
    {
        return add(value, value)
    }

    fun add(x_ : Double, y_ : Double) : Vector2
    {
        return Vector2(x + x_, y + y_)
    }

    fun subtract(other : Vector2) : Vector2
    {
        return Vector2(x - other.x, y - other.y)
    }

    fun norm() : Vector2
    {
        val length = magn()

        return Vector2(x / length, y / length)
    }

    fun magn() : Double
    {
        return sqrt(magnSqr())
    }

    fun magnSqr() : Double
    {
        return x * x + y * y
    }

    fun distance(other: Vector2): Double
    {
        return sqrt(sqrDistance(other))
    }
    fun sqrDistance(other: Vector2): Double
    {
        val dX = x - other.x
        val dY = y - other.y

        return dX * dX + dY * dY
    }

    fun dot(other: Vector2) : Double
    {
        return x * other.x + y * other.y
    }

    fun cross(other : Vector2) : Double
    {
        return x * other.y - y * other.x
    }



//    /**
//    Requires both vectors to be normalized.
//     */
//    fun ang(other : Vector2) : Double
//    {
//        return acos(dot(other))
//    }

    /**
    Requires both vectors to be normalized.
     */
    fun ang(other : Vector2) : Double
    {
        val a = x * other.y - y * other.x
        val b = x * other.x + y * other.y

        return atan2(a, b)
    }

    fun rotate(angInRad : Double) : Vector2
    {
        val sn = sin(angInRad)
        val cs = cos(angInRad)

        return Vector2(cs * x - sn * y, sn * x + cs * y)
    }

    fun copy() : Vector2
    {
        return Vector2(x, y)
    }

    override fun toString(): String {
        return "($x, $y)"
    }
}