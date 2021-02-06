package simtorcs.geometry

import kotlin.math.*

/**
 * Basic 2-d vector. Provides only the necessary methods.
 */
class Vector2(var x : Double, var y : Double) {

    constructor() : this (0.0, 0.0)
    constructor(x : Int, y : Int) : this(x.toDouble(), y.toDouble())

    companion object
    {
        fun fromAngleInRad(angInRad : Double) : Vector2
        {
            return Vector2(cos(angInRad), sin(angInRad))
        }
    }

    fun add(other : Vector2) : Vector2
    {
        return Vector2(x + other.x, y + other.y)
    }

    fun scale(factor : Double) : Vector2
    {
        return Vector2(x * factor, y * factor)
    }

    fun selfScale(factor : Double)
    {
        x *= factor
        y *= factor
    }

    fun subtract(other : Vector2) : Vector2
    {
        return Vector2(x - other.x, y - other.y)
    }

    fun cross(other : Vector2) : Double
    {
        return x * other.y - y * other.x
    }

    fun normalize() : Vector2
    {
        val length = magnitude()

        return Vector2(x / length, y / length)
    }

    fun magnitude() : Double
    {
        return sqrt(sqrMagnitude())
    }

    fun sqrMagnitude() : Double
    {
        return x * x + y * y
    }

    fun distanceTo(other: Vector2): Double
    {
        return sqrt(sqrDistanceTo(other))
    }

    fun sqrDistanceTo(other: Vector2): Double
    {
        val dX = x - other.x
        val dY = y - other.y

        return dX * dX + dY * dY
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