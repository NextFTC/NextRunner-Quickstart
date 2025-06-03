package org.firstinspires.ftc.teamcode.swerve

import com.acmerobotics.roadrunner.geometry.DualNum
import kotlin.math.atan2

fun <Param> atan2Dual(y: DualNum<Param>, x: DualNum<Param>): DualNum<Param> {
    require(x.size() == y.size()) { "x and y must have the same size" }

    val out = DualNum<Param>(DoubleArray(x.size()))
    if (out.size() == 0) return out

    // Calculate the base atan2 value
    val atan2Val = atan2(y[0], x[0])
    val result = DoubleArray(x.size())
    result[0] = atan2Val
    if (out.size() == 1) return DualNum(result)

    // Calculate the denominator (x^2 + y^2)
    val denom = x[0] * x[0] + y[0] * y[0]

    // First derivative: (x * dy/dt - y * dx/dt) / (x^2 + y^2)
    result[1] = (x[0] * y[1] - y[0] * x[1]) / denom
    if (out.size() == 2) return DualNum(result)

    // For the second derivative, we need to use the quotient rule and chain rule
    // Let f(t) = x * dy/dt - y * dx/dt and g(t) = x^2 + y^2
    // Then d/dt(f/g) = (g * df/dt - f * dg/dt) / g^2

    // Calculate f(t) = x * dy/dt - y * dx/dt
    val f = x[0] * y[1] - y[0] * x[1]

    // Calculate df/dt = dx/dt * dy/dt + x * d2y/dt2 - dy/dt * dx/dt - y * d2x/dt2
    //                 = x * d2y/dt2 - y * d2x/dt2
    val dfdt = x[0] * y[2] - y[0] * x[2]

    // Calculate dg/dt = 2 * x * dx/dt + 2 * y * dy/dt
    val dgdt = 2 * (x[0] * x[1] + y[0] * y[1])

    // Second derivative: (g * df/dt - f * dg/dt) / g^2
    result[2] = (denom * dfdt - f * dgdt) / (denom * denom)
    if (out.size() == 3) return DualNum(result)

    // For the third derivative, we need to continue applying the quotient rule and chain rule
    // This gets very complex, so we'll use a more systematic approach

    // Let's define some helper variables
    val x0 = x[0]
    val y0 = y[0]
    val x1 = x[1]
    val y1 = y[1]
    val x2 = x[2]
    val y2 = y[2]
    val x3 = x[3]
    val y3 = y[3]

    // Calculate d2f/dt2 = d/dt(x * d2y/dt2 - y * d2x/dt2)
    //                    = dx/dt * d2y/dt2 + x * d3y/dt3 - dy/dt * d2x/dt2 - y * d3x/dt3
    val d2fdt2 = x1 * y2 + x0 * y3 - y1 * x2 - y0 * x3

    // Calculate d2g/dt2 = d/dt(2 * (x * dx/dt + y * dy/dt))
    //                    = 2 * (dx/dt * dx/dt + x * d2x/dt2 + dy/dt * dy/dt + y * d2y/dt2)
    val d2gdt2 = 2 * (x1 * x1 + x0 * x2 + y1 * y1 + y0 * y2)

    // Calculate d/dt(g * df/dt - f * dg/dt) using the product rule
    val numerator = denom * d2fdt2 + dgdt * dfdt - dgdt * dfdt - f * d2gdt2

    // Calculate d/dt(g^2) = 2 * g * dg/dt
    val denomDerivative = 2 * denom * dgdt

    // Apply the quotient rule for the third derivative
    result[3] = (denom * denom * numerator - (denom * dfdt - f * dgdt) * denomDerivative) / (denom * denom * denom)

    return DualNum(result)
}
