package org.firstinspires.ftc.teamcode.swerve

import com.acmerobotics.roadrunner.control.RobotKinematics
import com.acmerobotics.roadrunner.control.WheelIncrements
import com.acmerobotics.roadrunner.control.WheelVelocities
import com.acmerobotics.roadrunner.geometry.DualNum
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Twist2dDual
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.geometry.Vector2dDual

typealias SwerveWheelIncrement<Param> = Pair<DualNum<Param>, DualNum<Param>>
typealias SwerveWheelVelocity<Param> = Pair<DualNum<Param>, DualNum<Param>>

/**
 * @param[modules] list of swerve module configurations (position and orientation)
 */
data class SwerveKinematics(
    @JvmField
    val modules: List<Vector2d>
) : RobotKinematics<SwerveKinematics.SwerveWheelIncrements<*>, SwerveKinematics.SwerveWheelVelocities<*>> {

    data class SwerveWheelIncrements<Param>(
        @JvmField
        val wheelDeltas: List<DualNum<Param>>,
        @JvmField
        val steeringAngles: List<DualNum<Param>>
    ) : WheelIncrements<Param>, Iterable<SwerveWheelIncrement<Param>> {
        override fun iterator(): Iterator<SwerveWheelIncrement<Param>> =
            wheelDeltas.zip(steeringAngles).iterator()
    }

    override fun <Param> forward(w: SwerveWheelIncrements<*>): Twist2dDual<Param> {
        w as SwerveWheelIncrements<Param>

        // Initialize accumulators for the robot's overall motion
        val size = w.wheelDeltas.first().size()
        var sumX = DualNum.constant<Param>(0.0, size)
        var sumY = DualNum.constant<Param>(0.0, size)
        var sumAngular = DualNum.constant<Param>(0.0, size)

        // Process each module's contribution to the robot's motion
        modules.forEachIndexed { index, module ->
            // Get the wheel delta and steering angle for this module
            val wheelDelta = w.wheelDeltas[index]
            val steeringAngle = w.steeringAngles[index]

            // Convert wheel delta and steering angle to x and y components
            // Using the steering angle to determine the direction of motion
            val cosAngle = steeringAngle.cos()
            val sinAngle = steeringAngle.sin()

            // Calculate the module's contribution to linear motion
            val moduleX = wheelDelta * cosAngle
            val moduleY = wheelDelta * sinAngle

            // Add to the linear motion accumulators
            sumX = sumX.plus(moduleX)
            sumY = sumY.plus(moduleY)

            // Calculate the module's contribution to angular motion
            // This is the cross product of the module position and its velocity vector
            val angularContribution = (moduleY * module.x) - (moduleX * module.y)
            sumAngular = sumAngular.plus(angularContribution)
        }

        // Average the contributions from all modules
        val numModules = modules.size.toDouble()
        val avgX = sumX.div(numModules)
        val avgY = sumY.div(numModules)
        val avgAngular = sumAngular.div(numModules)

        // Return the resulting twist
        return Twist2dDual(
            Vector2dDual(avgX, avgY),
            avgAngular
        )
    }

    data class SwerveWheelVelocities<Param>(
        @JvmField
        val wheelVels: List<DualNum<Param>>,
        @JvmField
        val steeringAngles: List<DualNum<Param>>
    ) : WheelVelocities<Param>, Iterable<SwerveWheelVelocity<Param>> {
        override fun all() = wheelVels

        override fun iterator(): Iterator<SwerveWheelVelocity<Param>> =
            wheelVels.zip(steeringAngles).iterator()
    }

    override fun <Param> inverse(t: PoseVelocity2dDual<Param>): SwerveWheelVelocities<Param> {
        val wheelVels = mutableListOf<DualNum<Param>>()
        val steeringAngles = mutableListOf<DualNum<Param>>()

        // Calculate wheel velocities and steering angles for each module
        modules.forEach { module ->
            // Calculate the velocity at the module position due to robot rotation
            // This is the cross product of angular velocity and the module position vector
            val rotVelX = t.angVel * -module.y
            val rotVelY = t.angVel * module.x

            // Combine the robot's linear velocity with the rotational velocity at this module
            val totalVelX = t.linearVel.x + rotVelX
            val totalVelY = t.linearVel.y + rotVelY

            // Calculate the wheel velocity (magnitude of the velocity vector)
            val wheelVel = totalVelX.times(totalVelX).plus(totalVelY.times(totalVelY)).sqrt()
            wheelVels.add(wheelVel)

            // Calculate the steering angle using Rotation2d
            // We use atan2 to get the angle of the velocity vector
            val steeringAngle = atan2Dual(totalVelY, totalVelX)
            steeringAngles.add(steeringAngle)
        }

        // Find maximum wheel velocity for normalization if needed
        val maxWheelVel = wheelVels.maxBy { it.value() }

        // Normalize wheel velocities if any exceeds 1.0
        if (maxWheelVel.value() > 1.0) {
            wheelVels.forEachIndexed { index, wheelVel ->
                wheelVels[index] = wheelVel.div(maxWheelVel)
            }
        }

        return SwerveWheelVelocities(wheelVels, steeringAngles)
    }
}