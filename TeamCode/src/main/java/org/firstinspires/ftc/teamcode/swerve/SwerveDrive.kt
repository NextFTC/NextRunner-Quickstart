package org.firstinspires.ftc.teamcode.swerve

import com.acmerobotics.roadrunner.control.HolonomicController
import com.acmerobotics.roadrunner.control.MotorFeedforward
import com.acmerobotics.roadrunner.control.RobotPosVelController
import com.acmerobotics.roadrunner.control.WheelVelConstraint
import com.acmerobotics.roadrunner.ftc.Drive
import com.acmerobotics.roadrunner.ftc.FollowerParams
import com.acmerobotics.roadrunner.ftc.Localizer
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual
import com.acmerobotics.roadrunner.geometry.Time
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.profiles.AngularVelConstraint
import com.acmerobotics.roadrunner.profiles.MinVelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileAccelConstraint
import com.acmerobotics.roadrunner.profiles.ProfileParams
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.trajectories.TurnConstraints
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.MecanumDrive

data class SwerveModule(val driveMotor: DcMotorEx, val steeringMotor: DcMotorEx) {
    constructor(hardwareMap: HardwareMap, driveMotorName: String, steeringMotorName: String) :
            this(hardwareMap[DcMotorEx::class.java, driveMotorName], hardwareMap[DcMotorEx::class.java, steeringMotorName])
}

class SwerveDrive(hardwareMap: HardwareMap, startPose: Pose2d): Drive {
    object PARAMS {
        val logoFacingDirection: LogoFacingDirection = LogoFacingDirection.UP
        val usbFacingDirection: UsbFacingDirection = UsbFacingDirection.FORWARD

        val modulePositions: List<Vector2d> = listOf(
            Vector2d(0.0, 0.0),
            Vector2d(0.0, 0.0),
            Vector2d(0.0, 0.0),
            Vector2d(0.0, 0.0)
        )

        val driveMotorNames: List<String> = listOf(
            "leftFrontDrive",
            "rightFrontDrive",
            "leftBackDrive",
            "rightBackDrive"
        )

        val steeringMotorNames: List<String> = listOf(
            "leftFrontSteer",
            "rightFrontSteer",
            "leftBackSteer",
            "rightBackSteer"
        )

        // drive model parameters
        const val inPerTick: Double = 1.0

        // feedforward parameters (in tick units)
        const val kS: Double = 0.0
        const val kV: Double = 0.0
        const val kA: Double = 0.0

        // path profile parameters (in inches)
        const val maxWheelVel: Double = 50.0
        const val minProfileAccel: Double = -30.0
        const val maxProfileAccel: Double = 50.0

        // turn profile parameters (in radians)
        const val maxAngVel: Double = Math.PI // shared with path
        const val maxAngAccel: Double = Math.PI

        // path controller gains
        const val axialGain: Double = 0.0
        const val lateralGain: Double = 0.0
        const val headingGain: Double = 0.0 // shared with turn

        const val axialVelGain: Double = 0.0
        const val lateralVelGain: Double = 0.0
        const val headingVelGain: Double = 0.0 // shared with turn; // shared with tur
    }
    
    override val localizer: Localizer = SwerveLocalizer(
        hardwareMap,
        PARAMS.modulePositions,
        PARAMS.inPerTick,
        startPose
    )

    override val controller: RobotPosVelController = HolonomicController(
        PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
        PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
    )

    val kinematics = SwerveKinematics(
        PARAMS.modulePositions
    )

    override val followerParams: FollowerParams = FollowerParams(
        ProfileParams(
            0.25, 0.1, 1e-2
        ),
        MinVelConstraint(
            listOf(
                WheelVelConstraint(kinematics, PARAMS.maxWheelVel),
                AngularVelConstraint(PARAMS.maxAngVel)
            )
        ),
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)
    )

    override val defaultTurnConstraints: TurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel
    )

    val feedforward: MotorFeedforward = MotorFeedforward(
        PARAMS.kS,
        PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
    )

    val modules: List<SwerveModule> = PARAMS.driveMotorNames.zip(PARAMS.steeringMotorNames).map {
        SwerveModule(hardwareMap, it.first, it.second)
    }

    override fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder = TrajectoryBuilder(
        TrajectoryBuilderParams(
            1e-6,
            followerParams.profileParams
        ),
        startPose, 0.0,
        defaultVelConstraint,
        defaultAccelConstraint
    )

    override fun setDrivePowers(powers: PoseVelocity2dDual<Time>) {
        val vels = kinematics.inverse(powers)

        vels.zip(modules).forEach { (vel, module) ->
            module.driveMotor.power = vel.first.value()
            module.steeringMotor.power = vel.second.value()
        }
    }

    override fun setDrivePowersWithFF(powers: PoseVelocity2dDual<Time>) {
        val vels = kinematics.inverse(powers)

        vels.zip(modules).forEach { (vel, module) ->
            module.driveMotor.power = feedforward.compute(vel.first)
            module.steeringMotor.power = vel.second.value()
        }
    }
}