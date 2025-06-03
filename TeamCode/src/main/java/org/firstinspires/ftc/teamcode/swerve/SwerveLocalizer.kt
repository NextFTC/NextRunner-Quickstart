package org.firstinspires.ftc.teamcode.swerve

import com.acmerobotics.roadrunner.ftc.*
import com.acmerobotics.roadrunner.geometry.*
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.MecanumDrive

class SwerveLocalizer(
    hardwareMap: HardwareMap,
    private val modulePositions: List<Vector2d>,
    private val inPerTick: Double,
    initialPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
) : Localizer {
    // Encoders for each swerve module's drive motor
    // Initialize drive encoders
    private val driveEncoders: List<Encoder> = SwerveDrive.PARAMS.driveMotorNames.map {
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, it)))
    }

    // Encoders for each swerve module's steering motor

    // Initialize steering encoders
    private val steeringEncoders: List<Encoder> = SwerveDrive.PARAMS.steeringMotorNames.map {
        OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, it)))
    }

    // IMU for heading tracking

    // Initialize IMU
    private val imu: LazyImu = LazyHardwareMapImu(
        hardwareMap, "imu", RevHubOrientationOnRobot(
            SwerveDrive.PARAMS.logoFacingDirection, SwerveDrive.PARAMS.usbFacingDirection
        )
    )

    // Kinematics for swerve drive calculations
    private val kinematics = SwerveKinematics(modulePositions)

    // Track last encoder positions for calculating deltas
    private var lastDrivePositions = mutableListOf<Int>()
    private var lastSteeringPositions = mutableListOf<Int>()

    // Track last heading for calculating delta
    private var lastHeading = Rotation2d.exp(0.0)

    // Current pose of the robot
    override var pose = initialPose

    // History of poses for debugging and visualization
    override val poseHistory = ArrayList<Pose2d>()

    // Flag to track if the localizer has been initialized
    private var initialized = false

    init {

        // Initialize last positions lists with zeros
        for (i in driveEncoders.indices) {
            lastDrivePositions.add(0)
            lastSteeringPositions.add(0)
        }
    }

    override fun update(): PoseVelocity2d {
        // Get current encoder positions and velocities
        val drivePositionsAndVelocities = driveEncoders.map { it.getPositionAndVelocity() }
        val steeringPositionsAndVelocities = steeringEncoders.map { it.getPositionAndVelocity() }

        // Get current heading from IMU
        val angles: YawPitchRollAngles = imu.get().robotYawPitchRollAngles
        val heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

        // Initialize on first update
        if (!initialized) {
            initialized = true

            // Store initial encoder positions
            for (i in driveEncoders.indices) {
                lastDrivePositions[i] = drivePositionsAndVelocities[i].position
                lastSteeringPositions[i] = steeringPositionsAndVelocities[i].position
            }

            lastHeading = heading

            return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
        }

        // Calculate heading delta
        val headingDelta = heading.minus(lastHeading)

        // Calculate wheel deltas and steering angles
        val wheelDeltas = mutableListOf<DualNum<Time>>()
        val steeringAngles = mutableListOf<DualNum<Time>>()

        for (i in driveEncoders.indices) {
            // Calculate drive wheel delta in ticks
            val driveDelta = drivePositionsAndVelocities[i].position - lastDrivePositions[i]
            val driveVelocity: Int = drivePositionsAndVelocities[i].velocity!!

            // Convert to inches
            wheelDeltas.add(
                DualNum(doubleArrayOf(
                    driveDelta.toDouble() * inPerTick,
                    driveVelocity.toDouble()* inPerTick
                ))
            )

            // Calculate steering angle in radians
            val steeringPos = steeringPositionsAndVelocities[i].position
            val steeringVel: Int = steeringPositionsAndVelocities[i].velocity!!

            // Convert encoder ticks to radians (adjust the conversion factor as needed)
            val ticksPerRadian = 1000.0 // Example value, adjust based on your hardware
            steeringAngles.add(
                DualNum(doubleArrayOf(
                    steeringPos.toDouble() / ticksPerRadian,
                    steeringVel.toDouble() / ticksPerRadian
                ))
            )

            // Update last positions
            lastDrivePositions[i] = drivePositionsAndVelocities[i].position
            lastSteeringPositions[i] = steeringPositionsAndVelocities[i].position
        }

        // Use swerve kinematics to calculate the twist
        val twist = kinematics.forward<Time>(
            SwerveKinematics.SwerveWheelIncrements(
                wheelDeltas,
                steeringAngles
            )
        )

        // Update last heading
        lastHeading = heading

        // Update pose using the calculated twist
        pose = pose.plus(
            Twist2d(
                twist.line.value(),
                headingDelta
            )
        )

        // Update pose history
        poseHistory.add(0, pose)
        if (poseHistory.size > 100) {
            poseHistory.removeAt(poseHistory.size - 1)
        }

        // Return the current velocity
        return twist.velocity().value()
    }
}
