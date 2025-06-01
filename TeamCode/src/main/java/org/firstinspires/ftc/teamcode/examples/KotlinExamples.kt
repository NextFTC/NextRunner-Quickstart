package org.firstinspires.ftc.teamcode.examples

import com.acmerobotics.roadrunner.ftc.Follower
import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectories.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode
import com.rowanmcalpin.nextftc.ftc.components.Components
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TimeFollower

@Autonomous
@Disabled
class CommandBuilderExampleKt(override val components: Components) : NextFTCOpMode() {
    lateinit var drive: MecanumDrive
    //because the variable is not initialized upon declaration, we add the lateinit modifier
    //remember that we can't initialize it until the runOpMode method, as it relies on hardwareMap
    //which itself is not initialized until then
    lateinit var command: Command

    override fun onInit() {
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        command = drive.commandBuilder()
            .forward(10.0)
            .splineTo(Vector2d(10.0, 10.0), Math.toRadians(90.0))
            .build()

    }

    override fun onStartButtonPressed() {
        command()
    }
}

//kotlin allows you to have multiple classes in one file!

@Autonomous
@Disabled
class FollowerExampleKt(override val components: Components) : NextFTCOpMode() {
    private lateinit var drive: MecanumDrive
    private lateinit var follower: Follower

    override fun onInit() {
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val traj: Trajectory<Arclength> = drive.trajectoryBuilder()
            .forward(10.0)
            .splineTo(Vector2d(10.0, 10.0), Math.toRadians(90.0))
            .buildToComposite()

        follower = TimeFollower(traj, drive)
    }

    override fun onUpdate() {
        if (!follower.isDone) {
            follower.follow()
        }
    }
}