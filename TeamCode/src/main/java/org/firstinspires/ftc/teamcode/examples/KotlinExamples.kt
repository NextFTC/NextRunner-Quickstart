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
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TimeFollower

@Autonomous
@Disabled
class CommandBuilderExampleKt() : NextFTCOpMode(MecanumDrive.INSTANCE) {
    //because the variable is not initialized upon declaration, we add the lateinit modifier
    //remember that we can't initialize it until the runOpMode method, as it relies on hardwareMap
    //which itself is not initialized until then
    lateinit var command: Command

    override fun onInit() {
        command = MecanumDrive.INSTANCE.commandBuilder()
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
class FollowerExampleKt() : NextFTCOpMode(MecanumDrive.INSTANCE) {
    private lateinit var follower: Follower

    override fun onInit() {
        val traj: Trajectory<Arclength> = MecanumDrive.INSTANCE.trajectoryBuilder()
            .forward(10.0)
            .splineTo(Vector2d(10.0, 10.0), Math.toRadians(90.0))
            .buildToComposite()

        follower = TimeFollower(traj, MecanumDrive.INSTANCE)
    }

    override fun onUpdate() {
        if (!follower.isDone) {
            follower.follow()
        }
    }
}