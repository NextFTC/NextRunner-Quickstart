package org.firstinspires.ftc.teamcode.examples

import com.acmerobotics.roadrunner.actions.Action
import com.acmerobotics.roadrunner.ftc.Follower
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.acmerobotics.roadrunner.geometry.Arclength
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.paths.Line
import com.acmerobotics.roadrunner.paths.PosePath
import com.acmerobotics.roadrunner.paths.fromPoints
import com.acmerobotics.roadrunner.trajectories.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.DisplacementFollower
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TimeFollower

@Autonomous
@Disabled
class ActionBuilderExampleKt: LinearOpMode() {
    lateinit var drive: MecanumDrive
    //because the variable is not initialized upon declaration, we add the lateinit modifier
    //remember that we can't initialize it until the runOpMode method, as it relies on hardwareMap
    //which itself is not initialized until then
    lateinit var action: Action

    override fun runOpMode() {
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        action = drive.actionBuilder()
            .forward(10.0)
            .splineTo(Vector2d(10.0, 10.0), Math.toRadians(90.0))
            .build()

        waitForStart()

        runBlocking(action)
    }
}

//kotlin allows you to have multiple classes in one file!

@Autonomous
@Disabled
class FollowerExampleKt: OpMode() {
    private lateinit var drive: MecanumDrive
    private lateinit var follower: Follower

    override fun init() {
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val traj: Trajectory<Arclength> = drive.trajectoryBuilder()
            .forward(10.0)
            .splineTo(Vector2d(10.0, 10.0), Math.toRadians(90.0))
            .buildToComposite()

        follower = TimeFollower(traj, drive)
    }

    override fun loop() {
        if (!follower.isDone) {
            follower.follow()
        }
    }
}

@Autonomous
@Disabled
class PathObjectsExampleKt : OpMode() {
    private lateinit var drive: MecanumDrive
    private lateinit var follower: Follower

    override fun init() {
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        val path1: PosePath = Line(
            Vector2d(0.0, 0.0),
            Vector2d(10.0, 10.0)
        ).withTangentHeading()

        val path2: PosePath = fromPoints(
            Vector2d(10.0, 10.0),
            Vector2d(20.0, 20.0),
            Vector2d(10.0, 10.0)
        ).withLinearHeading(Math.PI / 4, 0.0)

        val trajectory = drive.createTrajectory(path1 + path2)

        follower = DisplacementFollower(trajectory, drive)
    }

    override fun loop() {
        if (!follower.isDone) {
            follower.follow()
        }
    }
}