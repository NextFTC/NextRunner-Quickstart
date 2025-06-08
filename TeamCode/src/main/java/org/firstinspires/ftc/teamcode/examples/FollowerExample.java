package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.roadrunner.ftc.Follower;
import com.acmerobotics.roadrunner.geometry.Arclength;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectories.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TimeFollower;
import org.jetbrains.annotations.NotNull;

@Autonomous
@Disabled
public class FollowerExample extends NextFTCOpMode {
    private Follower follower;

    public FollowerExample() {
        super(MecanumDrive.INSTANCE);
    }

    @Override
    public void onInit() {
        Trajectory<Arclength> traj = MecanumDrive.INSTANCE.trajectoryBuilder()
                .forward(10.0)
                .splineTo(new Vector2d(10.0, 10.0), Math.toRadians(90.0))
                .buildToComposite();

        follower = new TimeFollower(traj, MecanumDrive.INSTANCE);
    }

    @Override
    public void onUpdate() {
        if (!follower.isDone()) {
            follower.follow();
        }
    }
}
