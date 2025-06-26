package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.roadrunner.ftc.Follower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.paths.BezierCurves;
import com.acmerobotics.roadrunner.paths.Line;
import com.acmerobotics.roadrunner.paths.PosePath;
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.DisplacementFollower;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Disabled
public class PathObjectsExample extends OpMode {
    private MecanumDrive drive;
    private Follower follower;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

        PosePath path1 = new Line(
                new Vector2d(0.0, 0.0),
                new Vector2d(10.0, 10.0)
        ).withTangentHeading();

        PosePath path2 = BezierCurves.fromPoints(
                new Vector2d(10.0, 10.0),
                new Vector2d(20.0, 20.0),
                new Vector2d(10.0, 10.0)
        ).withLinearHeading(Math.PI/4, 0.0);

        DisplacementTrajectory trajectory = drive.createTrajectory(path1.plus(path2));

        follower = new DisplacementFollower(trajectory, drive);
    }

    @Override
    public void loop() {
        if (!follower.isDone()) {
            follower.follow();
        }
    }
}
