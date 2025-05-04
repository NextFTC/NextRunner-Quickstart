package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.profiles.Profiles.forwardProfile;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.Drive;
import com.acmerobotics.roadrunner.ftc.Follower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dDual;
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d;
import com.acmerobotics.roadrunner.geometry.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.geometry.Time;
import com.acmerobotics.roadrunner.geometry.Twist2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.paths.PosePath;
import com.acmerobotics.roadrunner.profiles.AccelConstraint;
import com.acmerobotics.roadrunner.profiles.VelConstraint;
import com.acmerobotics.roadrunner.trajectories.DisplacementTrajectory;
import com.acmerobotics.roadrunner.trajectories.Trajectory;
import static com.acmerobotics.roadrunner.geometry.Math.range;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class DisplacementFollower implements Follower {
    private final DisplacementTrajectory trajectory;
    private final Drive drive;

    public DisplacementFollower(Trajectory<?> trajectory, Drive drive) {
        this.trajectory = trajectory.wrtDisp();
        this.drive = drive;
        this.currentTarget = trajectory.get(0.0).value();
    }

    public DisplacementFollower(
            PosePath path,
            Drive drive,
            VelConstraint velConstraintOverride,
            AccelConstraint accelConstraintOverride) {
        this(new DisplacementTrajectory(
                        path,
                        forwardProfile(
                                drive.getFollowerParams().profileParams,
                                path,
                                0.0,
                                velConstraintOverride,
                                accelConstraintOverride
                        )
                ),
                drive);
    }
    public DisplacementFollower(PosePath path, Drive drive) {
        this(path, drive, drive.getDefaultVelConstraint(), drive.getDefaultAccelConstraint());
    }
    public DisplacementFollower(PosePath path, Drive drive, VelConstraint velConstraint) {
        this(path, drive, velConstraint, drive.getDefaultAccelConstraint());
    }
    public DisplacementFollower(PosePath path, Drive drive, AccelConstraint accelConstraint) {
        this(path, drive, drive.getDefaultVelConstraint(), accelConstraint);
    }

    private Pose2d currentTarget;
    private PoseVelocity2dDual<Time> lastCommand; // = PoseVelocity2dDual.zero();

    private boolean isDone = false;
    private double ds = 0.0;


    @NonNull
    @Override
    public Pose2d getCurrentTarget() {
        return currentTarget;
    }

    @NonNull
    @Override
    public PoseVelocity2dDual<Time> getLastCommand() {
        return lastCommand;
    }

    @Override
    public boolean isDone() {
        return isDone;
    }

    @NonNull
    @Override
    public List<Vector2d> getPoints() {
        List<Vector2d> points = new ArrayList<>();
        List<Double> samples = range(0.0,
                trajectory.length(),
                Math.max(2, (int) Math.ceil(trajectory.length() / 2))
        );
        for (int i = 0; i < samples.size(); i++) {
            points.add(trajectory.get(samples.get(i)).value().position);
        }
        return points;
    }

    public PoseVelocity2dDual<Time> getDriveCommand() {
        PoseVelocity2d robotVel = drive.getLocalizer().update();
        Pose2d robotPose = drive.getLocalizer().getPose();

        ds = trajectory.project(robotPose.position, ds);

        Twist2d error = trajectory.get(trajectory.length()).value().minus(robotPose);
        if (ds >= trajectory.length() || (error.line.norm() < 1.0 && error.angle < Math.toDegrees(5.0))) {
            isDone = true;
            return PoseVelocity2dDual.zero();
        }

        Pose2dDual<Time> target = trajectory.get(ds);
        currentTarget = target.value();

        return drive.getController().compute(
                target,
                robotPose,
                robotVel
        );
    }

    @Override
    public void follow() {
        lastCommand = getDriveCommand();
        drive.setDrivePowersWithFF(lastCommand);
    }
}
