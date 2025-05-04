package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.geometry.Math.range;
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
import com.acmerobotics.roadrunner.profiles.TimeProfile;
import com.acmerobotics.roadrunner.profiles.VelConstraint;
import com.acmerobotics.roadrunner.trajectories.TimeTrajectory;
import com.acmerobotics.roadrunner.trajectories.Trajectories;
import com.acmerobotics.roadrunner.trajectories.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class TimeFollower implements Follower {
    private final TimeTrajectory trajectory;
    private final Drive drive;

    public TimeFollower(Trajectory<?> trajectory, Drive drive) {
        this.trajectory = trajectory.wrtTime();
        this.drive = drive;
        this.currentTarget = trajectory.get(0.0).value();
        this.duration = this.trajectory.duration;
        this.timer = new ElapsedTime();
    }

    public TimeFollower(
            PosePath path,
            Drive drive,
            VelConstraint velConstraintOverride,
            AccelConstraint accelConstraintOverride) {
        this(new TimeTrajectory(
                        path,
                        new TimeProfile(
                                forwardProfile(
                                drive.getFollowerParams().profileParams,
                                path,
                                0.0,
                                velConstraintOverride,
                                accelConstraintOverride
                        ))
                ),
                drive);
    }
    public TimeFollower(PosePath path, Drive drive) {
        this(path, drive, drive.getDefaultVelConstraint(), drive.getDefaultAccelConstraint());
    }
    public TimeFollower(PosePath path, Drive drive, VelConstraint velConstraint) {
        this(path, drive, velConstraint, drive.getDefaultAccelConstraint());
    }
    public TimeFollower(PosePath path, Drive drive, AccelConstraint accelConstraint) {
        this(path, drive, drive.getDefaultVelConstraint(), accelConstraint);
    }

    private Pose2d currentTarget;
    private PoseVelocity2dDual<Time> lastCommand; // = PoseVelocity2dDual.zero();

    private boolean isDone = false;
    private final ElapsedTime timer;
    private final double duration;
    private boolean started = false;


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
        if (!started) {
            timer.reset();
            started = true;
        }

        double dt = timer.seconds();

        PoseVelocity2d robotVel = drive.getLocalizer().update();
        Pose2d robotPose = drive.getLocalizer().getPose();

        Twist2d error = Trajectories.getEndWrtTime(trajectory).value().minus(robotPose);
        if (dt >= duration || (error.line.norm() < 1.0 && error.angle < Math.toDegrees(5.0))) {
            isDone = true;
            return PoseVelocity2dDual.zero();
        }

        Pose2dDual<Time> target = trajectory.get(dt);
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
