package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Drawing;
import com.acmerobotics.roadrunner.ftc.Drive;
import com.acmerobotics.roadrunner.ftc.Follower;
import com.acmerobotics.roadrunner.geometry.Geometry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d;
import com.acmerobotics.roadrunner.paths.PosePath;
import com.acmerobotics.roadrunner.profiles.AccelConstraint;
import com.acmerobotics.roadrunner.profiles.VelConstraint;
import com.acmerobotics.roadrunner.trajectories.Trajectory;
import com.rowanmcalpin.nextftc.core.command.Command;

import static com.acmerobotics.roadrunner.ftc.Drawing.drawRobot;

class FollowTrajectoryCommand extends Command {
    private final Follower follower;
    private final Drive drive;
    private final double[] xPoints;
    private final double[] yPoints;
    private final TelemetryPacket p = new TelemetryPacket();


    public FollowTrajectoryCommand(Follower follower, Drive drive) {
        this.follower = follower;
        this.drive = drive;
        Object[] xObjects = Geometry.xs(follower.getPoints()).toArray();
        Object[] yObjects = Geometry.ys(follower.getPoints()).toArray();

        xPoints = new double[xObjects.length];
        yPoints = new double[yObjects.length];
        for (int i = 0; i < xObjects.length; i++) {
            xPoints[i] = (double) xObjects[i];
            yPoints[i] = (double) yObjects[i];
        }
    }

    public FollowTrajectoryCommand(Trajectory<?> traj, Drive drive) {
        this(new DisplacementFollower(traj, drive), drive);
    }

    public FollowTrajectoryCommand(PosePath path, Drive drive, VelConstraint velConstraintOverride, AccelConstraint accelConstraintOverride) {
        this(new DisplacementFollower(path, drive, velConstraintOverride, accelConstraintOverride), drive);
    }

    @Override
    public void update() {
        follower.follow();

        p.put("x", drive.getLocalizer().getPose().position.x);
        p.put("y", drive.getLocalizer().getPose().position.y);
        p.put("heading (deg)", Math.toDegrees(drive.getLocalizer().getPose().heading.toDouble()));

        Pose2d error = follower.getCurrentTarget().minusExp(drive.getLocalizer().getPose());
        p.put("xError", error.position.x);
        p.put("yError", error.position.y);
        p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

        // only draw when active; only one drive Command should be active at a time
        Canvas c = p.fieldOverlay();
        Drawing.drawPoseHistory(drive, c);

        c.setStroke("#4CAF50");
        drawRobot(c, follower.getCurrentTarget());

        c.setStroke("#3F51B5");
        drawRobot(c, drive.getLocalizer().getPose());

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);
    }

    @Override
    public void stop(boolean interrupted) {
        drive.setDrivePowers(PoseVelocity2d.zero);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    @Override
    public boolean isDone() {
        return follower.isDone();
    }
}