package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Drawing.drawRobot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.actions.Action;
import com.acmerobotics.roadrunner.ftc.Drawing;
import com.acmerobotics.roadrunner.ftc.Drive;
import com.acmerobotics.roadrunner.ftc.Follower;
import com.acmerobotics.roadrunner.geometry.Geometry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.paths.PosePath;
import com.acmerobotics.roadrunner.profiles.AccelConstraint;
import com.acmerobotics.roadrunner.profiles.VelConstraint;
import com.acmerobotics.roadrunner.trajectories.Trajectory;

class FollowTrajectoryAction implements Action {
    private final Follower follower;
    private final Drive drive;
    private final double[] xPoints;
    private final double[] yPoints;

    public FollowTrajectoryAction(Follower follower, Drive drive) {
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

    public FollowTrajectoryAction(Trajectory<?> traj, Drive drive) {
        this(new DisplacementFollower(traj, drive), drive);
    }

    public FollowTrajectoryAction(PosePath path, Drive drive, VelConstraint velConstraintOverride, AccelConstraint accelConstraintOverride) {
        this(new DisplacementFollower(path, drive, velConstraintOverride, accelConstraintOverride), drive);
    }

    @Override
    public boolean run(TelemetryPacket p) {
        follower.follow();

        p.put("x", drive.getLocalizer().getPose().position.x);
        p.put("y", drive.getLocalizer().getPose().position.y);
        p.put("heading (deg)", Math.toDegrees(drive.getLocalizer().getPose().heading.toDouble()));

        Pose2d error = follower.getCurrentTarget().minusExp(drive.getLocalizer().getPose());
        p.put("xError", error.position.x);
        p.put("yError", error.position.y);
        p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

        // only draw when active; only one drive action should be active at a time
        Canvas c = p.fieldOverlay();
        Drawing.drawPoseHistory(drive, c);

        c.setStroke("#4CAF50");
        drawRobot(c, follower.getCurrentTarget());

        c.setStroke("#3F51B5");
        drawRobot(c, drive.getLocalizer().getPose());

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);

        return follower.isDone();
    }
}