package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2dDual;
import com.acmerobotics.roadrunner.geometry.PoseVelocity2d;
import com.acmerobotics.roadrunner.geometry.Time;
import com.acmerobotics.roadrunner.trajectories.TimeTurn;
import com.rowanmcalpin.nextftc.core.command.Command;
import org.jetbrains.annotations.NotNull;

import static com.acmerobotics.roadrunner.actions.Actions.now;
import static com.acmerobotics.roadrunner.ftc.Drawing.drawPoseHistory;
import static com.acmerobotics.roadrunner.ftc.Drawing.drawRobot;

class TurnCommand extends Command {
    @NotNull
    private final TimeTurn turn;
    @NotNull
    private final Drive drive;
    private double startTime;
    private boolean done = false;
    private final TelemetryPacket p = new TelemetryPacket();

    public TurnCommand(@NotNull TimeTurn turn, @NotNull Drive drive) {
        this.turn = turn;
        this.drive = drive;
    }

    @Override
    public void start() {
        startTime = now();
    }

    @Override
    public void update() {
        double t = now() - startTime;

        if (t > turn.duration) {
            done = true;
            return;
        }

        Pose2dDual<Time> target = turn.get(t);
        PoseVelocity2d robotVel = drive.getLocalizer().update();

        PoseVelocity2d command = drive.getController().compute(
                target,
                drive.getLocalizer().getPose(),
                robotVel
        ).value();

        drive.setDrivePowersWithFF(command);

        Canvas c = p.fieldOverlay();
        drawPoseHistory(drive, c);

        c.setStroke("#4CAF50");
        drawRobot(c, target.value());

        c.setStroke("#3F51B5");
        drawRobot(c, drive.getLocalizer().getPose());

        c.setStroke("#7C4DFFFF");
        c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0);
    }

    @Override
    public boolean isDone() {
        return done;
    }

    @Override
    public void stop(boolean interrupted) {
        drive.setDrivePowers(PoseVelocity2d.zero);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}


