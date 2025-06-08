package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous
public final class SplineTest extends NextFTCOpMode {
    private boolean isMecanumDrive;
    private Command command;
    private Pose2d beginPose = new Pose2d(0, 0, 0);

    public SplineTest() {
        super(TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) ? 
              MecanumDrive.INSTANCE : TankDrive.INSTANCE);
        isMecanumDrive = TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class);
    }

    @Override
    public void onInit() {
        if (isMecanumDrive) {
            MecanumDrive drive = MecanumDrive.INSTANCE;
            drive.setPose(beginPose);

            command = drive.commandBuilder(beginPose)
                    .splineTo(new Vector2d(30, 30), Math.PI / 2)
                    .splineTo(new Vector2d(0, 60), Math.PI)
                    .build();
        } else {
            TankDrive drive = TankDrive.INSTANCE;
            drive.getLocalizer().setPose(beginPose);

            command = drive.commandBuilder(beginPose)
                    .splineTo(new Vector2d(30, 30), Math.PI / 2)
                    .splineTo(new Vector2d(0, 60), Math.PI)
                    .build();
        }
    }

    @Override
    public void onStartButtonPressed() {
        command.invoke();
    }
}
