package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.*;

@Autonomous
public final class ManualFeedbackTuner extends NextFTCOpMode {
    public static double DISTANCE = 64;
    private boolean isMecanumDrive;
    private Command command;

    public ManualFeedbackTuner() {
        super(TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) ? 
              MecanumDrive.INSTANCE : TankDrive.INSTANCE);
        isMecanumDrive = TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class);
    }

    @Override
    public void onInit() {
        if (isMecanumDrive) {
            MecanumDrive drive = MecanumDrive.INSTANCE;

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }

            command = drive.commandBuilder(new Pose2d(0, 0, 0))
                    .lineToX(DISTANCE)
                    .lineToX(0)
                    .build();
        } else {
            TankDrive drive = TankDrive.INSTANCE;

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }

            command = drive.commandBuilder(new Pose2d(0, 0, 0))
                    .lineToX(DISTANCE)
                    .lineToX(0)
                    .build();
        }
    }

    @Override
    public void onUpdate() {
        if (!command.isDone()) {
            command.invoke();
        }
    }
}
