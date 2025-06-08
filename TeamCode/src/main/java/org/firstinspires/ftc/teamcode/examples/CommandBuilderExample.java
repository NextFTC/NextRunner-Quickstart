package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
@Disabled
public class CommandBuilderExample extends NextFTCOpMode {
    Command command;

    public CommandBuilderExample() {
        super(MecanumDrive.INSTANCE);
    }

    @Override
    public void onInit() {
        command = MecanumDrive.INSTANCE.commandBuilder()
                .forward(10.0)
                .splineTo(new Vector2d(10.0, 10.0), Math.toRadians(90.0))
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        command.invoke();
    }
}
