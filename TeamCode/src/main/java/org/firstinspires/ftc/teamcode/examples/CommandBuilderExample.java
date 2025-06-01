package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.components.Components;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.jetbrains.annotations.NotNull;

@Autonomous
@Disabled
public class CommandBuilderExample extends NextFTCOpMode {
    Components components;
    MecanumDrive drive;
    Command command;

    public CommandBuilderExample(Components components) {
        this.components = components;
    }

    @Override
    public void onInit() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        command = drive.commandBuilder()
                .forward(10.0)
                .splineTo(new Vector2d(10.0, 10.0), Math.toRadians(90.0))
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        command.schedule();
    }

    @NotNull
    @Override
    public Components getComponents() {
        return components;
    }
}
