package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.drivetrain.FollowPathCommand;
import org.firstinspires.ftc.teamcode.led.LedSubsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.vision.MotifIndicatorCommand;

@TeleOp(name="Andy", group = "Andy")
public class AndyOpMode extends CommandOpMode {

    private DrivetrainSubsystem drivetrainSubsystem;
    private LedSubsystem ledSubsystem;
    private AprilTagSubsystem aprilTagSubsystem;

    @Override
    public void initialize() {
        // Create subsystems
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap);
        ledSubsystem = new LedSubsystem(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);

        // Create commands
        RunCommand teleopDriveCommand = new RunCommand(() -> drivetrainSubsystem.drive(
                    -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 1),
                drivetrainSubsystem);

        RunCommand telemetryCommand = new RunCommand(() -> {
            drivetrainSubsystem.telemetrize(telemetry);
            telemetry.update();
        });

        MotifIndicatorCommand motifIndicatorCommand =
                new MotifIndicatorCommand(ledSubsystem, aprilTagSubsystem, telemetry);

        Follower follower = drivetrainSubsystem.createFollower(new PathConstraints(0, 0));
        Pose pose1 = new Pose(0,0,0);
        Pose pose2 = new Pose(1, 1, Math.PI);
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
        FollowPathCommand followPathCommand =
                new FollowPathCommand(follower, path, drivetrainSubsystem);

        // Schedule commands
        schedule(telemetryCommand, motifIndicatorCommand);

        // Register subsystems
        register(drivetrainSubsystem);

        // Set default commands for subsystems
        drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);

        GamepadEx gamepad = new GamepadEx(gamepad1);
        gamepad.getGamepadButton(GamepadKeys.Button.A).whileHeld(followPathCommand);
    }

}
