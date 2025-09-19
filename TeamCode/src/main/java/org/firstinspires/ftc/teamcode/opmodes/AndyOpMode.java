package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

        /*
        The origin is the field perimeter corner by the red loading zone.
        We'll drive from the perspective of the red alliance:
         - Pushing up on the left stick moves toward the blue alliance wall, which is positive X.
         - Pushing to the left on the left stick moves toward the obelisk wall, which is positive Y.
         - Pushing to the left on the right stick rotates the the positive direction,
         counterclockwise
         */
        RunCommand teleopDriveCommand = new RunCommand(() -> drivetrainSubsystem.drive(
                -gamepad1.left_stick_y, // Stick up is negative but moves +X, so invert
                -gamepad1.left_stick_x, // Stick left is negative but moves +Y, so invert
                -gamepad1.right_stick_x, // Stick left is negative but moves +rotation, so invert
                1),
                drivetrainSubsystem);

        RunCommand telemetryCommand = new RunCommand(() -> {
            drivetrainSubsystem.telemetrize(telemetry);
            telemetry.update();
        });

        // Schedule commands
        schedule(telemetryCommand);

        // Register subsystems
        register(drivetrainSubsystem);

        // Set default commands for subsystems
        drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Test PedroPathing command
        Follower follower = drivetrainSubsystem.createFollower();

        // PathChain that sweeps across all of the spike marks for the red alliance
        Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(56.000, 36.000), new Pose(19.000, 36.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(new Pose(19.000, 36.000), new Pose(56.000, 60.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(new BezierLine(new Pose(56.000, 60.000), new Pose(19.000, 60.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(new Pose(19.000, 60.000), new Pose(56.000, 84.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(new BezierLine(new Pose(56.000, 84.000), new Pose(19.000, 84.000)))
                .setTangentHeadingInterpolation()
                .build();

        FollowPathCommand followPathCommand =
                new FollowPathCommand(startPose, follower, path, drivetrainSubsystem)
                        .withGlobalMaxPower(0.75);

        // Motif AprilTag
        MotifIndicatorCommand motifIndicatorCommand =
                new MotifIndicatorCommand(ledSubsystem, aprilTagSubsystem, telemetry);

        // Bind driver buttons
        GamepadEx gamepad = new GamepadEx(gamepad1);
        gamepad.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(motifIndicatorCommand);
        gamepad.getGamepadButton(GamepadKeys.Button.A).whenHeld(followPathCommand);
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> drivetrainSubsystem.resetLocalization());
    }

}
