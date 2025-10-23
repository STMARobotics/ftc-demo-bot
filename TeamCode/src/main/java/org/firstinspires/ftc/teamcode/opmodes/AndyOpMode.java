package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainSubsystem;

@TeleOp(name="Drive Robot", group = "Drive")
public class AndyOpMode extends CommandOpMode {

    private DrivetrainSubsystem drivetrainSubsystem;

    @Override
    public void initialize() {
        // Create subsystems
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap);

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
        // Bind driver buttons
        GamepadEx gamepad = new GamepadEx(gamepad1);
        gamepad.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> drivetrainSubsystem.resetLocalization());
    }

}
