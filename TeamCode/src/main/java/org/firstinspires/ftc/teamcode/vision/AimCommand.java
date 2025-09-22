package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drivetrain.DrivetrainSubsystem;

/**
 * Command to aim at an AprilTag detected by a HuskyLens.
 */
public class AimCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final HuskyLensSubsystem huskyLensSubsystem;
    private final int tagId;
    private final PIDController profiledPIDController = new PIDController(
            0.004, 0, 0);
    private final Telemetry telemetry;

    public AimCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            HuskyLensSubsystem huskyLensSubsystem,
            Telemetry telemetry,
            int tagId) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.huskyLensSubsystem = huskyLensSubsystem;
        this.telemetry = telemetry;
        this.tagId = tagId;

        addRequirements(drivetrainSubsystem, huskyLensSubsystem);
    }

    @Override
    public void initialize() {
        if (!huskyLensSubsystem.knock()) {
            telemetry.addLine("Problem communicating with " + huskyLensSubsystem.getDeviceName());
        }
        huskyLensSubsystem.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        profiledPIDController.reset();
        // 160 is the center X coordinate for HuskyLens
        // https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#6.2%20Coordinate%20System
        profiledPIDController.setSetPoint(160);
    }

    @Override
    public void execute() {
        HuskyLens.Block[] blocks = huskyLensSubsystem.getBlocks();
        HuskyLensSubsystem.telemetrize(telemetry, blocks);
        for (HuskyLens.Block block : blocks) {
            if (tagId == block.id) {
                // found the tag, try to put it in the center of the screen
                double rotation = profiledPIDController.calculate(block.x);
                drivetrainSubsystem.driveRobotCentric(0, 0, rotation);
                return;
            }
        }
        // No matching tag found
        drivetrainSubsystem.stop();
    }

    public boolean isFinished() {
        return profiledPIDController.atSetPoint();
    }
}
