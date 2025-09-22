package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A subsystem for interacting with the HuskyLens.
 */
public class HuskyLensSubsystem extends SubsystemBase {

    private final HuskyLens huskyLens;

    public HuskyLensSubsystem(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
    }

    public void selectAlgorithm(HuskyLens.Algorithm algorithm) {
        huskyLens.selectAlgorithm(algorithm);
    }

    /**
     * Pings the HuskyLens to check if it's up.
     * @return result of the knock command
     */
    public boolean knock() {
        return huskyLens.knock();
    }

    /**
     * Gets the device name
     * @return device name
     */
    public String getDeviceName() {
        return huskyLens.getDeviceName();
    }

    /**
     * Gets the blocks from the HuskyLens.
     * @return array of blocks
     */
    public HuskyLens.Block[] getBlocks() {
        return huskyLens.blocks();
    }

    /**
     * Logs block info to telemetry
     * @param telemetry telemetry to write to
     * @param blocks blocks to log
     */
    public static void telemetrize(Telemetry telemetry, HuskyLens.Block[] blocks) {
        telemetry.addData("Block count", blocks.length);
        for (HuskyLens.Block block : blocks) {
            telemetry.addData("Block", block.toString());
        }
    }

}
