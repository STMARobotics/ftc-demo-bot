package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drivetrain subsystem. Encapsulates the details of <i>how</i> the drivetrain works.
 */
public class DrivetrainSubsystem extends SubsystemBase {

    private static final String FRONT_LEFT_MOTOR_NAME = "front_left_motor";
    private static final String BACK_LEFT_MOTOR_NAME = "back_left_motor";
    private static final String FRONT_RIGHT_MOTOR_NAME = "front_right_motor";
    private static final String BACK_RIGHT_MOTOR_NAME = "back_right_motor";


    // Motors
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;

    private final IMU imu;

    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR_NAME);
        backLeftMotor = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR_NAME);
        backRightMotor = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR_NAME);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        resetLocalization();
    }

    /**
     * Drive the robot on field centric manner
     * @param translationX robot strafe along the X axis in range [-1, 1]. The X axis runs along the
     *                     field perimeter on the audience side. The robot is facing the positive
     *                     X direction when it has a heading of 0 radians (0°)
     * @param translationY robot speed along the Y axis in range [-1, 1]. The Y axis runs along the
     *                     field perimeter on the red alliance side. The robot is facing the
     *                     positive Y direction when it has a heading of 1/2 PI radians (90°).
     * @param rotation robot rotation speed in range of [-1, 1]. Counterclockwise positive
     * @param reductionFactor value to multiply the speed parameters by in range [0, 1]
     */
    public void drive(double translationX, double translationY, double rotation, double reductionFactor) {
        double clampedReduction = MathUtils.clamp(reductionFactor, 0.0, 1.0);

        // Square and reduce the axes
        double modifiedY = square(translationY * clampedReduction);
        double modifiedX = square(translationX * clampedReduction);
        double modifiedRotation = square(rotation * clampedReduction);

        // Rotate the heading based on the robot's heading on the field
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = modifiedX * Math.sin(botHeading) - modifiedY * Math.cos(botHeading);
        double rotY = modifiedX * Math.cos(botHeading) + modifiedY * Math.sin(botHeading);

        // Calculate the output for each wheel
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(modifiedRotation), 1);
        double frontLeftPower = (rotY + rotX - modifiedRotation) / denominator;
        double backLeftPower = (rotY - rotX - modifiedRotation) / denominator;
        double frontRightPower = (rotY - rotX + modifiedRotation) / denominator;
        double backRightPower = (rotY + rotX + modifiedRotation) / denominator;

        // Apply the output to the motors
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void stop() {
        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }

    public void resetLocalization() {
        imu.resetYaw();
    }

    public void telemetrize(Telemetry telemetry) {
    }

    public static double square(double value) {
        return Math.copySign(value * value, value);
    }

}
