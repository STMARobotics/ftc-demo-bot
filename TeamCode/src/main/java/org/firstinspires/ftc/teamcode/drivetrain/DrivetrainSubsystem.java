package org.firstinspires.ftc.teamcode.drivetrain;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Drivetrain subsystem. Encapsulates the details of <i>how</i> the drivetrain works.
 */
public class DrivetrainSubsystem extends SubsystemBase {

    // Motor name constants
    private static final String FRONT_LEFT_MOTOR_NAME = "front_left_motor";
    private static final String BACK_LEFT_MOTOR_NAME = "back_left_motor";
    private static final String FRONT_RIGHT_MOTOR_NAME = "front_right_motor";
    private static final String BACK_RIGHT_MOTOR_NAME = "back_right_motor";

    // OTOS sensor constants
    private static final String OTOS_SENSOR_NAME = "sensor_otos";
    private static final double OTOS_LINEAR_SCALAR = 1.0;
    private static final double OTOS_ANGULAR_SCALAR = 1.0;

    private final HardwareMap hardwareMap;

    private final Motor frontLeftMotor;
    private final Motor backLeftMotor;
    private final Motor frontRightMotor;
    private final Motor backRightMotor;

    private final SparkFunOTOS myOtos;

    // PedroPathing
    private final static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5);
    private final static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(FRONT_RIGHT_MOTOR_NAME)
            .rightRearMotorName(BACK_RIGHT_MOTOR_NAME)
            .leftRearMotorName(BACK_LEFT_MOTOR_NAME)
            .leftFrontMotorName(FRONT_LEFT_MOTOR_NAME)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    private final static OTOSConstants otosLocalizer = new OTOSConstants()
            .hardwareMapName(OTOS_SENSOR_NAME)
            .linearUnit(DistanceUnit.METER)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(OTOS_LINEAR_SCALAR)
            .angularScalar(OTOS_ANGULAR_SCALAR);

    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        frontLeftMotor = new Motor(hardwareMap, FRONT_LEFT_MOTOR_NAME);
        backLeftMotor = new Motor(hardwareMap, BACK_LEFT_MOTOR_NAME);
        frontRightMotor = new Motor(hardwareMap, FRONT_RIGHT_MOTOR_NAME);
        backRightMotor = new Motor(hardwareMap, BACK_RIGHT_MOTOR_NAME);

        frontLeftMotor.setInverted(true);
        backLeftMotor.setInverted(true);

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        myOtos = hardwareMap.get(SparkFunOTOS.class, OTOS_SENSOR_NAME);

        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.09, -0.16, Math.PI);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(OTOS_LINEAR_SCALAR);
        myOtos.setAngularScalar(OTOS_ANGULAR_SCALAR);
        myOtos.calibrateImu();
        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

    }

    /**
     * Drive the robot on field centric manner
     * @param translationY robot forward/back speed in range of [-1, 1]. Forward positive
     * @param translationX robot strafe speed in range [-1, 1]. Right positive
     * @param rotation robot rotation speed in range of [-1, 1]. Counterclockwise positive
     * @param reductionFactor value to multiply the speed parameters by in range [0, 1]
     */
    public void drive(double translationY, double translationX, double rotation, double reductionFactor) {
        reductionFactor = MathUtils.clamp(reductionFactor, 0.0, 1.0);

        translationY *= reductionFactor;
        translationX *= reductionFactor;
        rotation *= reductionFactor;

        double botHeading = myOtos.getPosition().h;
        double rotX = translationX * Math.cos(-botHeading) - translationY * Math.sin(-botHeading);
        double rotY = translationX * Math.sin(-botHeading) + translationY * Math.cos(-botHeading);

        rotX *= 1.1;

        double denominator = calculateDenominator(rotX, rotY, rotation);
        double frontLeftPower = (rotY + rotX + rotation) / denominator;
        double backLeftPower = (rotY - rotX + rotation) / denominator;
        double frontRightPower = (rotY - rotX - rotation) / denominator;
        double backRightPower = (rotY + rotX - rotation) / denominator;

        frontLeftMotor.set(frontLeftPower);
        backLeftMotor.set(backLeftPower);
        frontRightMotor.set(frontRightPower);
        backRightMotor.set(backRightPower);
    }

    /**
     * Creates a PedroPath Follower.
     * @param pathConstraints PedroPath path constraints
     * @return new follower
     */
    public Follower createFollower(PathConstraints pathConstraints) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .OTOSLocalizer(otosLocalizer)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    private double calculateDenominator(double rotX, double rotY, double rx) {
        double sum = Math.abs(rotX) + Math.abs(rotY) + Math.abs(rx);
        return sum > 1 ? sum : 1;
    }

    /**
     * Adds drivetrain telemetry data.
     * @param telemetry telemetry object
     */
    public void telemetrize(Telemetry telemetry) {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition(); // Is this a blocking call that I should get once in periodic()?

        // Inform user of available controls
        telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
        telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
        telemetry.addLine();

        // Log the position to the telemetry
        telemetry.addData("X coordinate (meters)", pos.x);
        telemetry.addData("Y coordinate (meters)", pos.y);
        telemetry.addData("Heading angle (radians)", pos.h);
    }
}
