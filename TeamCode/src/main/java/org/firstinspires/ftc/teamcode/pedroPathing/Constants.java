package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    private static final String FRONT_LEFT_MOTOR_NAME = "front_left_motor";
    private static final String BACK_LEFT_MOTOR_NAME = "back_left_motor";
    private static final String FRONT_RIGHT_MOTOR_NAME = "front_right_motor";
    private static final String BACK_RIGHT_MOTOR_NAME = "back_right_motor";
    private static final String OTOS_NAME = "sensor_otos";

    public static final OTOSConstants OTOS_CONSTANTS = new OTOSConstants()
            .hardwareMapName(OTOS_NAME)
            .linearUnit(DistanceUnit.METER)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(1.0)
            .angularScalar(1.0)
            .offset(new SparkFunOTOS.Pose2D(0.016, 0.09, Math.PI));

    public static final FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(5);

    public static final MecanumConstants DRIVE_CONSTANTS = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(FRONT_RIGHT_MOTOR_NAME)
            .rightRearMotorName(BACK_RIGHT_MOTOR_NAME)
            .leftRearMotorName(BACK_LEFT_MOTOR_NAME)
            .leftFrontMotorName(FRONT_LEFT_MOTOR_NAME)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static final PathConstraints PATH_CONSTRAINTS =
            new PathConstraints(0.99, 100, 1, 1);

}