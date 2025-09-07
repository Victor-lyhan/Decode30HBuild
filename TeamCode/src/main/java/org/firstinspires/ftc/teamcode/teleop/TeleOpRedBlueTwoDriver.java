package org.firstinspires.ftc.teamcode.teleop;


import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ValueStorage;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.actionparts.*;

@TeleOp(name = "TeleOpTwoController", group = "TeleOp")
public class TeleOpRedBlueTwoDriver extends LinearOpMode {
    DcMotorEx fr;
    DcMotorEx fl;
    DcMotorEx br;
    DcMotorEx bl;

    BNO055IMU gyro;

    double initialHeading = ValueStorage.lastPose.heading.toDouble();
    double robotHeading;
    double joystickAngle;
    double joystickMagnitude;
    double turn;

    RisingEdgeDetector detector;

    /* Declare OpMode members. */
    GearheadsMecanumRobotRR robot;   // Use gearheads robot hardware

    //Different action systems used by the Robot
    private IntakeSystem intakesystem;
    private launchSystem launchsystem;
    @Override
    public void runOpMode() {
        robot = new GearheadsMecanumRobotRR(this);
        initOpMode();
        intakesystem = robot.intakesystem;
        launchsystem = robot.launchsystem;

        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro.initialize(parameters);

        detector = new RisingEdgeDetector();

        // waitForStart();

        //Stop motors to prevent accidental starts after autonomous
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        while (opModeIsActive() && !isStopRequested()) {
            moveRobot();
            operateIntake();
            operateOuttake();
        }
    }


    private void moveRobot() {
        if (gamepad1.ps) {
            initialHeading -= robotHeading;
        }
        robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + initialHeading;
        joystickAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        joystickMagnitude = pow(gamepad1.left_stick_x, 2) + pow(gamepad1.left_stick_y, 2);
        turn = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);

        if (gamepad1.right_trigger < 0.1) {
            fr.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) - turn, -1, 1));
            fl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) + turn, -1, 1));
            br.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) - turn, -1, 1));
            bl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) + turn, -1, 1));
        } else {//speed Damper if right trigger pressed.
            fr.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) - turn, -1, 1) / 3);
            fl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) + turn, -1, 1) / 3);
            br.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) - turn, -1, 1) / 3);
            bl.setPower(-Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) + turn, -1, 1) / 3);
        }
    }

    /**
     * Operate intake system
     */

    private void operateIntake() {
        if (gamepad2.a) {
            intakesystem.wristDown();
        }
        if (gamepad2.b) {
            intakesystem.clawOpen();
        }
        if (gamepad2.x) {
            intakesystem.clawClose();
        }
        if (gamepad2.y) {
            intakesystem.wristUp();
        }
    }

    private void operateOuttake() {
        if (gamepad2.left_bumper || gamepad2.right_bumper) {
            launchsystem.flapUp(); //TODO: add pause
            sleep(100);
            launchsystem.flapDown();
        }
        if (gamepad2.left_trigger > 0.7) {
            launchsystem.spinnersOn();
        } else if (gamepad2.left_trigger <= 0.7) {
            launchsystem.spinnersOff();
        }
    }


    /**
     * Initialize the opmode
     */

    private void initOpMode() {
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        telemetry.addData("Status", "Started");
        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initTeleOp(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}