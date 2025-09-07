package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actionparts.IntakeSystem;
import org.firstinspires.ftc.teamcode.actionparts.launchSystem;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class GearheadsMecanumRobotRR {
    private Telemetry telemetry;
    //Different action systems used by the Robot
    public IntakeSystem intakesystem;
    public launchSystem launchsystem;

    //Gyro
    public BNO055IMU imu;


    private LinearOpMode curOpMode = null;   //current opmode

    /* local OpMode members. */
    public HardwareMap hwMap = null;

    /* Constructor */
    public GearheadsMecanumRobotRR(LinearOpMode opMode) {
        this.curOpMode = opMode;
        this.telemetry = curOpMode.telemetry;
        hwMap = opMode.hardwareMap;
    }


    /**
     * Initializes the intake system
     */
    private void initIntakeSystem() {
        DcMotor wristMotor = hwMap.get(DcMotor.class, "wristMotor");
        Servo clawServo = hwMap.get(Servo.class, "clawServo");

        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setDirection(DcMotor.Direction.REVERSE); //TODO: test this

        intakesystem = new IntakeSystem(wristMotor, clawServo, curOpMode);
        intakesystem.initialize();
    }

    /**
     * Initializes the launch system
     */
    private void initLaunchSystem() {
        DcMotor leftSpinnerMotor = hwMap.get(DcMotor.class, "leftSpinnerMotor");
        DcMotor rightSpinnerMotor = hwMap.get(DcMotor.class, "rightSpinnerMotor");
        Servo gateServo = hwMap.get(Servo.class, "gateServo");

        leftSpinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //TODO: check
        rightSpinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSpinnerMotor.setDirection(DcMotor.Direction.REVERSE); //TODO: test this
        rightSpinnerMotor.setDirection(DcMotor.Direction.FORWARD); //TODO: test this, reverse this if opposite

        launchsystem = new launchSystem(leftSpinnerMotor, rightSpinnerMotor, gateServo, curOpMode);
        launchsystem.initialize();
    }


    /**
     * Initializes the Gyro
     *
     * @param calibrate
     */
    private void initGyro(boolean calibrate) {
        imu = hwMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
        calibrate = false;
        if (calibrate) {
            curOpMode.telemetry.addData("Mode", "calibrating...");
            curOpMode.telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!curOpMode.isStopRequested() && !imu.isGyroCalibrated()) {
                curOpMode.sleep(10);
                curOpMode.idle();
            }
        }

        curOpMode.telemetry.addData("Mode", "waiting for start");
        curOpMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        curOpMode.telemetry.update();
    }


    /* Initialize standard Hardware interfaces */
    public void initAutonomous(HardwareMap ahwMap, String teamType) {
        init(ahwMap);
        initGyro(true);
    }

    /* Initialize standard Hardware interfaces */
    public void initTeleOp(HardwareMap ahwMap) {
        init(ahwMap);
        //initGyro(true);
    }

    private void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        initIntakeSystem();
        initLaunchSystem();
    }
}

