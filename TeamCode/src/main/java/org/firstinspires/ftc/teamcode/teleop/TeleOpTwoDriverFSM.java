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
import org.firstinspires.ftc.teamcode.actionparts.IntakeSystem;
import org.firstinspires.ftc.teamcode.actionparts.launchSystem;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

@TeleOp(name = "Main-TeleOpTwoController-FSM1", group = "TeleOp")
public class TeleOpTwoDriverFSM extends LinearOpMode {
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
    private final TimedStateMachine intakeFSM = new TimedStateMachine();
    private final TimedStateMachine launchFSM = new TimedStateMachine();

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

        //Stop motors to prevent accidental starts after autonomous
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        setupIntakeFSM();
        setupLaunchFSM();

        boolean prevX = false;
        boolean prevY = false;

        while (opModeIsActive() && !isStopRequested()) {

            moveRobot();
            // ADDED: FSM controls — X to start, Back to cancel
            boolean xPressed = gamepad2.x && !prevX; // rising edge

            if (gamepad2.back) {
                intakeFSM.cancel();
                stopSubsystems();
            }

            if (xPressed) {
                intakeFSM.onTrigger();   // <- exactly matches your rules
            }
            prevX = gamepad2.x;

            intakeFSM.update();

            telemetry.addData("FSM", intakeFSM.isRunning() ? "RUN" : "IDLE");
            telemetry.addData("Step", "%d / %d", intakeFSM.currentIndex(), intakeFSM.size());
            telemetry.addData("Awaiting Advance", intakeFSM.isAwaitingAdvance());
            telemetry.update();


            // ADDED: FSM controls — B to start, Back to cancel
            boolean yPressed = gamepad2.y && !prevY; // rising edge

            if (gamepad2.back) {
                launchFSM.cancel();
                stopSubsystems();
            }

            if (yPressed) {
                launchFSM.onTrigger();   // <- exactly matches your rules
            }
            prevY = gamepad2.y;

            launchFSM.update();

            telemetry.addData("FSM", launchFSM.isRunning() ? "RUN" : "IDLE");
            telemetry.addData("Step", "%d / %d", launchFSM.currentIndex(), launchFSM.size());
            telemetry.addData("Awaiting Advance", launchFSM.isAwaitingAdvance());
            telemetry.update();

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

    // ADDED: One method to define the 4 states you specified
    private void setupIntakeFSM() {
        intakeFSM.clear()
            /**
             * Ready to Grab
             * intake claw: open
             * intake wrist: down
             * outtake flap: down
             * outtake spinners: off
             */
            .add(new TimedStateMachine.Step()
                .onStart(() -> {
                    if (intakesystem != null) {
                        intakesystem.wristDown();
                        intakesystem.clawWideOpen();
                    }
                })
                .maxDurationMS(500)
                .onStop(() -> {
                    if (launchsystem != null) {
//                        launchsystem.flapDown();
//                        launchsystem.spinnersOff();
                    }
                })
            )
            /**
             * grab
             */
            .add(new TimedStateMachine.Step()
                .onStart(() -> {
                    if (intakesystem != null) {
                        intakesystem.clawClose();
                    }
                })
                .maxDurationMS(300)
                .onStop(() -> {})
            )

            /**
             *3. Transition
             * intake wrist: up
             * intake claw: open
             */
            .add(new TimedStateMachine.Step()
                .onStart(() -> {
                    if (intakesystem != null) {
                        intakesystem.wristUp();
                    }
                })
                .maxDurationMS(700)
                .onStop(() -> {
                    if (intakesystem != null) {
                       intakesystem.clawOpen();
                    }
                })
            );
    }

    private void setupLaunchFSM() {
        launchFSM.clear()
                /**
                 * shoot first
                 * outtake spinners: on
                 * wait: 1000
                 * outtake flap: up
                 */
                .add(new TimedStateMachine.Step()
                        .onStart(() -> {
                            if (launchsystem != null) {
                                launchsystem.spinnersOn();
                            }
                        })
                        .maxDurationMS(1000)
                        .onStop(() -> {
                            if (launchsystem != null) {
                                launchsystem.flapUp();
                            }
                        })
                )
                /**
                 * 2. load claw ball
                 * outtake flap: down
                 * wait 500
                 * gamepad2.x
                 */
                .add(new TimedStateMachine.Step()
                        .onStart(() -> {
                            if (launchsystem != null) {
                                launchsystem.flapDown();
                            }
                        })
                        .maxDurationMS(500)
                        .onStop(() -> {
                            intakeFSM.onTrigger();
                        })
                )

                /**
                 * 3.  shoot second
                 * outtake flap: up
                 * wait 1000
                 * outtake flap: down
                 */
                .add(new TimedStateMachine.Step()
                        .onStart(() -> {
                            if (launchsystem != null) {
                                launchsystem.flapUp();
                            }
                        })
                        .maxDurationMS(500)
                        .onStop(() -> {
                            if (launchsystem != null) {
                                launchsystem.flapDown();
                            }
                        })
                )

                /**
                 * 4. shoot third
                 * a. outtake flap: up
                 * b. wait: 1000
                 * outtake flap: down
                 * outtake spinners: off
                 */
                .add(new TimedStateMachine.Step()
                        .onStart(() -> {
                            if (launchsystem != null) {
                                launchsystem.flapUp();
                            }
                        })
                        .maxDurationMS(1000)
                        .onStop(() -> {
                            if (launchsystem != null) {
                                launchsystem.spinnersOff();
                                launchsystem.flapDown();
                            }
                        })
                );
    }

    // Helper to stop all subsystems when cancelling FSM
    private void stopSubsystems() {
//        if (intakesystem != null) intakesystem.stopInTake();
//        if (elevator != null) elevator.stop();
    }
}