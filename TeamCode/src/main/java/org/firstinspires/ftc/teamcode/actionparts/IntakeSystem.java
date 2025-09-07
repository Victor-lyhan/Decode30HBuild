package org.firstinspires.ftc.teamcode.actionparts;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class the represents the Intake System
 */
public class IntakeSystem {
    //DC motor used by the intake system
    public DcMotor wristMotor;
    public Servo clawServo;
    private Telemetry telemetry;
    private OpMode opmode;

    // TODO: Change parameters
    private static final double WIDE_POSITION = 0.97; //ball pick up, servo
    private static final double OPEN_POSITION = 0.76; //ball release
    private static final double CLOSED_POSITION = 0.69;
    private static final int UP_POSITION = 927; // wristMotor
    private static final int DOWN_POSITION = -2;


    public IntakeSystem(DcMotor wristMotor, Servo clawServo, OpMode opmode) {
        this.wristMotor = wristMotor;
        this.clawServo = clawServo;
        this.opmode = opmode;
        this.telemetry = opmode.telemetry;
    }

    /**
     * Initialize the system
     */
    public void initialize(){
        //Add code
//        clawWideOpen();
        wristDown();
    }

    public void clawWideOpen() {
        clawServo.setPosition(WIDE_POSITION);
//        telemetry.addData("status","claw open");
//        telemetry.update();
    }

    public void clawOpen() {
        clawServo.setPosition(OPEN_POSITION);
//        telemetry.addData("status","claw open");
//        telemetry.update();
    }


    public void clawClose() {
        clawServo.setPosition(CLOSED_POSITION);
//        telemetry.addData("status","claw close");
//        telemetry.update();
    }

    public void wristUp() {
        wristMotor.setTargetPosition(UP_POSITION);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(0.8); //TODO: Might need to change direction
//        telemetry.addData("status","wrist up");
//        telemetry.update();
    }

    public void wristDown() {
        wristMotor.setTargetPosition(DOWN_POSITION);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(0.8); //TODO: Might need to change direction
//        telemetry.addData("status","wrist down");
//        telemetry.update();
    }

    public void quickHit(){
        //TODO:If time, create auto pickup and drop routine
    }

    public void autoPickuponDetect(){
        //TODO:If time, create routine to auto-pickup based on detect
    }


}

