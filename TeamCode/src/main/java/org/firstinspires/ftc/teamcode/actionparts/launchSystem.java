package org.firstinspires.ftc.teamcode.actionparts;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class the represents the Intake System
 */
public class launchSystem {
    //DC motor used by the intake system
    private Telemetry telemetry;
    private OpMode opmode;
    public DcMotor leftSpinnerMotor;
    public DcMotor rightSpinnerMotor;
    public Servo gateServo;

    // TODO: Change parameters
    private static final double UP_POSITION = 0.36;
    private static final double DOWN_POSITION = 0.48;


    public launchSystem(DcMotor leftSpinnerMotorMotor, DcMotor rightSpinnerMotor, Servo gateServo, OpMode opmode) {
        this.leftSpinnerMotor = leftSpinnerMotorMotor;
        this.rightSpinnerMotor = rightSpinnerMotor;
        this.gateServo = gateServo;
        this.opmode = opmode;
        this.telemetry = opmode.telemetry;
    }

    /**
     * Initialize the system
     */
    public void initialize(){
        //Add code
        spinnersOff();
        flapDown();
    }

    public void flapUp() {
        gateServo.setPosition(UP_POSITION);
//        telemetry.addData("status","gate up");
//        telemetry.update();
    }

    public void flapDown() {
        gateServo.setPosition(DOWN_POSITION);
//        telemetry.addData("status","gate down");
//        telemetry.update();
    }

    public void spinnersOn() {
        leftSpinnerMotor.setPower(-0.5); //TODO: Might need to change direction
        rightSpinnerMotor.setPower(-0.5);
//        telemetry.addData("status","spinners on");
//        telemetry.update();
    }

    public void spinnersOff() {
        leftSpinnerMotor.setPower(0);
        rightSpinnerMotor.setPower(0);
//        telemetry.addData("status","spinners off");
//        telemetry.update();
    }
}