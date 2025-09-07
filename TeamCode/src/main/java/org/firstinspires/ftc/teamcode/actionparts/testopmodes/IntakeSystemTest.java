package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actionparts.IntakeSystem;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

@TeleOp(name="Intake System Test", group="Test")
public class IntakeSystemTest extends LinearOpMode {

    private IntakeSystem intakeSystem;
    private GearheadsMecanumRobotRR robot;
    @Override
    public void runOpMode() {
        robot = new GearheadsMecanumRobotRR(this);
        robot.initTeleOp(hardwareMap);

        // Create intake system
        intakeSystem = robot.intakesystem;

        // Optionally call initialize (if you add reset/brake logic later)
        intakeSystem.initialize();

        telemetry.addLine("Ready to run Intake Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Controls ---
            if (gamepad1.a) {
                intakeSystem.clawOpen(); // intake forward
            } else if (gamepad1.b) {
                intakeSystem.clawClose(); // intake reverse
            } else if (gamepad1.x) {
                intakeSystem.wristUp(); // stop intake
            } else if (gamepad1.y) {
                intakeSystem.wristDown(); // stop intake
            }

            // --- Telemetry ---
            //telemetry.addData("Motor Power", intakeSystem.wristMotor.getPower());
            //telemetry.update();
        }
    }
}
