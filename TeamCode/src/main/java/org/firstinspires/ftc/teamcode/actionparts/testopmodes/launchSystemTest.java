package org.firstinspires.ftc.teamcode.actionparts.testopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actionparts.launchSystem;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

@TeleOp(name="Launch System Test", group="Test")
public class launchSystemTest extends LinearOpMode {

    private launchSystem launchSystem;
    private GearheadsMecanumRobotRR robot;
    @Override
    public void runOpMode() {
        robot = new GearheadsMecanumRobotRR(this);
        robot.initTeleOp(hardwareMap);

        // Create intake system
        launchSystem = robot.launchsystem;

        // Optionally call initialize (if you add reset/brake logic later)
        launchSystem.initialize();

        telemetry.addLine("Ready to run Intake Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Controls ---
            if (gamepad1.a) {
                launchSystem.flapDown(); // intake forward
            } else if (gamepad1.b) {
                launchSystem.flapUp(); // intake reverse
            } else if (gamepad1.x) {
                launchSystem.spinnersOn(); // stop intake
            } else if (gamepad1.y) {
                launchSystem.spinnersOff(); // stop intake
            }

            // --- Telemetry ---
            //telemetry.addData("Motor Power", launchSystem.wristMotor.getPower());
            //telemetry.update();
        }
    }
}
