package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Blue", group = "Auto")
public class AutoBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Set hub orientation (edit to match your Control Hub mounting!)
        RevHubOrientationOnRobot hubOrientation = MecanumAutoAPI.defaultREVHubOrientation();

        // Create the API. Replace motor names with your config names.
        MecanumAutoAPI.Params p = new MecanumAutoAPI.Params();
        // Example: if you have 96mm wheels -> 3.78in
        // p.wheelDiameterIn = 3.78;
        // p.ticksPerMotorRev = 383.6; // if using 435RPM Yellow Jacket
        // p.lateralMultiplier = 1.16; // tune this on your field

        MecanumAutoAPI api = new MecanumAutoAPI(
                this, hardwareMap,
                "front_left", "front_right", "back_left", "back_right",
                hubOrientation,
                p
        );

        // Show calibration numbers (CPI, geometry, etc.) on DS before start
        api.logCalibrationTelemetry();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // Path
        api.forward(81, 0.3);
        sleep(500);
        api.turnLeft(45, 15);
        sleep(500);
        api.backward(25, 0.3);

        telemetry.addLine("Path complete");
        telemetry.update();
        sleep(1000);
    }
}


