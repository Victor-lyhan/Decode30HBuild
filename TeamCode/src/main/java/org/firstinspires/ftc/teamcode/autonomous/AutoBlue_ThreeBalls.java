package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Three Balls", group = "Auto")
public class AutoBlue_ThreeBalls extends AbstractAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
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
        //intakesystem.clawClose();

        /* Two ball
        api.forward(81, 0.3);
        sleep(500);
        api.turnLeft(45, 15);
        sleep(500);
        api.backward(22, 0.3);

        launchsystem.spinnersOn();
        sleep(1500);

        launchsystem.flapUp();
        sleep(500);
        launchsystem.flapDown();
        sleep(500);

        launchsystem.flapUp();
        sleep(500);
        launchsystem.flapDown();
        */

        intakesystem.clawClose();
        sleep(500);
        api.backward(80,0.3);
        sleep(500);
        api.turnRight(138, 5);
        sleep(500);
        api.backward(24, 0.3);

        launchsystem.spinnersOn();
        sleep(1500);

        launchsystem.flapUp();
        sleep(500);
        launchsystem.flapDown();
        sleep(1500);

        launchsystem.flapUp();
        sleep(500);
        launchsystem.flapDown();
        sleep(500);

        intakesystem.wristUp();
        sleep(500);
        intakesystem.clawOpen();
        sleep(1500);
        launchsystem.flapUp();
        sleep(500);

        launchsystem.spinnersOff();


        telemetry.addLine("Path complete");
        telemetry.update();
        sleep(1000);
    }
}

