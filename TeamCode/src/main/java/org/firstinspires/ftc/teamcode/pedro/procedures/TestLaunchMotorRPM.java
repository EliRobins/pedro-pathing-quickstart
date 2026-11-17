package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test Launch Motor RPM", group = "Tests")
public class TestLaunchMotorRPM extends OpMode {
    private static final double RPM_STEP = 100.0;
    private static final double MAX_RPM = 6000.0;

    private DcMotorEx launch1;
    private DcMotorEx launch2;

    private double launch1TargetRpm;
    private double launch2TargetRpm;

    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousY;
    private boolean previousA;
    private boolean previousRightBumper;
    private boolean previousLeftBumper;

    @Override
    public void init() {
        launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launch2 = hardwareMap.get(DcMotorEx.class, "launch2");

        // Reverse one motor so two opposing launcher wheels turn in the same
        // launching direction. Swap these if your mechanism spins backward.
        launch1.setDirection(DcMotorSimple.Direction.FORWARD);
        launch2.setDirection(DcMotorSimple.Direction.REVERSE);

        launch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopLaunchers();

        telemetry.addLine("Launcher RPM test ready");
        telemetry.addLine("Both target RPMs start at zero");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !previousDpadUp) {
            launch1TargetRpm = clampRpm(launch1TargetRpm + RPM_STEP);
        }
        if (gamepad1.dpad_down && !previousDpadDown) {
            launch1TargetRpm = clampRpm(launch1TargetRpm - RPM_STEP);
        }

        if (gamepad1.y && !previousY) {
            launch2TargetRpm = clampRpm(launch2TargetRpm + RPM_STEP);
        }
        if (gamepad1.a && !previousA) {
            launch2TargetRpm = clampRpm(launch2TargetRpm - RPM_STEP);
        }

        if (gamepad1.right_bumper && !previousRightBumper) {
            launch1TargetRpm = clampRpm(launch1TargetRpm + RPM_STEP);
            launch2TargetRpm = clampRpm(launch2TargetRpm + RPM_STEP);
        }
        if (gamepad1.left_bumper && !previousLeftBumper) {
            launch1TargetRpm = clampRpm(launch1TargetRpm - RPM_STEP);
            launch2TargetRpm = clampRpm(launch2TargetRpm - RPM_STEP);
        }

        // B is an immediate stop and does not require an edge press.
        if (gamepad1.b) {
            launch1TargetRpm = 0.0;
            launch2TargetRpm = 0.0;
        }

        setMotorRpm(launch1, launch1TargetRpm);
        setMotorRpm(launch2, launch2TargetRpm);

        double launch1CurrentRpm = getMotorRpm(launch1);
        double launch2CurrentRpm = getMotorRpm(launch2);

        telemetry.addData("Launch 1 target", "%.0f RPM", launch1TargetRpm);
        telemetry.addData("Launch 1 current", "%.0f RPM", launch1CurrentRpm);
        telemetry.addData("Launch 1 error", "%.0f RPM", launch1TargetRpm - launch1CurrentRpm);
        telemetry.addLine();
        telemetry.addData("Launch 2 target", "%.0f RPM", launch2TargetRpm);
        telemetry.addData("Launch 2 current", "%.0f RPM", launch2CurrentRpm);
        telemetry.addData("Launch 2 error", "%.0f RPM", launch2TargetRpm - launch2CurrentRpm);
        telemetry.addLine();
        telemetry.addLine("D-pad up/down: adjust launch1");
        telemetry.addLine("Y/A: adjust launch2");
        telemetry.addLine("Bumpers: adjust both | B: stop both");
        telemetry.update();

        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousY = gamepad1.y;
        previousA = gamepad1.a;
        previousRightBumper = gamepad1.right_bumper;
        previousLeftBumper = gamepad1.left_bumper;
    }

    private void setMotorRpm(DcMotorEx motor, double rpm) {
        double ticksPerSecond = rpm * motor.getMotorType().getTicksPerRev() / 60.0;
        motor.setVelocity(ticksPerSecond);
    }

    private double getMotorRpm(DcMotorEx motor) {
        return motor.getVelocity() * 60.0 / motor.getMotorType().getTicksPerRev();
    }

    private double clampRpm(double rpm) {
        return Math.max(0.0, Math.min(MAX_RPM, rpm));
    }

    private void stopLaunchers() {
        launch1TargetRpm = 0.0;
        launch2TargetRpm = 0.0;
        launch1.setPower(0.0);
        launch2.setPower(0.0);
    }

    @Override
    public void stop() {
        stopLaunchers();
    }
}
