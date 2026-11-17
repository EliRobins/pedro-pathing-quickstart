package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Turn Power Test")
public class TurnPowerTest extends OpMode {
    private Mecanum drivetrain;
    private double power = 0.02;

    private boolean previousUp;
    private boolean previousDown;

    @Override
    public void init() {
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !previousUp) {
            power = Math.min(1.0, power + 0.01);
        }

        if (gamepad1.dpad_down && !previousDown) {
            power = Math.max(0.0, power - 0.01);
        }

        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;

        if (gamepad1.right_bumper) {
            drivetrain.drive(new DrivePowers(0, 0, power), false);
        } else if (gamepad1.left_bumper) {
            drivetrain.drive(new DrivePowers(0, 0, -power), false);
        } else {
            drivetrain.stop();
        }

        telemetry.addData("Turn power", "%.2f", power);
        telemetry.addLine("D-pad: adjust power");
        telemetry.addLine("Bumpers: test each direction");
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}