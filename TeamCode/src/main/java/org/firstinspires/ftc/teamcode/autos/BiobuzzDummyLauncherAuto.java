package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// Uncomment these imports when the intake and launcher are installed.
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Dummy-test BIOBUZZ autonomous using Pedro Pathing 3 and Ivy.
 *
 * Sequence:
 * 1. Drive to a HIVE launch position and simulate launching the 4 preloaded POLLEN.
 * 2. Drive to a GARDEN pickup position while simulating intake.
 * 3. Return to the launch position and simulate a second launch.
 * 4. Return to the alliance LOADING ZONE for AUTO PARK.
 *
 * All mechanism calls are commented out so this OpMode can run on a drivetrain-only
 * test robot. The waits remain active to represent realistic mechanism timing.
 *
 * This is a compact test route, not a measured competition-field route. Tune every
 * pose and mechanism value on your finished robot before using it in a match.
 */
@Autonomous(name = "BIOBUZZ Dummy: Shoot + Intake + Park", group = "BIOBUZZ")
public class BiobuzzDummyLauncherAuto extends OpMode {
    private static final double LAUNCHER_SPIN_UP_MS = 1500;
    private static final double POLLEN_FEED_MS = 2500;
    private static final double INTAKE_COLLECTION_MS = 1800;
    private static final double MECHANISM_SETTLE_MS = 250;
    private static final double AUTO_STOP_SECONDS = 29.0;

    private final PoseFactory poses = PoseFactory.degrees();

    // Local test coordinates: +X is forward and +Y is robot-left.
    private final Pose startPose = poses.of(0, 0, 0);
    private final Pose launchPose = poses.of(36, 0, 0);
    private final Pose pickupPose = poses.of(36, 24, 0);
    private final Pose parkControlPose = poses.of(24, 24, 0);
    private final Pose parkPose = poses.of(0, 24, 0);

    private final ElapsedTime autoTimer = new ElapsedTime();

    private Follower follower;
    private String currentAction = "Waiting for start";
    private boolean routineComplete;

    // Uncomment these fields when the mechanism motors are installed.
    // private DcMotorEx launcherLeft;
    // private DcMotorEx launcherRight;
    // private DcMotorEx intake;

    private Path startToLaunch() {
        return line(startPose, launchPose).constant(startPose);
    }

    private Path launchToPickup() {
        return line(launchPose, pickupPose).constant(launchPose);
    }

    private Path pickupToLaunch() {
        return line(pickupPose, launchPose).constant(launchPose);
    }

    private Path launchToPark() {
        return curve(launchPose, parkControlPose, parkPose).constant(parkPose);
    }

    /**
     * Simulates spinning up both launcher motors and feeding POLLEN with the intake.
     * Uncomment the motor lines after the mechanisms are added to the robot.
     */
    private Command launchPollen(String shotName) {
        return sequential(
                instant(() -> {
                    currentAction = "Spinning up launcher: " + shotName;
                    // launcherLeft.setPower(1.0);
                    // launcherRight.setPower(1.0);
                }),
                waitMs(LAUNCHER_SPIN_UP_MS),
                instant(() -> {
                    currentAction = "Feeding POLLEN: " + shotName;
                    // intake.setPower(1.0);
                }),
                waitMs(POLLEN_FEED_MS),
                instant(() -> {
                    // intake.setPower(0.0);
                    // launcherLeft.setPower(0.0);
                    // launcherRight.setPower(0.0);
                    currentAction = "Launch complete: " + shotName;
                }),
                waitMs(MECHANISM_SETTLE_MS)
        );
    }

    /** Simulates running the intake while the robot moves through the pickup area. */
    private Command collectPollen() {
        return sequential(
                instant(() -> {
                    currentAction = "Intaking POLLEN";
                    // intake.setPower(1.0);
                }),
                waitMs(INTAKE_COLLECTION_MS),
                instant(() -> {
                    // intake.setPower(0.0);
                    currentAction = "POLLEN collected";
                })
        );
    }

    private Command autoRoutine() {
        return sequential(
                instant(() -> currentAction = "Driving to HIVE launch position"),
                follow(follower, startToLaunch()),
                launchPollen("preload"),

                instant(() -> currentAction = "Driving through GARDEN pickup"),
                parallel(
                        follow(follower, launchToPickup()),
                        collectPollen()
                ),

                instant(() -> currentAction = "Returning to HIVE launch position"),
                follow(follower, pickupToLaunch()),
                launchPollen("collected POLLEN"),

                instant(() -> currentAction = "Parking in LOADING ZONE"),
                follow(follower, launchToPark()),
                instant(() -> {
                    stopMechanisms();
                    follower.stop();
                    currentAction = "Auto complete";
                    routineComplete = true;
                })
        );
    }

    /** Hardware setup to enable after the two launcher motors and intake are installed. */
    private void configureMechanisms() {
        // Change these strings if your Robot Configuration uses different names.
        // launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        // launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        // intake = hardwareMap.get(DcMotorEx.class, "intake");

        // One launcher motor will usually need reversing because the wheels face each other.
        // launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        // launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // stopMechanisms();
    }

    private void stopMechanisms() {
        // intake.setPower(0.0);
        // launcherLeft.setPower(0.0);
        // launcherRight.setPower(0.0);
    }

    @Override
    public void init() {
        Scheduler.reset();
        routineComplete = false;
        configureMechanisms();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        telemetry.addLine("Dummy mechanism auto ready");
        telemetry.addLine("Motor calls are commented out; timing pauses are active");
        telemetry.addLine("Use only in an open test area until poses are calibrated");
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        if (!routineComplete && autoTimer.seconds() < AUTO_STOP_SECONDS) {
            follower.update();
            Scheduler.execute();
        } else if (!routineComplete) {
            stopMechanisms();
            follower.stop();
            Scheduler.reset();
            currentAction = "Stopped by 29-second fail-safe";
            routineComplete = true;
        }

        telemetry.addData("Action", currentAction);
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("X", "%.1f in", follower.pose().x());
        telemetry.addData("Y", "%.1f in", follower.pose().y());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMechanisms();
        if (follower != null) {
            follower.stop();
        }
        Scheduler.reset();
    }
}
