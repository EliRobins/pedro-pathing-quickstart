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

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Basic 2026-2027 BIOBUZZ autonomous using Pedro Pathing 3 and Ivy.
 *
 * Field setup:
 * 1. Point the front of the robot directly away from your alliance perimeter wall.
 * 2. Keep the robot touching that wall and fully outside the LOADING ZONE.
 * 3. Place it immediately to the right of the LOADING ZONE when viewed from
 *    behind the robot, looking into the field.
 * 4. Load the four required POLLEN according to the current game manual.
 *
 * Routine:
 * - Drives 24 inches away from the wall to earn LEAVE.
 * - Curves left and returns to the wall inside the LOADING ZONE to earn AUTO PARK.
 *
 * This project does not yet define an intake or launcher subsystem, so this routine
 * intentionally does not attempt a HIVE TIP. Add the mechanism command at the marked
 * location in autoRoutine() after those hardware classes exist.
 */
@Autonomous(name = "BIOBUZZ Basic: Leave + Park", group = "BIOBUZZ")
public class BiobuzzBasicAuto extends OpMode {
    private static final double AUTO_STOP_SECONDS = 29.0;

    private final PoseFactory poses = PoseFactory.degrees();

    // Local coordinates: +X is forward into the field and +Y is robot-left.
    private final Pose startPose = poses.of(0, 0, 0);
    private final Pose leavePose = poses.of(24, 0, 0);
    private final Pose parkControlPose = poses.of(24, 24, 0);
    private final Pose parkPose = poses.of(0, 24, 0);

    private final ElapsedTime autoTimer = new ElapsedTime();

    private Follower follower;
    private boolean routineComplete;

    private Path leaveStartingWall() {
        return line(startPose, leavePose).constant(startPose);
    }

    private Path returnToLoadingZone() {
        return curve(leavePose, parkControlPose, parkPose)
                .constant(parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, leaveStartingWall()),
                waitMs(250),

                // Add a launcher/intake command here once that subsystem exists.

                follow(follower, returnToLoadingZone()),
                instant(() -> {
                    follower.stop();
                    routineComplete = true;
                })
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        routineComplete = false;

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        telemetry.addLine("BIOBUZZ Basic Auto ready");
        telemetry.addLine("Start touching the wall, immediately right of the LOADING ZONE");
        telemetry.addLine("Robot front must point away from the wall");
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
            // Fail-safe: guarantee that the drivetrain is unpowered before AUTO ends.
            follower.stop();
            Scheduler.reset();
            routineComplete = true;
        }

        telemetry.addData("Status", routineComplete ? "Complete" : "Running");
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("X", "%.1f in", follower.pose().x());
        telemetry.addData("Y", "%.1f in", follower.pose().y());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
        Scheduler.reset();
    }
}
