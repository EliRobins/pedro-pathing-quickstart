package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * A small first autonomous using the Pedro Pathing 3 + Ivy structure from the
 * official documentation. The robot starts at (0, 0) and drives forward 24 inches.
 */
@Autonomous(name = "First Auto", group = "Competition")
public class FirstAuto extends OpMode {
    private final PoseFactory poses = PoseFactory.degrees();

    private final Pose startPose = poses.of(0, 0, 0);
    private final Pose endPose = poses.of(24, 0, 0);

    private Follower follower;

    private Path driveForward() {
        return line(startPose, endPose).linear(startPose, endPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, driveForward())
                // Add mechanism commands or more follow(...) commands here.
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        telemetry.addLine("First Auto ready");
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
        Scheduler.reset();
    }
}
