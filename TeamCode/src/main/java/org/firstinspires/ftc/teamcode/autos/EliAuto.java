package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;

import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous
public class EliAuto extends OpMode {
    // The follower controls the drivetrain, keeps track of the robot's position, and runs the path following algorithm
    private Follower follower;
    // A `Pose` represents a position and a heading on the field with x, y, and heading values
    // This uses the Pedro coordinate system instead of the FTC official coordinate system
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(0, 0, 0);
    private final Pose park = p.of(0, 0, 90);

    // This method returns the path the robot should follow
    private Path park() {
        // `line()` creates a straight path from `startPose` to `park`
        // `.linear()` sets it to use a linear interpolation for the heading
        return line(startPose, park).linear(startPose, park);
    }

    // Runs when the OpMode is initialized, but before the start button is pressed
    @Override
    public void init() {
        // I guess the scheduler should be reset
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        // The Pose given to `setPose()` should match where the robot is actually placed
        follower.setPose(startPose);
    }

    // This runs when the start button is pressed
    @Override
    public void start() {
        schedule(follow(follower, park()));
    }

    @Override
    public void loop() {
        // Make sure they're still following, you know
        follower.update();
        // Apparently the scheduler needs to be killed now...
        Scheduler.execute();
    }
}