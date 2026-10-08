package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class EliAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(48, 48, 90);
    private final Pose controlPose = p.of(36, 60, 45);

    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }
}
