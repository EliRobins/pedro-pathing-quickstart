package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class OtherTeam {
    private final List<Integer> redHiveBack = List.of(30, 31, 32, 33);
    private final List<Integer> redHiveFront = List.of(34, 35, 36, 37);
    private final List<Integer> blueHiveBack = List.of(42, 43, 44, 45);
    private final List<Integer> blueHiveFront = List.of(38, 39, 40, 41);
    private final List<Integer> redHive = Stream.concat(redHiveBack.stream(), redHiveFront.stream())
            .collect(Collectors.toList());
    private final List<Integer> blueHive = Stream.concat(blueHiveBack.stream(), blueHiveFront.stream())
            .collect(Collectors.toList());
    private final List<Integer> hiveBack = Stream.concat(redHiveBack.stream(), blueHiveBack.stream())
            .collect(Collectors.toList());
    private final List<Integer> hiveFront = Stream.concat(redHiveFront.stream(), blueHiveFront.stream())
            .collect(Collectors.toList());

    private final List<Integer> hives = Stream.concat(redHive.stream(), blueHive.stream())
            .collect(Collectors.toList());

    private void updateHiveStates() {
        List<LLResultTypes.FiducialResult> aprilTags = lastResult.getFiducialResults();

        for (LLResultTypes.FiducialResult aprilTag : aprilTags) {
            int aprilTagId = aprilTag.getFiducialId();

            if (!hives.contains(aprilTagId))
                continue;

            Pose3D aprilTagPose = aprilTag.getTargetPoseRobotSpace();

            boolean red = redHive.contains(aprilTagId);
            boolean front = hiveFront.contains(aprilTagId);
            boolean up = aprilTagPose.getPosition().z >= hiveTipCutoff;

            boolean tipped = front ^ up ^ red;

            if (red)
                redHiveTipped = tipped;
            else
                blueHiveTipped = tipped;
        }
    }

    public static boolean getHiveState(Robot.Alliance alliance) {
        return alliance == Robot.Alliance.RED ? redHiveTipped : blueHiveTipped;
    }

    @Override
    public void update() {
        if (getState() == State.DISABLED || isSwitchingPipelines()) return;

        updateYaw(DriveBase.getPos());

        lastResult = limelight.getLatestResult();

        if (lastResult != null && lastResult.isValid()) {
            updateStateMachine(Map.of(
                    State.BALL_TRACKING, () -> updateBallLocations(DriveBase.getPos()),
                    State.HIVE_TRACKING, this::updateHiveStates
            ));
            lastResultNull = false;
        } else {
            lastResultNull = true;
        }
    }
}
