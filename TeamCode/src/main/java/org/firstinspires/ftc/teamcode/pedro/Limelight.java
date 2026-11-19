package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Limelight {
    public enum HiveState {
        UNKNOWN,
        UPRIGHT,
        TIPPED
    }
    private Limelight3A limelight;
    private HiveState redHiveState = HiveState.UNKNOWN;
    private HiveState blueHiveState = HiveState.UNKNOWN;

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
    public Limelight(Limelight3A ll) {
        limelight = ll;
    }
    private void updateHiveStates() {
        List<LLResultTypes.FiducialResult> aprilTags = limelight.getLatestResult().getFiducialResults();

        for (LLResultTypes.FiducialResult aprilTag : aprilTags) {
            int aprilTagId = aprilTag.getFiducialId();

            if (!hives.contains(aprilTagId))
                continue;

            Pose3D aprilTagPose = aprilTag.getTargetPoseRobotSpace();

            boolean red = redHive.contains(aprilTagId);
            boolean front = hiveFront.contains(aprilTagId);
            boolean up = aprilTagPose.getPosition().z >= .5;

            boolean tipped = front ^ up ^ red;
            HiveState state = tipped ? HiveState.TIPPED : HiveState.UPRIGHT;

            if (red)
                redHiveState = state;
            else
                blueHiveState = state;
        }
    }

    public HiveState getHiveState(Robot.Alliance alliance) {
        return alliance == Robot.Alliance.RED ? redHiveState : blueHiveState;
    }
}
