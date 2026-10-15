package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Demonstrates paths to the centers of the audience- and far-facing sides of
 * the BIOBUZZ HIVE.
 *
 * <p>Pedro coordinates are in inches with (0, 0) at the lower-left field
 * corner. The two public static path methods can be reused by any OpMode and
 * accept any legal starting pose outside the HIVE keep-out area.</p>
 */
@Autonomous(name = "Hive Side Auto", group = "Competition")
public class HiveSideAuto extends OpMode {
    private static final PoseFactory POSES = PoseFactory.degrees();

    // Official field/frame dimensions. Tune ROBOT_CENTER_STANDOFF_INCHES for
    // your robot so its frame and mechanisms remain clear of the HIVE.
    private static final double FIELD_CENTER_INCHES = 72.0;
    private static final double HIVE_FRAME_WIDTH_INCHES = 49.46;
    private static final double HIVE_FRAME_DEPTH_INCHES = 38.95;
    public static final double ROBOT_CENTER_STANDOFF_INCHES = 12.0;

    private static final double HIVE_LEFT_X =
            FIELD_CENTER_INCHES - HIVE_FRAME_WIDTH_INCHES / 2.0;
    private static final double HIVE_RIGHT_X =
            FIELD_CENTER_INCHES + HIVE_FRAME_WIDTH_INCHES / 2.0;
    private static final double HIVE_AUDIENCE_Y =
            FIELD_CENTER_INCHES - HIVE_FRAME_DEPTH_INCHES / 2.0;
    private static final double HIVE_FAR_Y =
            FIELD_CENTER_INCHES + HIVE_FRAME_DEPTH_INCHES / 2.0;

    private static final double AUDIENCE_APPROACH_Y =
            HIVE_AUDIENCE_Y - ROBOT_CENTER_STANDOFF_INCHES;
    private static final double FAR_APPROACH_Y =
            HIVE_FAR_Y + ROBOT_CENTER_STANDOFF_INCHES;
    private static final double LEFT_BYPASS_X =
            HIVE_LEFT_X - ROBOT_CENTER_STANDOFF_INCHES;
    private static final double RIGHT_BYPASS_X =
            HIVE_RIGHT_X + ROBOT_CENTER_STANDOFF_INCHES;

    /** Centered on the audience-facing side and pointing toward the HIVE. */
    public static final Pose AUDIENCE_HIVE_SIDE =
            POSES.of(FIELD_CENTER_INCHES, AUDIENCE_APPROACH_Y, 90.0);

    /** Centered on the far-facing side and pointing toward the HIVE. */
    public static final Pose FAR_HIVE_SIDE =
            POSES.of(FIELD_CENTER_INCHES, FAR_APPROACH_Y, 270.0);

    // Change this to the robot's measured match-start pose.
    private static final Pose START_POSE = POSES.of(9.0, 72.0, 0.0);

    private Follower follower;
    private HiveSide selectedSide = HiveSide.AUDIENCE;

    /**
     * Builds a collision-aware path from {@code startingPose} to the center of
     * the HIVE side nearest the audience.
     */
    public static Path pathToAudienceHiveSide(Pose startingPose) {
        return pathToHiveSide(startingPose, AUDIENCE_HIVE_SIDE);
    }

    /**
     * Builds a collision-aware path from {@code startingPose} to the center of
     * the HIVE side farthest from the audience.
     */
    public static Path pathToFarHiveSide(Pose startingPose) {
        return pathToHiveSide(startingPose, FAR_HIVE_SIDE);
    }

    private static Path pathToHiveSide(Pose startingPose, Pose targetPose) {
        if (startingPose == null) {
            throw new IllegalArgumentException("startingPose cannot be null");
        }
        if (isInsideHiveKeepOut(startingPose)) {
            throw new IllegalArgumentException(
                    "startingPose must be outside the HIVE keep-out area");
        }

        if (!crossesHiveKeepOut(startingPose, targetPose)) {
            return line(startingPose, targetPose).linear(startingPose, targetPose);
        }

        double sourceSideY = startingPose.y() < FIELD_CENTER_INCHES
                ? AUDIENCE_APPROACH_Y
                : FAR_APPROACH_Y;

        double leftDistance = distance(startingPose.x(), startingPose.y(),
                LEFT_BYPASS_X, sourceSideY);
        double rightDistance = distance(startingPose.x(), startingPose.y(),
                RIGHT_BYPASS_X, sourceSideY);
        double bypassX = leftDistance <= rightDistance ? LEFT_BYPASS_X : RIGHT_BYPASS_X;

        Pose firstCorner = new Pose(bypassX, sourceSideY, startingPose.heading());
        Pose secondCorner = new Pose(bypassX, targetPose.y(), targetPose.heading());

        return path(
                line(startingPose, firstCorner).linear(startingPose, firstCorner),
                line(firstCorner, secondCorner).linear(firstCorner, secondCorner),
                line(secondCorner, targetPose).linear(secondCorner, targetPose)
        );
    }

    private static boolean isInsideHiveKeepOut(Pose pose) {
        return pose.x() > LEFT_BYPASS_X
                && pose.x() < RIGHT_BYPASS_X
                && pose.y() > AUDIENCE_APPROACH_Y
                && pose.y() < FAR_APPROACH_Y;
    }

    /** Uses a slab intersection test against the rectangular HIVE keep-out area. */
    private static boolean crossesHiveKeepOut(Pose start, Pose end) {
        double[] interval = {0.0, 1.0};
        double dx = end.x() - start.x();
        double dy = end.y() - start.y();

        return clip(-dx, start.x() - LEFT_BYPASS_X, interval)
                && clip(dx, RIGHT_BYPASS_X - start.x(), interval)
                && clip(-dy, start.y() - AUDIENCE_APPROACH_Y, interval)
                && clip(dy, FAR_APPROACH_Y - start.y(), interval)
                && interval[0] < 1.0 - 1e-9
                && interval[1] > 1e-9;
    }

    private static boolean clip(double denominator, double numerator, double[] interval) {
        if (Math.abs(denominator) < 1e-9) {
            return numerator >= 0.0;
        }

        double ratio = numerator / denominator;
        if (denominator < 0.0) {
            if (ratio > interval[1]) {
                return false;
            }
            interval[0] = Math.max(interval[0], ratio);
        } else {
            if (ratio < interval[0]) {
                return false;
            }
            interval[1] = Math.min(interval[1], ratio);
        }
        return true;
    }

    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
        follower.update();

        showSelection();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down) {
            selectedSide = HiveSide.AUDIENCE;
        } else if (gamepad1.dpad_up) {
            selectedSide = HiveSide.FAR;
        }
        showSelection();
    }

    @Override
    public void start() {
        Pose currentPose = follower.pose();
        Path selectedPath = selectedSide == HiveSide.AUDIENCE
                ? pathToAudienceHiveSide(currentPose)
                : pathToFarHiveSide(currentPose);
        schedule(follow(follower, selectedPath));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("Target side", selectedSide);
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

    private void showSelection() {
        telemetry.addLine("Hive Side Auto ready");
        telemetry.addLine("D-pad down: audience side");
        telemetry.addLine("D-pad up: far side");
        telemetry.addData("Selected", selectedSide);
        telemetry.update();
    }

    private enum HiveSide {
        AUDIENCE,
        FAR
    }
}
