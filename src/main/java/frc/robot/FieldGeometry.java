package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

public final class FieldGeometry {

    private FieldGeometry() {}

    /* ===================================================== */
    /* ================= FIELD LAYOUT ====================== */
    /* ===================================================== */

    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(
                    AprilTagFields.k2026RebuiltAndymark
            );

    /*
        FIELD COORDINATE SYSTEM (WPILib Standard)

        Origin (0,0):
            Blue alliance driver station LOWER-LEFT corner
            (when viewed in simulation)

        +X:
            Toward Red alliance wall

        +Y:
            Toward left side of field from Blue perspective

        0° Rotation:
            Facing +X direction

        Rotation:
            Counterclockwise positive
    */

    /* ===================================================== */
    /* ================= ALLIANCE CHECK ==================== */
    /* ===================================================== */

    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(Alliance.Blue) == Alliance.Blue;
    }

    /* ===================================================== */
    /* ============ ROBOT ORIENTATION CONFIG =============== */
    /* ===================================================== */

    /*
        Change these if robot orientation changes.

        SHOOTER_FORWARD:
            Robot front faces target

        INTAKE_BACKWARD:
            Robot back faces target
     */

    public static final Rotation2d SHOOTER_FORWARD =
            Rotation2d.fromDegrees(0);

    public static final Rotation2d INTAKE_BACKWARD =
            Rotation2d.fromDegrees(180);

    /* ===================================================== */
    /* ================= OUTPOST SETTINGS ================== */
    /* ===================================================== */

    // Distance forward from tag to stop
    public static final double OUTPOST_FORWARD_OFFSET = 0.85;

    // Left/right shift relative to tag
    public static final double OUTPOST_LATERAL_OFFSET = 0.0;

    /* ===================================================== */
    /* ================= FUEL HUB SETTINGS ================= */
    /* ===================================================== */

    // Distance away from hub center
    public static final double HUB_DISTANCE_OFFSET = 1.1;

    /* ===================================================== */
    /* ======== CLOSEST ALLIANCE TAG SELECTION ============ */
    /* ===================================================== */

    public static Optional<Integer> getClosestAllianceTag(
            Pose2d robotPose
    ) {
        boolean blue = isBlue();
        double bestDistance = Double.MAX_VALUE;
        Integer bestTag = null;

        for (int id = 1; id <= 32; id++) {

            // Alliance filtering:
            // 1–16 = Red
            // 17–32 = Blue
            if (blue && id <= 16) continue;
            if (!blue && id >= 17) continue;

            var tagOpt = FIELD_LAYOUT.getTagPose(id);
            if (tagOpt.isEmpty()) continue;

            Pose2d tagPose = tagOpt.get().toPose2d();

            double distance =
                    robotPose.getTranslation()
                            .getDistance(tagPose.getTranslation());

            if (distance < bestDistance) {
                bestDistance = distance;
                bestTag = id;
            }
        }

        return Optional.ofNullable(bestTag);
    }

    /* ===================================================== */
    /* ================= OUTPOST TARGET ==================== */
    /* ===================================================== */

    public static Optional<Pose2d> getOutpostTargetPose(
            Pose2d robotPose
    ) {

        Optional<Integer> tagId =
                getClosestAllianceTag(robotPose);

        if (tagId.isEmpty()) return Optional.empty();

        var tagOpt = FIELD_LAYOUT.getTagPose(tagId.get());
        if (tagOpt.isEmpty()) return Optional.empty();

        Pose2d tagPose = tagOpt.get().toPose2d();

        // Apply translation offset from tag
        Pose2d translated =
                tagPose.transformBy(
                        new Transform2d(
                                new Translation2d(
                                        OUTPOST_FORWARD_OFFSET,
                                        OUTPOST_LATERAL_OFFSET
                                ),
                                new Rotation2d()
                        )
                );

        // Face OUTPOST using intake (robot back toward tag)
        Rotation2d heading =
                tagPose.getRotation()
                        .rotateBy(INTAKE_BACKWARD);

        return Optional.of(
                new Pose2d(
                        translated.getTranslation(),
                        heading
                )
        );
    }

    /* ===================================================== */
    /* ================= FUEL HUB CENTER =================== */
    /* ===================================================== */

    public static Translation2d getFuelHubCenter() {

        // Pick two hub tags per alliance
        int tagA = isBlue() ? 25 : 9;
        int tagB = isBlue() ? 26 : 10;

        Pose2d a =
                FIELD_LAYOUT.getTagPose(tagA).get().toPose2d();
        Pose2d b =
                FIELD_LAYOUT.getTagPose(tagB).get().toPose2d();

        return a.getTranslation()
                .interpolate(b.getTranslation(), 0.5);
    }

    /* ===================================================== */
    /* ================= FUEL HUB TARGET =================== */
    /* ===================================================== */

    public static Pose2d getFuelHubTargetPose(
            Pose2d robotPose
    ) {

        Translation2d hubCenter = getFuelHubCenter();

        Translation2d direction =
                robotPose.getTranslation()
                        .minus(hubCenter);

        double distance = direction.getNorm();

        Translation2d unit =
                distance > 1e-6
                        ? direction.div(distance)
                        : new Translation2d();

        // Face hub using shooter/front
        Rotation2d facing =
                new Rotation2d(
                        Math.atan2(
                                hubCenter.getY() - robotPose.getY(),
                                hubCenter.getX() - robotPose.getX()
                        )
                ).rotateBy(SHOOTER_FORWARD);

        Translation2d stopPoint =
                hubCenter.plus(
                        unit.times(HUB_DISTANCE_OFFSET)
                );

        return new Pose2d(stopPoint, facing);
    }
}
