package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    public static final String TABLE_NAME = "Vision";
    public static final String VALUES_TOPIC = "values";
    public static final String RECORDING_TOPIC = "recording";
    public static final String STREAM_CAM_0 = "shouldStream0";
    public static final String ENABLED_0_TOPIC = "enable0";
    public static final String ENABLED_1_TOPIC = "enable1";

    public static final double WIDTH = 800;
    public static final double HEIGHT = 600;
    public static final double FOV = 100;

    public static final double DRIVE_P = 0.005;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0.0002;
    public static final double SPIN_P = 0.005;
    public static final double SPIN_I = 0;
    public static final double SPIN_D = 0.0002;
    public static final double SPIN_SETPOINT = 0.0;
    public static final double APRIL_TAG_AREA = 28908;

    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private static final double FRONT_CAMERA_X_OFFSET = Units.inchesToMeters(13.730);
    private static final double FRONT_CAMERA_Y_OFFSET = Units.inchesToMeters(-11.995);
    private static final double FRONT_CAMERA_Z_OFFSET = Units.inchesToMeters(7.35);
    public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(
            new Translation3d(FRONT_CAMERA_X_OFFSET, FRONT_CAMERA_Y_OFFSET, FRONT_CAMERA_Z_OFFSET),
            new Rotation3d(0, 0, 0));
    public static final String ROBOT_TO_FRONT_CAM_NAME = "frontCamera";

    private static final double BACK_CAMERA_X_OFFSET = Units.inchesToMeters(-10.5277);
    private static final double BACK_CAMERA_Y_OFFSET = Units.inchesToMeters(-11.391);
    private static final double BACK_CAMERA_Z_OFFSET = Units.inchesToMeters(7.6);
    public static final Transform3d ROBOT_TO_BACK_CAM = new Transform3d(
            new Translation3d(BACK_CAMERA_X_OFFSET, BACK_CAMERA_Y_OFFSET, BACK_CAMERA_Z_OFFSET),
            new Rotation3d(0, 0, Math.toRadians(180 - 45)));
    public static final String ROBOT_TO_BACK_CAM_NAME = "backCamera";

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
}
