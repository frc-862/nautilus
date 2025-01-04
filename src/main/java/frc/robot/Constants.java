package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class RobotMap {

    }

    public static class TunerConstants {
        // To replace with phoenix constants
    }

    public static class VisionConstants {
        public static final String camera1Name = "cam1";
    }

    public static class PoseConstants {
        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0),
                Units.feetToMeters(26.0));
    }
}
