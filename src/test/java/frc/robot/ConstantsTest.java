package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ConstantsTest {

    @Test
    public void testVisionConstants() {
        // Verify that the camera1Name constant is correctly initialized
        assertEquals("cam1", Constants.VisionConstants.camera1Name, 
            "camera1Name should be 'cam1'");
    }

    @Test
    public void testPoseConstants() {
        // Expected field limits in meters
        double expectedFieldLimitX = Units.feetToMeters(54.0); // 16.4592 meters
        double expectedFieldLimitY = Units.feetToMeters(26.0); // 7.9248 meters

        // Verify FIELD_LIMIT coordinates
        Translation2d fieldLimit = Constants.PoseConstants.FIELD_LIMIT;
        assertEquals(expectedFieldLimitX, fieldLimit.getX(), 1e-5, 
            "FIELD_LIMIT X-coordinate is incorrect");
        assertEquals(expectedFieldLimitY, fieldLimit.getY(), 1e-5, 
            "FIELD_LIMIT Y-coordinate is incorrect");
    }
}