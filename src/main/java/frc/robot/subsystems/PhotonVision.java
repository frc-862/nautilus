// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class PhotonVision extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private VisionSystemSim visionSim;

    private PhotonCameraSim cameraSim;
    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    private PhotonPipelineResult result = new PhotonPipelineResult();

    private Pose2d lastEstimatedRobotPose = new Pose2d();
    private EstimatedRobotPose estimatedRobotPose = new EstimatedRobotPose(new Pose3d(), 0, null, null);
    private double lastPoseTime = 0;

    private Field2d field = new Field2d();

    public PhotonVision() {
        camera = new PhotonCamera(VisionConstants.camera1Name);

        poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d());

        if (!Robot.isReal()) {
            visionTarget = new VisionTargetSim(VisionConstants.targetPose, VisionConstants.targetModel);
            cameraProp = new SimCameraProperties();
            visionSim = new VisionSystemSim("test");

            visionSim.addVisionTargets(visionTarget);
            visionSim.addAprilTags(VisionConstants.tagLayout);

            // TODO: set camera properties
            // cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            // cameraProp.setCalibError(0.25, 0.08);
            // cameraProp.setFPS(20);
            // cameraProp.setAvgLatencyMs(35);
            // cameraProp.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(camera, cameraProp);
            visionSim.addCamera(cameraSim, VisionConstants.robotToCamera);

            // Enable the raw and processed streams. These are enabled by default.
            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);

            // Enable drawing a wireframe visualization of the field to the camera streams.
            // This is extremely resource-intensive and is disabled by default.
            // cameraSim.enableDrawWireframe(true);
        }
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public double getXBestTarget() {
        return result.getBestTarget().getYaw();
    }

    public double getYBestTarget() {
        return result.getBestTarget().getPitch();
    }

    public double getSkewBestTarget() {
        return result.getBestTarget().getSkew();
    }

    public Transform3d getTransformBestTarget() {
        return result.getBestTarget().getBestCameraToTarget();
    }

    public double getPitch(){
        return camera.getCameraTable().getEntry("targetPitch").getDouble(0);
    }

    public double getYaw(){
        return camera.getCameraTable().getEntry("targetYaw").getDouble(0);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update(result);
    }

    public void setEstimatedPose(EstimatedRobotPose pose) {
        estimatedRobotPose = pose;

        lastPoseTime = pose.timestampSeconds;
    }

    public Command updateOdometry(Swerve swerve) {
        if (Robot.isReal()) {
            return run(() -> {
                swerve.addVisionMeasurement(estimatedRobotPose);
            }).ignoringDisable(true);
        } else {
            return run(() -> {
                visionSim.update(swerve.getPose());
                swerve.addVisionMeasurement(estimatedRobotPose);
            }).ignoringDisable(true);
        }
    }

    @Override
    public void periodic() {

        LightningShuffleboard.setBool("Vision", "HasTarget", result.hasTargets());

        try {
            // get the latest result
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            result = results.get(results.size() - 1);
        } catch (Exception e) {
            DataLogManager.log("[VISION] Pose Estimator Failed to update: " + e.getLocalizedMessage());
        }

        LightningShuffleboard.setBool("Vision", "HasResult", result.hasTargets());
        LightningShuffleboard.set("Vision", "Timestamp", result.getTimestampSeconds());

        if (result.hasTargets()) {
            getEstimatedGlobalPose(lastEstimatedRobotPose).ifPresentOrElse(
                    (m_estimatedRobotPose) -> setEstimatedPose(m_estimatedRobotPose),
                    () -> DataLogManager.log("[VISION] Pose Estimator Failed to update"));

            lastEstimatedRobotPose = estimatedRobotPose.estimatedPose.toPose2d();
            field.setRobotPose(lastEstimatedRobotPose);

            LightningShuffleboard.set("Vision", "Field", field);
        } else {
            if (!DriverStation.isFMSAttached()) {
                DataLogManager.log("[VISION] Pose Estimator Failed to update");
            }
        }

    }

    @Override
    public void simulationPeriodic() {
        LightningShuffleboard.set("Vision", "Field_SIM", visionSim.getDebugField());
    }
}
