// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class PhotonVision extends SubsystemBase {

    private PhotonCamera leftCam;
    private PhotonCamera rightCam;
    private PhotonPoseEstimator poseEstimatorLeft;
    private PhotonPoseEstimator poseEstimatorRight;

    private VisionSystemSim visionSim;

    private PhotonCameraSim cameraSim;
    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    private PhotonPipelineResult leftResult = new PhotonPipelineResult();
    private PhotonPipelineResult rightResult = new PhotonPipelineResult();

    private EstimatedRobotPose leftPose = new EstimatedRobotPose(new Pose3d(), 0, null, null);
    private EstimatedRobotPose rightPose = new EstimatedRobotPose(new Pose3d(), 0, null, null);

    private Field2d field = new Field2d();

    private Swerve drivetrain;

    public PhotonVision(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        leftCam = new PhotonCamera(VisionConstants.leftCamName);
        rightCam = new PhotonCamera(VisionConstants.rightCamName);

        poseEstimatorLeft = new PhotonPoseEstimator(VisionConstants.tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotLeftToCamera);
        poseEstimatorRight = new PhotonPoseEstimator(VisionConstants.tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotRightToCamera);

        poseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (!Robot.isReal()) {
            // TODO: update this to have multiple cameras

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

            cameraSim = new PhotonCameraSim(leftCam, cameraProp);
            visionSim.addCamera(cameraSim, VisionConstants.robotLeftToCamera);

            // Enable the raw and processed streams. These are enabled by default.
            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);

            // Enable drawing a wireframe visualization of the field to the camera streams.
            // This is extremely resource-intensive and is disabled by default.
            // cameraSim.enableDrawWireframe(true);
        }
    }

    public void setEstimatedPose(EstimatedRobotPose pose) {
        // could probably add an EnableVision flag here or something
        drivetrain.addVisionMeasurement(pose);
    }

    /**
     * Switch the pipeline of a camera
     *
     * @param camera   Camera to switch the pipeline of
     * @param pipeline Pipeline to switch to
     */
    public void switchPipelines(PhotonCamera camera, int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    @Override
    public void periodic() {
        try {
            for (PhotonPipelineResult result : leftCam.getAllUnreadResults()) {
                leftResult = result;
                if (leftResult.hasTargets()) {
                    poseEstimatorLeft.update(leftResult).ifPresentOrElse((pose) -> leftPose = pose,
                            () -> DataLogManager.log("[VISION] Left pose update failed"));
                    setEstimatedPose(leftPose);

                    LightningShuffleboard.setBool("Vision", "LEFT targets found", !leftResult.targets.isEmpty());
                    field.setRobotPose(leftPose.estimatedPose.toPose2d());
                    LightningShuffleboard.send("Vision", "LEFT field", field);
                }

                LightningShuffleboard.setBool("Vision", "LEFT functional", true);
                LightningShuffleboard.setBool("Vision", "LEFT hasTarget", leftResult.hasTargets());
                LightningShuffleboard.setDouble("Vision", "LEFT Timestamp", leftResult.getTimestampSeconds());
            }
        } catch (IndexOutOfBoundsException e) {
            LightningShuffleboard.setBool("Vision", "LEFT functional", false);
            LightningShuffleboard.setBool("Vision", "LEFT hasTarget", false);

        }

        try {
            for (PhotonPipelineResult result : rightCam.getAllUnreadResults()) {
                rightResult = result;
                if (rightResult.hasTargets()) {
                    poseEstimatorRight.update(rightResult).ifPresentOrElse((pose) -> rightPose = pose,
                            () -> DataLogManager.log("[VISION] Right pose update failed"));
                    setEstimatedPose(rightPose);

                    LightningShuffleboard.setBool("Vision", "RIGHT targets found", !rightResult.targets.isEmpty());
                    field.setRobotPose(rightPose.estimatedPose.toPose2d());
                    LightningShuffleboard.send("Vision", "RIGHT field", field);
                }

                setEstimatedPose(rightPose);
                LightningShuffleboard.setBool("Vision", "RIGHT functional", true);
                LightningShuffleboard.setBool("Vision", "RIGHT hasTarget", rightResult.hasTargets());
                LightningShuffleboard.setDouble("Vision", "RIGHT Timestamp", rightResult.getTimestampSeconds());

            }
        } catch (IndexOutOfBoundsException e) {
            LightningShuffleboard.setBool("Vision", "RIGHT functional", false);
            LightningShuffleboard.setBool("Vision", "RIGHT hasTarget", false);

        }
    }

    public boolean leftHasTarget() {
        return leftResult.hasTargets();
    }

    public double getLeftTY() {
        return leftCam.getCameraTable().getEntry("targetPixelsY").getDouble(0);
    }

    public double getLeftTX() {
        return leftCam.getCameraTable().getEntry("targetPixelsX").getDouble(0);
    }

    public int getLeftTagNum() {
        return leftResult.getBestTarget().getFiducialId();
    }

    public boolean rightHasTarget() {
        return rightResult.hasTargets();
    }

    public double getRightTY() {
        return rightCam.getCameraTable().getEntry("targetPixelsY").getDouble(0);
    }

    public double getRightTX() {
        return rightCam.getCameraTable().getEntry("targetPixelsX").getDouble(0);
    }

    public int getRightTagNum() {
        return rightResult.getBestTarget().getFiducialId();
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
        LightningShuffleboard.send("Vision", "Field_SIM", visionSim.getDebugField());
    }
}
