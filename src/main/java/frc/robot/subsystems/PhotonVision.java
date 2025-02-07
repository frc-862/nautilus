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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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

    private boolean visionWorks;

    private Field2d field = new Field2d();

    private Swerve drivetrain;

    public PhotonVision(Swerve drivetrain) {
        leftCam = new PhotonCamera(VisionConstants.leftCamName);
        rightCam = new PhotonCamera(VisionConstants.rightCamName);
        
        // leftCam.setPipelineIndex(0);

        poseEstimatorLeft = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.robotLeftToCamera);
        poseEstimatorRight = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.robotRightToCamera);

        poseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.drivetrain = drivetrain;

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

    public void switchPipelines(int pipeline, PhotonCamera camera) {
        camera.setPipelineIndex(pipeline);
    }

    @Override
    public void periodic() {
        try {
            for(PhotonPipelineResult result : leftCam.getAllUnreadResults()) {
                leftResult = result;
                if (leftResult.hasTargets()) {
                    visionWorks = true;
                    poseEstimatorLeft.update(leftResult).ifPresentOrElse((pose) -> leftPose = pose, () -> DataLogManager.log("left pose update failed"));
                    setEstimatedPose(leftPose);
                    
                    LightningShuffleboard.setBool("Vision", "targets found", !leftResult.targets.isEmpty());
                    field.setRobotPose(leftPose.estimatedPose.toPose2d()); 
                    LightningShuffleboard.send("Vision", "left field", field); 
                }

                
                LightningShuffleboard.setBool("Vision", "left functional", true);
                LightningShuffleboard.setBool("Vision", "left hasTarget", leftResult.hasTargets());
                LightningShuffleboard.setDouble("Vision", "left Timestamp", leftResult.getTimestampSeconds());
            }
        } catch (IndexOutOfBoundsException e) {
            LightningShuffleboard.setBool("Vision", "left functional", false);
            LightningShuffleboard.setBool("Vision", "left hasTarget", false);

        }

        try {
            for(PhotonPipelineResult result : rightCam.getAllUnreadResults()) {
                rightResult = result;
                if (rightResult.hasTargets()) {
                    visionWorks = true;
                    poseEstimatorRight.update(rightResult).ifPresentOrElse((pose) -> rightPose = pose, () -> DataLogManager.log("right pose update failed"));
                    
                    LightningShuffleboard.setBool("Vision", "targets found", !rightResult.targets.isEmpty());
                    field.setRobotPose(rightPose.estimatedPose.toPose2d()); 
                    LightningShuffleboard.send("Vision", "left field", field); 
                }
                
                setEstimatedPose(rightPose);
                LightningShuffleboard.setBool("Vision", "right functional", true);
                LightningShuffleboard.setBool("Vision", "right hasTarget", rightResult.hasTargets());
                LightningShuffleboard.setDouble("Vision", "right Timestamp", rightResult.getTimestampSeconds());

            }
        } catch (IndexOutOfBoundsException e) {
            LightningShuffleboard.setBool("Vision", "right functional", false);
            LightningShuffleboard.setBool("Vision", "right hasTarget", false);

        }

        LightningShuffleboard.setBool("Vision", "PoseEstimatorWorks", visionWorks);
        
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
        LightningShuffleboard.send("Vision", "Field_SIM", visionSim.getDebugField());
    }

    public boolean leftHasTarget() {
        return leftResult.hasTargets();
    }

    public double getTY(VisionConstants.Camera camera, double offset) {
        switch(camera){
            case LEFT:
            return leftCam.getCameraTable().getEntry("targetPixelsY").getDouble(0) + offset;
            
            case RIGHT:
            return rightCam.getCameraTable().getEntry("targetPixelsY").getDouble(0) + offset;

            default:
            return 0;
        }
    }

    public double getTX(VisionConstants.Camera camera, double offset) {
        switch(camera){
            case LEFT:
            return leftCam.getCameraTable().getEntry("targetPixelsX").getDouble(0) + offset;
            
            case RIGHT:
            return rightCam.getCameraTable().getEntry("targetPixelsX").getDouble(0) + offset;

            default:
            return 0;
        }
    }

    public int getTagNum(VisionConstants.Camera camera) {
        switch(camera){
            case LEFT:
            return leftResult.getBestTarget().getFiducialId();

            case RIGHT:
            return rightResult.getBestTarget().getFiducialId();

            default:
            return 0;
        }
    }

    public boolean rightHasTarget() {
        return rightResult.hasTargets();
    }

}
