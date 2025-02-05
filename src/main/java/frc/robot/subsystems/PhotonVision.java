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
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class PhotonVision extends SubsystemBase {

    private PhotonCamera leftCam;
    private PhotonCamera rightCam;
    private PhotonPoseEstimator poseEstimator;
    private VisionSystemSim visionSim;

    private PhotonCameraSim cameraSim;
    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    private PhotonPipelineResult leftResult = new PhotonPipelineResult();
    private PhotonPipelineResult rightResult = new PhotonPipelineResult();

    private boolean visionWorks;

    private Field2d field = new Field2d();

    private Swerve drivetrain;

    public PhotonVision(Swerve drivetrain) {
        leftCam = new PhotonCamera(VisionConstants.leftCamName);
        rightCam = new PhotonCamera(VisionConstants.rightCamName);
        
        leftCam.setPipelineIndex(0);

        poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d());

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.drivetrain = drivetrain;

        if (!Robot.isReal()) {
            //TODO: update this to have multiple cameras

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
            visionSim.addCamera(cameraSim, VisionConstants.robotToCamera);

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

    public void switchPipelines(int pipeline, PhotonCamera camera){
        camera.setPipelineIndex(pipeline);
    }

    @Override
    public void periodic() {
        visionWorks = true;

        try {
            // get the latest result
            List<PhotonPipelineResult> results = leftCam.getAllUnreadResults();
            leftResult = results.get(results.size() - 1);

            if (leftResult.hasTargets()) {
                visionWorks = true;
                poseEstimator.update(leftResult).ifPresentOrElse(
                        (m_estimatedRobotPose) -> setEstimatedPose(m_estimatedRobotPose),
                        () -> visionWorks = false);
            }
        } catch (Exception e) {
            visionWorks = false;
        }

        
        try {
            // get the latest result
            List<PhotonPipelineResult> results = rightCam.getAllUnreadResults();
            rightResult = results.get(results.size() - 1);

            if (rightResult.hasTargets()) {
                visionWorks = true;
                poseEstimator.update(rightResult).ifPresentOrElse(
                        (m_estimatedRobotPose) -> setEstimatedPose(m_estimatedRobotPose),
                        () -> visionWorks = false);
            }
        } catch (Exception e) {
            visionWorks = false;
        }
        
      
        
        LightningShuffleboard.send("Vision", "Field", field);
        LightningShuffleboard.setBool("Vision", "PoseEstimatorWorks", visionWorks);
        LightningShuffleboard.setBool("Vision", "left hasTarget", leftResult.hasTargets());
        LightningShuffleboard.setBool("Vision", "right hasTarget", rightResult.hasTargets());
        LightningShuffleboard.setDouble("Vision", "left Timestamp", leftResult.getTimestampSeconds());
        LightningShuffleboard.setDouble("Vision", "right Timestamp", rightResult.getTimestampSeconds());
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
        LightningShuffleboard.send("Vision", "Field_SIM", visionSim.getDebugField());
    }




    public boolean leftHasTarget() {
        return leftResult.hasTargets();
    }

    public double getLeftTY(){
        return leftCam.getCameraTable().getEntry("targetPixelsY").getDouble(0);
    }

    public double getLeftTX(){
        return leftCam.getCameraTable().getEntry("targetPixelsX").getDouble(0);
    }

    public int getLeftTagNum(){
        return leftResult.getBestTarget().getFiducialId();
    }


    public boolean rightHasTarget() {
        return rightResult.hasTargets();
    }

    public double getRightTY(){
        return rightCam.getCameraTable().getEntry("targetPixelsY").getDouble(0);
    }

    public double getRightTX(){
        return rightCam.getCameraTable().getEntry("targetPixelsX").getDouble(0);
    }

    public int getRightTagNum(){
        return rightResult.getBestTarget().getFiducialId();
    }

}
