// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Pose4d;

public class PhotonVision extends SubsystemBase {
    
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    private PhotonPipelineResult result;

    private Pose2d lastEstimatedRobotPose = new Pose2d();

    private Pose4d estimatedRobotPose = new Pose4d();
    private Field2d field = new Field2d();

    private double lastPoseTime = 0;

    public PhotonVision() {
        camera = new PhotonCamera(VisionConstants.camera1Name);

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d());
    }

    public void initLogging() {

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

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update(result);
    }

    public void setEstimatedPose(EstimatedRobotPose pose) {
        estimatedRobotPose = new Pose4d(pose.estimatedPose.getTranslation(), pose.estimatedPose.getRotation(),
                pose.timestampSeconds - lastPoseTime);

        lastPoseTime = pose.timestampSeconds;
    }

    // public Command updateOdometry(Swerve swerve) {
    //     return run(() -> {
    //         swerve.applyVisionPose(estimatedRobotPose);
    //     }).ignoringDisable(true);
    // }

    @Override
    public void periodic() {
        // try {
            result = camera.getLatestResult();
        // } catch (IndexOutOfBoundsException e) {
        //     System.out.println("[VISION] Failed to gather camera result");
        // }


        LightningShuffleboard.setBool("Vision", "HasResult", result.hasTargets());
        LightningShuffleboard.set("Vision", "timestamp", result.getTimestampSeconds());

        if (result.hasTargets()) {
            getEstimatedGlobalPose(lastEstimatedRobotPose).ifPresentOrElse((m_estimatedRobotPose) -> setEstimatedPose(m_estimatedRobotPose), () -> {System.out.println("[VISION] god freaking dang it");});
        
            lastEstimatedRobotPose = estimatedRobotPose.toPose2d();
            field.setRobotPose(lastEstimatedRobotPose);

            LightningShuffleboard.set("Vision", "Field", field);


        } else {
            if (!DriverStation.isFMSAttached()) {
                System.out.println("[VISION] Pose Estimator Failed to update");
            }
        }

    }
}
