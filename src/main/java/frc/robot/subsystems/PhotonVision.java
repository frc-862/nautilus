// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Tuple;

public class PhotonVision extends SubsystemBase {

    private PhotonCamera leftCam;
    private PhotonCamera rightCam;

    private VisionSystemSim visionSim;

    private PhotonCameraSim cameraSim;
    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    private Swerve drivetrain;

    private CameraThread leftThread;
    private CameraThread rightThread;

    public PhotonVision(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        leftCam = new PhotonCamera(VisionConstants.leftCamName);
        rightCam = new PhotonCamera(VisionConstants.rightCamName);

        leftThread = new CameraThread(leftCam, "left");
        rightThread = new CameraThread(rightCam, "right");

        leftThread.start();
        rightThread.start();

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


    private void updateVision() {
        Optional<Tuple<EstimatedRobotPose, Double>> leftUpdates = leftThread.getUpdates();
        Optional<Tuple<EstimatedRobotPose, Double>> rightUpdates = rightThread.getUpdates();

        if(leftUpdates.isPresent() && rightUpdates.isPresent()) {
            Tuple<EstimatedRobotPose, Double> left = leftUpdates.get();
            Tuple<EstimatedRobotPose, Double> right = rightUpdates.get();
        }
    }


    private class CameraThread extends Thread {
        private EstimatedRobotPose pose = new EstimatedRobotPose(new Pose3d(), 0, null, null);
        private PhotonPoseEstimator poseEstimator;
        private PhotonCamera camera;
        private Double averageDistance = 0d;
        private String camName; //for logging
        private Queue<Tuple<EstimatedRobotPose, Double>> shitCode;

        public CameraThread(PhotonCamera camera, String camName) {
            this.camera = camera;
            this.camName = camName;

            poseEstimator = new PhotonPoseEstimator(VisionConstants.tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotLeftToCamera);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        public void run() {
            try {
                List<PhotonPipelineResult> results = camera.getAllUnreadResults();
                double numberOfResults = results.size(); //double to prevent integer division errors
                double totalDistances = 0;
                for (PhotonPipelineResult result : results) {
                    if (result.hasTargets()) {
                        poseEstimator.update(result).ifPresentOrElse((pose) -> this.pose = pose,
                                () -> DataLogManager.log(camName + "pose update failed"));
                                
                        // grabs the aveerage distance to the best target (for the latest set of results)
                        totalDistances += result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
                        
                        LightningShuffleboard.setBool("Vision", camName + " targets found", !result.targets.isEmpty());
                        LightningShuffleboard.setPose2d("Vision", camName + " pose", pose.estimatedPose.toPose2d());
                        LightningShuffleboard.setBool("Vision", camName + " hasTarget", result.hasTargets());
                        LightningShuffleboard.setDouble("Vision", camName + " Timestamp", result.getTimestampSeconds());
                    } else {
                        LightningShuffleboard.setBool("Vision", camName + " hasTarget", false);
                    }
                    
                    LightningShuffleboard.setBool("Vision", camName + " functional", true);
                }

                // averages distance over all results
                averageDistance = totalDistances / numberOfResults;
                shitCode.add(new Tuple<EstimatedRobotPose, Double>(pose, averageDistance));
            } catch (IndexOutOfBoundsException e) {
                // if there are no results, 
                LightningShuffleboard.setBool("Vision", camName + " functional", false);
                LightningShuffleboard.setBool("Vision", camName + " hasTarget", false);
            }
        }

        /**
         * Returns the most recent pose and average distance to the best target
         * 
         * using a tuple as a type-safe alternative to the classic "return an array" (i hate java)
         * 
         * @return Tuple<EstimatedRobotPose, Double> - the most recent pose and average distance to the best target
         */
        public Optional<Tuple<EstimatedRobotPose, Double>> getUpdates() {
            return Optional.ofNullable(shitCode.poll());
        }
    }      
}
