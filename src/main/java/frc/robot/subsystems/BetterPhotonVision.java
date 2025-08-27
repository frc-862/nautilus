// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.ReefPose;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class BetterPhotonVision extends SubsystemBase {
    // We use the drivetrain to actually add vision measurment
    private Swerve drivetrain;

    // Create the threads
    private CameraThread leftCameraThread;
    private CameraThread rightCameraThread;

    //  This is similar to a type in typescrip, it will store our vision data in our queue instead of using a tuple
    private record VisionUpdate(EstimatedRobotPose pose, double distance) {}

    // Vision update as an atomic ref so its thread safe
    private AtomicReference<VisionUpdate> leftVisionUpdate = new AtomicReference<>();
    private AtomicReference<VisionUpdate> rightVisionUpdate = new AtomicReference<>();

    /** 
     * Creates a new BetterPhotonVision. 
     * 
     * @param drivetrain The drivetrain
     * */
    public BetterPhotonVision(Swerve drivetrain) {
        // Set the drivetrain to our drivetrain
        this.drivetrain = drivetrain;

        // Create the camera threads
        leftCameraThread = new CameraThread(ReefPose.LEFT);
        rightCameraThread = new CameraThread(ReefPose.RIGHT);

        // Start the camera threads
        leftCameraThread.start();
        rightCameraThread.start();
    }

    @Override
    public void periodic() {
        // Update vision
        // See its this simple :)
        updateVision();

        // Shuffleboard values to test if our photon vision is working
        // TODO: Remove these later on 
        LightningShuffleboard.setDouble("BetterPhotonVision", "LeftCamTagId", leftCameraThread.getTagId());
        LightningShuffleboard.setDouble("BetterPhotonVision", "RightCamTagId", rightCameraThread.getTagId());

        LightningShuffleboard.setDouble("BetterPhotonVision", "LeftCamDistanceToTag", leftCameraThread.getDistanceFromBestTag());
        LightningShuffleboard.setDouble("BetterPhotonVision", "RightCamDistanceToTag", rightCameraThread.getDistanceFromBestTag());
    }

    /**
     * Use this to update vision and add measurments to drivetrain
     * 
     * If we have to add more vision logic, it will be done in here since camera measurments are done seperatly
     * TODO --- Update this to only take the latest update
     */
    private void updateVision() {
        // Update the left vision
        VisionUpdate leftUpdate = leftVisionUpdate.getAndSet(null);
        if (leftUpdate != null) {
            drivetrain.addVisionMeasurement(leftUpdate.pose, leftUpdate.distance);
        }   

        // Update the right vision
        VisionUpdate rightUpdate = rightVisionUpdate.getAndSet(null);
        if (rightUpdate != null) {                
            drivetrain.addVisionMeasurement(rightUpdate.pose, rightUpdate.distance);
        }
    }

    // TODO --- Add any getters that could be useful
    
    /**
     * Thread that will run camera operations in parralel
     */
    private class CameraThread extends Thread {
        // Create the variables for the camera
        ReefPose cameraName;
        PhotonCamera camera;

        // Set up default pose and pose estimator
        EstimatedRobotPose pose = new EstimatedRobotPose(new Pose3d(), 0, List.of(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        PhotonPoseEstimator poseEstimator;

        // The average distance to the best tag
        double averageDistanceToTag = 0;

        // yay
        public CameraThread(ReefPose cameraName) {
            // Set camera name
            this.cameraName = cameraName;

            // Initialize the camera to have the right cam name based on its pose relative to the reef
            initCamera();

            // Initialize the pose estimator with the correct robot-to-camera offset
            poseEstimator = new PhotonPoseEstimator(
                VisionConstants.tagLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                this.cameraName == ReefPose.LEFT ? VisionConstants.robotLeftToCamera : VisionConstants.robotRightToCamera
            );
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }


        @Override 
        public void run() {
            // Pause before starting the thread
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                DataLogManager.log("[BetterPhotonVision] Failed to init sleep");
            }

            // The camera loop
            while (true) {
                // If the camera is still uninitialized, initialize it and try again
                if (camera == null) {
                    initCamera();
                } else {
                    // Try catch since there could possible be no results
                    try {
                        /* This gets all the results that have accumulated since we last called this
                        * Each result can have multiple tags and it represents a processed frame */
                        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

                        // Use these to get an accurate average distance of the results' best targets
                        double numberOfResults = 0;
                        double totalDistance = 0;
                        double averageDistance = 0;

                        // Variable to tell if any of our results have targets so we know if we should update vision
                        boolean hasSomeTarget = false;

                        // We traverse through each result becuase we have to proccess each one individually
                        for (PhotonPipelineResult result : results) {
                            // Determine if there are tags in the result that we have to ignore
                            if (shouldDoSingleTag(result)) { 
                                // We clear multitag result so our pose estimator doens't use it
                                // Clear multitag result since it contsins tags we don't want
                                result.multitagResult = Optional.empty();

                                // Filter our result's target list to get rid of tags we dont' want
                                result.targets.removeIf((targ) -> VisionConstants.TAG_IGNORE_LIST.contains((short) targ.getFiducialId()));
                            }

                            // Check if the result (frame) has targets
                            if (result.hasTargets()) {
                                // Get the best target from the result since the result can have multiple targets
                                var target = result.getBestTarget();

                                // Since we have a target set this to true
                                hasSomeTarget = true;

                                // If the pose ambiguity is over 0.2 then it should not be used. 
                                // Pose ambiguity is from 0-1: 0-> good, 1-> too high
                                if (target.getPoseAmbiguity() < 0.2) {
                                    // Get the target's distance from the camera
                                    double targetDistance = target.getBestCameraToTarget().getTranslation().getNorm();

                                    if (!(targetDistance > 4)) {
                                        // If the target is good add one to the total
                                        numberOfResults += 1;

                                        // Use our pose estimator to use our result to estimate our robot pose on the feild
                                        /* It returns an optional value so if it is not present, we use it as our new pose, 
                                        but if there are no tags log it and don't update pose*/
                                        // TODO --- idk if theres a better way to do this than just use the last result
                                        poseEstimator.update(result).ifPresentOrElse(((pose) -> this.pose = pose), () -> {
                                            DataLogManager.log("[BetterPhotonVision] No tags detected");
                                        });

                                        // Add the distance of our best target to total distance
                                        totalDistance += targetDistance;
                                    }
                                } else {
                                    // This gets logged if our pose ambiguity is too high
                                    DataLogManager.log("[BetterPhotonVision] Pose Ambiguity High");
                                }
                            };
                        }

                        // Get the average distance from all the best tags
                        averageDistance = numberOfResults > 0d ? totalDistance / numberOfResults : 0d;

                        // Set the global distance
                        this.averageDistanceToTag = averageDistance;

                        // We add a vision measurment to the drivetrain
                        if (hasSomeTarget) {
                            // Add our vision measurment to the queue
                            addVisionUpdate(pose, averageDistance);
                        }

                    } catch (Exception e) {
                        DataLogManager.log("[BetterPhotonVision] No results");
                    }
                }

                // Sleep for 25 miliseconds before next loop considering the cameara is updating at 15fps
                try {
                    Thread.sleep(25);
                } catch (InterruptedException e) {
                    DataLogManager.log("[BetterPhotonVision] Failed to sleep");
                }
            }
        }

        /**
         * Gets the id of the best tag of the latest result
         * @return
         */ @SuppressWarnings("removal")
        public double getTagId() {
            try {
                try {
                    return camera.getLatestResult().getBestTarget().getFiducialId();
                } catch (NullPointerException e) {
                    DataLogManager.log("[BetterPhotonVision] No targets found");
                    return -1;
                }
            } catch (IndexOutOfBoundsException e) {
                DataLogManager.log("[BetterPhotonVision] Can't get tag-- no results");
                return -1;
            }
        }

        /**
         * Get the distance from the best tag to the camera
         * @return
         */
        public double getDistanceFromBestTag() {
            return this.averageDistanceToTag;
        }

        /**
         * Initializes the camera based on the reef pose
         */
        private void initCamera() {
            camera = cameraName == ReefPose.LEFT 
                ? new PhotonCamera(VisionConstants.leftCamName) 
                : new PhotonCamera(VisionConstants.rightCamName);
        }

        /**
         * Gets whether we should use single tag
         * 
         * @param result Some photon pipeline result
         * @return Returns true if our multitag result has tags that we want to have ignored
         */
        private boolean shouldDoSingleTag(PhotonPipelineResult result) {
            return result.getMultiTagResult()
                .map((r) -> r.fiducialIDsUsed.stream()
                .anyMatch(VisionConstants.TAG_IGNORE_LIST::contains))
                .orElse(false);
        }

        /**
         * Sets the latest update
         * 
         * @param pose The estimated robot pose
         * @param distance The distance from the target
         */
        private void addVisionUpdate(EstimatedRobotPose pose, double distance) {
            if (cameraName.equals(ReefPose.LEFT)) {
                leftVisionUpdate.set(new VisionUpdate(pose, distance));
            } else if (cameraName.equals(ReefPose.RIGHT)) {
                rightVisionUpdate.set(new VisionUpdate(pose, distance));
            }
        }
    }
}
