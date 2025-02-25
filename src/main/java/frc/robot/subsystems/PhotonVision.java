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
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.Robot;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Tuple;

public class PhotonVision extends SubsystemBase {
    private VisionSystemSim visionSim;

    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;

    private SimCameraProperties cameraProp;
    private VisionTargetSim visionTarget;

    private Swerve drivetrain;

    private CameraThread leftThread;
    private CameraThread rightThread;

    public PhotonVision(Swerve drivetrain) {
        this.drivetrain = drivetrain;

        leftThread = new CameraThread(Camera.LEFT);
        rightThread = new CameraThread(Camera.RIGHT);

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

            leftCameraSim = new PhotonCameraSim(leftThread.getCameraObject(), cameraProp);
            rightCameraSim = new PhotonCameraSim(rightThread.getCameraObject(), cameraProp);

            visionSim.addCamera(leftCameraSim, VisionConstants.robotLeftToCamera);
            visionSim.addCamera(rightCameraSim, VisionConstants.robotRightToCamera);

            // Enable the raw and processed streams. These are enabled by default.
            leftCameraSim.enableRawStream(true);
            leftCameraSim.enableProcessedStream(true);

            rightCameraSim.enableRawStream(true);
            rightCameraSim.enableProcessedStream(true);

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

        /*
         * if pipeline is switched out of 3d mode, the camera's thread should be paused, then resumed when switched back
         * this is to prevent the camera from trying to update the pose when it's unable to
         * error handling should catch this, but it's better to be safe than sorry
         *
         * threads should be paused with Thread.wait() and resumed with Thread.notify()
         */
    }

    /**
     * check if the camera has a target using networktables
     * @param camera - the camera to check
     * @return boolean - if the camera has a target
     */
    public boolean hasTarget(Camera camera) {
        if(isCameraInitialized(camera)) {
        return camera == Camera.LEFT
            ? leftThread.getCameraObject().getCameraTable().getEntry("hasTarget").getBoolean(false)
            : rightThread.getCameraObject().getCameraTable().getEntry("hasTarget").getBoolean(false);
        } else {
            return false;
        }
    }

     /**
     * check if the vision has a target using networktables
     * @return boolean - if the camera has a target
     */
    public boolean hasTarget() {
        return hasTarget(Camera.LEFT) || hasTarget(Camera.RIGHT);
    }

    /**
     * get the target's Y position in pixels
     * @param camera - the camera to check
     * @param offset - the offset to add to the value
     * @return double - the target's Y position in pixels
     */
    public double getTY(VisionConstants.Camera camera, double offset) {
        if(isCameraInitialized(camera)) {
            return camera == Camera.LEFT
                ? leftThread.getCameraObject().getCameraTable().getEntry("targetPixelsY").getDouble(0) + offset
                : rightThread.getCameraObject().getCameraTable().getEntry("targetPixelsY").getDouble(0) + offset;
        } else {
            return 0;
        }
    }

    /**
     * get the target's X position in pixels
     * @param camera - the camera to check
     * @param offset - the offset to add to the value
     * @return double - the target's X position in pixels
     */
    public double getTX(Camera camera, double offset) {
        if(isCameraInitialized(camera)) {
            return camera == Camera.LEFT
                ? leftThread.getCameraObject().getCameraTable().getEntry("targetPixelsX").getDouble(0) + offset
                : rightThread.getCameraObject().getCameraTable().getEntry("targetPixelsX").getDouble(0) + offset;
        } else {
            return 0;
        }
    }

    /**
     * get the target's tag number
     * @param camera - the camera to check
     * @return FiducialID of tag with least ambiguity (-1 if no tag found)
     */
    public int getTagNum(VisionConstants.Camera camera) {
        //this is performed independently of the thread, mainly because its a simple operation and happens regardless of pose
        try {
            if(!hasTarget(camera)){
                throw new Exception("No target found");
            } else {
                switch(camera){
                    case LEFT:
                        return leftThread.getCameraObject().getLatestResult().getBestTarget().getFiducialId();

                    case RIGHT:
                        return rightThread.getCameraObject().getLatestResult().getBestTarget().getFiducialId();

                    default:
                        throw new IllegalArgumentException("Invalid camera");
                }
            }
        } catch (Exception e) {
            DataLogManager.log("[PhotonVision] ERROR: NO TAG FOUND");
            return -1;
        }
    }

    /**
     * get the target's tag number
     * @param camera - the camera to check
     * @return FiducialID of tag with least ambiguity (-1 if no tag found)
     */
    public Transform3d getTransformToTag(Camera camera) throws Exception {
        //this is performed independently of the thread, mainly because its a simple operation and happens regardless of pose
        if(!hasTarget(camera)){
            throw new Exception("No target found");
        } else {
            switch(camera){
                case LEFT:
                    return leftThread.getCameraObject().getLatestResult().getBestTarget().bestCameraToTarget;
                case RIGHT:
                    return rightThread.getCameraObject().getLatestResult().getBestTarget().bestCameraToTarget;

                default:
                    throw new IllegalArgumentException("Invalid camera");
            }
        }
    }

    private boolean isCameraInitialized(Camera camName) {
        return camName == Camera.LEFT ? leftThread.cameraInitialized : rightThread.cameraInitialized;
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
        LightningShuffleboard.send("Vision", "Field_SIM", visionSim.getDebugField());
    }

    private synchronized void updateVision(Camera caller) {
        Tuple<EstimatedRobotPose, Double> leftUpdates = leftThread.getUpdates();
        Tuple<EstimatedRobotPose, Double> rightUpdates = rightThread.getUpdates();

        // LightningShuffleboard.setDouble("Vision", "left dist", leftUpdates.v);
        // LightningShuffleboard.setDouble("Vision", "right dist", rightUpdates.v);

        // prefer the camera that called the function (has known good values)
        // if the other camera has a target, prefer the one with the lower distance to best tag
        switch (caller) {
            case LEFT:
                if(rightThread.hasTarget() && rightUpdates.v < leftUpdates.v) {
                    drivetrain.addVisionMeasurement(rightUpdates.k, rightUpdates.v);
                } else {
                    drivetrain.addVisionMeasurement(leftUpdates.k, leftUpdates.v);
                }
            break;
            case RIGHT:
                if(leftThread.hasTarget() && leftUpdates.v < rightUpdates.v) {
                    drivetrain.addVisionMeasurement(leftUpdates.k, rightUpdates.v);
                } else {
                    drivetrain.addVisionMeasurement(rightUpdates.k, rightUpdates.v);
                }
            break;
        }
    }


    private class CameraThread extends Thread {
        private EstimatedRobotPose pose = new EstimatedRobotPose(new Pose3d(), 0, List.of(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        private PhotonPoseEstimator poseEstimator;
        private PhotonCamera camera;
        private Double averageDistance = 0d;
        private Camera camName;
        private Tuple<EstimatedRobotPose, Double> updates;
        private boolean hasTarget = false;
        private AprilTagFieldLayout tags;

        public boolean cameraInitialized = false;

        CameraThread(Camera camName) {
            this.camName = camName;

            initializeCamera();

            poseEstimator = new PhotonPoseEstimator(VisionConstants.tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camName == Camera.LEFT ? VisionConstants.robotLeftToCamera : VisionConstants.robotRightToCamera);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            tags = poseEstimator.getFieldTags();

            if(!DriverStation.getAlliance().isEmpty()) {
                switch (DriverStation.getAlliance().get()) {
                    case Blue:
                        tags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                        break;
                
                    case Red:
                        tags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                        break;

                    default:
                        DataLogManager.log("[PhotonVision]: Failed to get driver station alliance");
                        break;
                }
            }

            poseEstimator.setFieldTags(tags);

        }

        @Override
        public void run() {
            try {
                sleep(500);
            } catch (InterruptedException e) {
                DataLogManager.log(camName.toString() + " sleep inital failed");
            }
            while (true) {
                if(!cameraInitialized) {
                    initializeCamera();
                } else {
                    try {
                        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
                        double numberOfResults = results.size(); //double to prevent integer division errors
                        double totalDistances = 0;
                        boolean hasTarget = false;

                        //fundamentally, this loop updates the pose and distance for each result. It also logs the data to shuffleboard
                        //this is done in a thread-safe manner, as global variables are only updated at the end of the loop (no race conditions)
                        for (PhotonPipelineResult result : results) {
                            if (result.hasTargets()) {
                                // the local hasTarget variable will turn true if ANY PipelineResult within this loop has a target
                                hasTarget = true;

                                if(!(result.getBestTarget().getPoseAmbiguity() > 0.5)) {                                
                                    poseEstimator.update(result).ifPresentOrElse((pose) -> this.pose = pose,
                                            () -> DataLogManager.log("[PhotonVision] ERROR: " + camName.toString() + " pose update failed"));
                                } else {
                                    DataLogManager.log("[PhotonVision] WARNING: " + camName.toString() + " pose ambiguity is high");
                                }
                                // grabs the distance to the best target (for the latest set of result)
                                totalDistances += result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();

                                // LightningShuffleboard.setBool("Vision", camName.toString() + " targets found", !result.targets.isEmpty());
                                // LightningShuffleboard.setPose2d("Vision", camName.toString() + " pose", pose.estimatedPose.toPose2d());
                                // LightningShuffleboard.setDouble("Vision", camName.toString() + " Timestamp", result.getTimestampSeconds());
                                // LightningShuffleboard.setBool("Vision", camName.toString() + " hasTarget", true);
                            } else {
                                //note there is no hasTarget = false here, as the variable should not be set to false if a target was found in a previous iteration
                                // LightningShuffleboard.setBool("Vision", camName.toString() + " hasTarget", false);
                                // LightningShuffleboard.setBool("Vision", camName.toString() + " functional", true);
                            }
                        }

                        // averages distance over all results
                        averageDistance = totalDistances / numberOfResults;
                        updates = new Tuple<EstimatedRobotPose, Double>(pose, averageDistance);
                        this.hasTarget = hasTarget;
                        if (hasTarget) {
                            updateVision(camName);
                        }
                    } catch (IndexOutOfBoundsException e) {
                        // if there are no results,
                        LightningShuffleboard.setBool("Vision", camName.toString() + " functional", false);
                        LightningShuffleboard.setBool("Vision", camName.toString() + " hasTarget", false);
                        this.hasTarget = false;
                    }
                    try {
                        sleep(5);
                    } catch (InterruptedException e) {
                        DataLogManager.log(camName.toString() + " sleep failed");
                    }
                }
            }
        }

        /**
         * Returns the most recent pose and average distance to the best target <p>
         *
         * using a tuple as a type-safe alternative to the classic "return an array" (i hate java) <p>
         * this is also thread-safe, and will onlu return the most recent values from the same timestamp <p>
         *
         * @return Tuple<EstimatedRobotPose, Double> - the most recent pose and average distance to the best target
         */
        public Tuple<EstimatedRobotPose, Double> getUpdates() {
            return updates;
        }

        /**
         * Returns if the camera (during latest loop cycle) has a target
         *
         * This is separate from the value on the NetworkTables, as this value is updated for the entire loop cycle <p>
         * there is an edge case where a target is found, but the pose is not updated <p>
         * likewise, there is an edge case where a target is found and then lost, but the pose is updated <p>
         * to solve this, the hasTarget value is marked as true iff any update has been sent to the estimator <p>
         * @ImplNote this is NOT strictly synchronized with the value from {@link #getUpdates()}; be careful when using this value. should use PhotonVision's hasTarget() function for most cases
         * @return has a target or not
         */
        public boolean hasTarget() {
            return hasTarget;
        }

        public PhotonCamera getCameraObject() {
            return camera;
        }

        private void initializeCamera() {
            try {
                camera = new PhotonCamera(camName == Camera.LEFT ? VisionConstants.leftCamName : VisionConstants.rightCamName);
                cameraInitialized = true;
            } catch(Exception e) {
                System.out.println("warning: camera not initialized");
            }
        }
    }



    //keeping these here to prevent build errors, don't mind em for now
    //TODO: remove these when the time comes
    public double getLeftTY() {
        return getTY(Camera.LEFT, 0);
    }

    public double getLeftTX() {
        return getTX(Camera.LEFT, 0);
    }

    public int getLeftTagNum() {
        return getTagNum(Camera.LEFT);
    }

    public boolean leftHasTarget() {
        return hasTarget(Camera.LEFT);
    }
}
