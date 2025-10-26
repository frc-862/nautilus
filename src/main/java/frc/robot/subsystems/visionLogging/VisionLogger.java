package frc.robot.subsystems.visionLogging;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;

import edu.wpi.first.wpilibj.Notifier;

public class VisionLogger {
    
    private static LoggedCamera[] cameras;

    private static String robotName;
    private static String videoPath;
    private static boolean loggerError = false;

    /**
     * Adds cameras to be logged.
     * @param cameras
     */
    public static void addCameras(LoggedCamera...cameras) {
        VisionLogger.cameras = cameras;
    }

    /**
     * Adds a camera to be logged.
     * @param camera
     */
    public static void addCamera(LoggedCamera camera) {
        LoggedCamera[] newCameras = new LoggedCamera[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            newCameras[i] = cameras[i];
        }
        newCameras[cameras.length] = camera;
        cameras = newCameras;
    }

    /**
     * Starts logging for all cameras.
     * Initializes videoCapture and videoWriter for each camera.
     */
    public static void startLogging() {

        if (loggerError) return;

        for (LoggedCamera camera : cameras) {

            // start notifier if already initialized
            if (camera.isLoggerInitialized()) {
                camera.getNotifier().startPeriodic(1.0 / camera.getFPS());
            }

            // initialize videoCapture to get frames from camera stream
            VideoCapture videoCapture = new VideoCapture(camera.getCameraPath());
            if (!videoCapture.isOpened()) {
                System.out.println("Error: Could not open camera " + camera.getCameraName() + " at " + camera.getCameraPath());
                loggerError = true;
                return;
            }

            // initialize videoWriter to write frames to video file
            VideoWriter videoWriter = new VideoWriter(camera.getVideoPath(),
                    VideoWriter.fourcc('M', 'J', 'P', 'G'), camera.getFPS(), camera.getResolution());
            if (!videoWriter.isOpened()) {
                System.out.println("Error: Could not open video writer for camera " + camera.getCameraName() + " at " + camera.getVideoPath());
                loggerError = true;
                return;
            }

            // store frames to be written
            Mat frame = new Mat();

            // create notifier to periodically read frames and write to video
            Notifier notifier = new Notifier(() -> {
                try {
                    if (videoCapture.read(frame)) {
                        videoWriter.write(frame);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            });
            notifier.startPeriodic(1.0 / camera.getFPS());

            camera.setNotifier(notifier);
            camera.setLoggerInitialized(true);
        }
    }


    /**
     * Stops logging for all cameras.
     */
    public static void stopLogging() {

        if (loggerError) return;

        for (LoggedCamera camera : cameras) {
            Notifier notifier = camera.getNotifier();
            if (notifier != null) {
                notifier.stop();
            }

        }
    }

    /**
     * sets the robot name to be used in the video file names
     * Can be overridden by setting the robot name in LoggedCamera
     * Default is no name
     * @param name
     */
    public static void setRobotName(String name) {
        robotName = name;
    }

    /**
     * gets the robot name to be used in the video file names
     * Default is no name
     * @return
     */
    public static String getRobotName() {
        return robotName;
    }

    /**
     * sets the path where videos will be saved
     * can be overridden by setting the video path in LoggedCamera
     * Default is /home/lvuser/vision_logs/
     * @param path
     */
    public static void setVideoPath(String path) {
        videoPath = path;
    }

    /**
     * gets the path where videos will be saved
     * Default is /home/lvuser/vision_logs/
     * @return
     */
    public static String getVideoPath() {
        return videoPath != null ? videoPath : "/home/lvuser/vision_logs/";
    }

}
