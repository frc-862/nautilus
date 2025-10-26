package frc.robot.subsystems.visionLogging;

import org.opencv.core.Size;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

public class LoggedCamera {
    
    private String cameraName;
    private String port;
    private String coprocessorName;
    private String coprocessorIP;
    private String robotName;
    private double FPS = 0.0;
    private Size resolution;
    private String videoPath;
    private String append;
    private String prepend;
    private Notifier notifier;
    private boolean loggerInitialized = false;

    /**
     * Stores information about a camera to be logged. Used to store video.
     * Name will be part of the video file name.
     * @param name
     * @param port
     */
    public LoggedCamera(String name, String port){
        this.cameraName = name;
        this.port = port;
    }

    /**
     * Stores information about a camera to be logged. Used to store video.
     * @param port
     */
    public LoggedCamera(String port){
        this.port = port;
    }

    /**
     * Sets the camera name information.
     * Camera name will be part of the video file name.
     * Defaults to no name.
     * @param name
     * @return this for chaining calls
     */
    public LoggedCamera withName(String name){
        this.cameraName = name;
        return this;
    }

    /**
     * Sets the coprocessor identifier information.
     * Defaults to no name and 10.8.62.15
     * @param coprocessorName
     * @param coprocessorIP
     * @return this for chaining calls
     */
    public LoggedCamera withCoprocessorIdentifier(String coprocessorName, String coprocessorIP){
        this.coprocessorName = coprocessorName;
        this.coprocessorIP = coprocessorIP;
        return this;
    }

    /**
     * Sets the robot name information.
     * Robot name will be part of the video file name.
     * Defaults to identifier set in VisionLogger.
     * Defaults to no robot identifier if no identifier is set in VisionLogger.
     * @param robotName
     * @return this for chaining calls
     */
    public LoggedCamera withRobotIdentifier(String robotName){
        this.robotName = robotName;
        return this;
    }

    /**
     * Sets the frames per second information that the video will be created at.
     * Defaults to 30 FPS.
     * @param FPS
     * @return this for chaining calls
     */
    public LoggedCamera withFPS(double FPS){
        this.FPS = FPS;
        return this;
    }

    /**
     * Sets the resolution information.
     * Defaults to 1280x720.
     * @param resolution
     * @return this for chaining calls
     */
    public LoggedCamera withResolution(Size resolution){
        this.resolution = resolution;
        return this;
    }

    /**
     * Sets the video file path information.
     * If not set, will check VisionLogger for path.
     * Defaults to /home/lvuser/vision_logs/ if not set in VisionLogger.
     * @param videoPath
     * @return this for chaining calls
     */
    public LoggedCamera withVideoPath(String videoPath){
        this.videoPath = videoPath;
        return this;
    }

    /**
     * Appends this string to the end of the video file name but prior to match name and time stamp.
     * @param append
     * @return this for chaining calls
     */
    public LoggedCamera withAppend(String append){
        this.append = append;
        return this;
    }

    /**
     * Prepends this string to the beginning of the video file name.
     * @param prepend
     * @return this for chaining calls
     */
    public LoggedCamera withPrepend(String prepend){
        this.prepend = prepend;
        return this;
    }

    private String getMatchName() {
        if (!DriverStation.isFMSAttached()) return "NOFMS_";

        return DriverStation.getEventName() + "_" + DriverStation.getMatchType().toString().charAt(0) 
            + DriverStation.getMatchNumber() + "_";
    }

    /**
     * @return filename constructed from the various name components
     * ex. nautilus_photonvision_rightCam_MSC_Q23_17021234123.avi
     */
    public String getCameraName() {
        String fileName = "";

        if (prepend != null)
            fileName += prepend + "_";

        if (robotName != null)
            fileName += robotName + "_";
        else if (VisionLogger.getRobotName() != null)
            fileName += VisionLogger.getRobotName() + "_";

        if (coprocessorName != null)
            fileName += coprocessorName + "_";

        if (cameraName != null)
            fileName += cameraName + "_";
        
        if (append != null)
            fileName += append + "_";
        
        fileName += getMatchName() + System.currentTimeMillis() + ".avi";

        return fileName;
    }

    /**
     * @return path to the camera stream
     */
    public String getCameraPath() {
        String path = "http://";
        
        if (coprocessorIP != null)
            path += coprocessorIP;
        else
            path += "10.8.62.15";

        path += ":" + port + "/stream.mjpg";

        return path;
    }

    /**
     * @return frames per second for logging
     * Defaults to 30 FPS if not set.
     */
    public double getFPS() {
        if (FPS == 0.0)
            return 30.0;
        return FPS;
    }

    /**
     * @return resolution for logging
     * Defaults to 1280x720 if not set.
     */
    public Size getResolution() {
        if (resolution == null)
            return new Size(1280, 720);
        return resolution;
    }

    /**
     * @return path where video will be saved
     * If not set, will check VisionLogger for path.
     * Defaults to /home/lvuser/vision_logs/ if not set in VisionLogger.
     */
    public String getVideoPath() {
        if (videoPath != null)
            return videoPath;
        else if (VisionLogger.getVideoPath() != null)
            return VisionLogger.getVideoPath();
        return videoPath;
    }

    /**
     * store the notifier used for logging
     * @param notifier
     */
    public void setNotifier(Notifier notifier) {
        this.notifier = notifier;
    }

    /**
     * @return notifier used for logging
     */
    public Notifier getNotifier() {
        return notifier;
    }

    /**
     * @return whether the logger has been initialized
     */
    public boolean isLoggerInitialized() {
        return loggerInitialized;
    }

    /**
     * @param loggerInitialized whether the logger has been initialized
     */
    public void setLoggerInitialized(boolean loggerInitialized) {
        this.loggerInitialized = loggerInitialized;
    }
}
