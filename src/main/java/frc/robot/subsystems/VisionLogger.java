// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.opencv.videoio.Videoio;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionLogger extends SubsystemBase {

    VideoCapture rightCam;
    VideoWriter rightWriter;
    Mat rightFrame;
    int i = 0;
    /** Creates a new VisionLogger. */
    public VisionLogger() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // get PV camera stream
        rightCam = new VideoCapture("http://10.8.62.15:1181/stream.mjpg");
        if (!rightCam.isOpened()) {
            System.out.println("Error: Could not open right camera.");
            return;
        }

        double width = rightCam.get(Videoio.CAP_PROP_FRAME_WIDTH);
        width = (width >= 0) ? width : 1280;
        double height = rightCam.get(Videoio.CAP_PROP_FRAME_HEIGHT);
        height = (height >= 0) ? height : 720;

        // publish to video file

        rightWriter = new VideoWriter("/home/lvuser/vision_logs/" + getMatchName() + System.currentTimeMillis() + ".avi",
            VideoWriter.fourcc('M', 'J', 'P', 'G'), 50d, new Size(width, height));
        if (!rightWriter.isOpened()) {
            System.out.println("Error: Could not open right video writer.");
            return;
        }

        rightFrame = new Mat();
    }

    @Override
    public void periodic() {
        try {
            logVision();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void logVision() {
        if (i > 49) return; // limit to first 50 frames for testing becasuse of limited storage space on RIO
        if (rightCam.read(rightFrame)) rightWriter.write(rightFrame);
        i++;
    }

    public String getMatchName() {
        if (!DriverStation.isFMSAttached()) return "NOFMS_";
        
        return DriverStation.getEventName() + "_" + DriverStation.getMatchType().toString().charAt(0) 
            + DriverStation.getMatchNumber() + "_";
    }
}
