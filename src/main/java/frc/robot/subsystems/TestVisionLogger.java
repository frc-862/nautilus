// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TestVisionLogger extends SubsystemBase {

    
    VideoCapture rightCam;
    VideoWriter rightWriter;
    Mat rightFrame;
    int i = 0;
    /** Creates a new VisionLogger. */
    public TestVisionLogger() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // get PV camera stream
        rightCam = new VideoCapture("http://10.8.62.15:1181/stream.mjpg");
        if (!rightCam.isOpened()) {
            System.out.println("Error: Could not open right camera.");
            return;
        }

        // publish to video file

        rightWriter = new VideoWriter("/home/lvuser/vision_logs/" + getMatchName() + System.currentTimeMillis() + ".avi",
            VideoWriter.fourcc('M', 'J', 'P', 'G'), 50d, new Size(1280, 720));
        if (!rightWriter.isOpened()) {
            System.out.println("Error: Could not open right video writer.");
            return;
        }

        rightFrame = new Mat();
    }

    @Override
    public void periodic() {
        if (!LightningShuffleboard.getBool("Photon Vision", "Log Vision", false)) return;
        try {
            logVision();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void logVision() {
        if (i > 199 || !LightningShuffleboard.getBool("Photon Vision", "Log Right Cam Video", false)) return; // limit to first 200 frames for testing becasuse of limited storage space on RIO
        if (rightCam.read(rightFrame)) 
            rightWriter.write(rightFrame);
        i++;
    }

    public String getMatchName() {
        if (!DriverStation.isFMSAttached()) return "NOFMS_";

        return DriverStation.getEventName() + "_" + DriverStation.getMatchType().toString().charAt(0) 
            + DriverStation.getMatchNumber() + "_";
    }
}
