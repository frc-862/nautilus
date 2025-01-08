package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class StandinCommands {
    /*
     * move elevator
     * move wrist
     * move algae collector
     * intake coral
     * intake algae
     */

     /**
      * Command will end after an approximation of how long it will take to move the elevator to a given position
      * @param position between 30 and 90 inches
      * @return command
      */
    public static Command moveElevator(double position) {
        return new WaitCommand(position * 0.05); //assume ~20in/s
    }

    public static Command moveWrist(double position) {
        return new WaitCommand(position * 0.0002222d); //assumed frequency was 1/4500 
    }

    public static Command moveAlgaeCollector() {
        return new WaitCommand(1);
    }

    public static Command intakeCoral() {
        //random number between 0.5 and 2s
        return new WaitCommand(0);
    }
}
