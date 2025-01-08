package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class StandinCommands {
    /**
     * move elevator
     * move wrist
     * move algae collector
     * intake coral
     * intake algae
     */

     /**
      * Command will end after an approimation of how long it will take to move the elevator to a given position
      * @param position between 30 and 90 inches
      * @return
      */
    public static Command moveElevator(double position) {
        return new WaitCommand(position * 0.05); //assume ~20in/s
    }

    //TODO: complete implementation
}
