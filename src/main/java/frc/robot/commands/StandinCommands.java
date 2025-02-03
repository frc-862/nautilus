package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.FishingRodConstants.ROD_STATES;

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
        return new WaitCommand(0.5);
    }

    public static Command scoreCoral() {
        return new WaitCommand(0.5);
    }

    public static Command rodStow(){
        return new ParallelCommandGroup(moveElevator(FishingRodConstants.ELEVATOR_MAP.get(ROD_STATES.STOW)), moveWrist(FishingRodConstants.WRIST_MAP.get(ROD_STATES.STOW)));
    }

    public static Command rodSource() {
        return new ParallelCommandGroup(moveElevator(FishingRodConstants.ELEVATOR_MAP.get(ROD_STATES.SOURCE)), moveWrist(FishingRodConstants.WRIST_MAP.get(ROD_STATES.SOURCE)));
    }

    public static Command rodL1(){
        return new ParallelCommandGroup(moveElevator(FishingRodConstants.ELEVATOR_MAP.get(ROD_STATES.L1)), moveWrist(FishingRodConstants.WRIST_MAP.get(ROD_STATES.L1)));
    }
    
    public static Command rodL2(){
        return new ParallelCommandGroup(moveElevator(FishingRodConstants.ELEVATOR_MAP.get(ROD_STATES.L2)), moveWrist(FishingRodConstants.WRIST_MAP.get(ROD_STATES.L2)));
    }

    public static Command rodL3(){
        return new ParallelCommandGroup(moveElevator(FishingRodConstants.ELEVATOR_MAP.get(ROD_STATES.L3)), moveWrist(FishingRodConstants.WRIST_MAP.get(ROD_STATES.L3)));
    }

    public static Command rodL4(){
        return new ParallelCommandGroup(moveElevator(FishingRodConstants.ELEVATOR_MAP.get(ROD_STATES.L4)), moveWrist(FishingRodConstants.WRIST_MAP.get(ROD_STATES.L3)));
    }

}
