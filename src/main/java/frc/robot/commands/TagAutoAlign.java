package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TagAutoAlign extends Command {
    private PhotonVision vision;
    private Swerve drivetrain;
  
    private PIDController controllerX = new PIDController(AutoAlignConstants.X_Kp, AutoAlignConstants.X_Ki, 
        AutoAlignConstants.X_Kd);

    private PIDController controllerY = new PIDController(AutoAlignConstants.Y_Kp, AutoAlignConstants.Y_Ki, 
        AutoAlignConstants.Y_Kd);

    private PIDController controllerR;

    private double dx_dt;
    private double dy_dt;
    private double dr_dt;

    private double TY;
    private double TX;
    private double txError;
    private double tyError;
    private double robotYaw;
    private double yawDiff;

    private int numTimesWithSameData = 0;
    private double[] lastData = new double[] {0, 0};

    private XboxController driver;

    /**
     * Used to align to Tag
     * will always use PID Controllers
     * @param vision
     * @param drivetrain
     */
    public TagAutoAlign(PhotonVision vision, Swerve drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    /**
     * used to align to tag
     * will use driver input for robot centric foreward movement in teleop
     * @param vision
     * @param drivetrain
     * @param driver
     */
    public TagAutoAlign (PhotonVision vision, Swerve drivetrain, XboxController driver){
        this(vision, drivetrain);
        this.driver = driver;
    }

    @Override
    public void initialize() {
        // zero velocity values

        controllerX = new PIDController(
            LightningShuffleboard.getDouble("TestAutoAlign", "X Kp", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "X Ki", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "X Kd", 0));

        controllerY = new PIDController(
            LightningShuffleboard.getDouble("TestAutoAlign", "Y Kp", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "Y Ki", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "Y Kd", 0));

        controllerR = new PIDController(
            LightningShuffleboard.getDouble("TestAutoAlign", "R Kp", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "R Ki", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "R Kd", 0));

        

        controllerX.setSetpoint(720);
        controllerX.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", "x tolerance", 0));
        // controllerX.enableContinuousInput(0, 360);

        controllerY.setSetpoint(0);
        controllerY.setTolerance(AutoAlignConstants.AutoAlignTolerance);
        

        controllerR.setSetpoint(0);
        controllerR.enableContinuousInput(0, 360);

    }

    @Override
    public void execute() {

        controllerX.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", "x tolerance", 0));

        if(!vision.hasTarget()){

            System.out.println("Error: Cannot See April Tag");
            cancel();
        }

        // update pitch and yaw values

        TY = vision.getTY();
        TX = vision.getTX();

        txError = TX - AutoAlignConstants.targetTX;

        robotYaw = MathUtil.inputModulus(drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 360);
        try {
            yawDiff = Math.sin(AutoAlignConstants.tagAngles.get(vision.getTagNum()) - robotYaw);
        } catch (Exception e) {
            System.out.println("Error: Cannot see April Tag");
            cancel();;
        }

        if(lastData[0] == TY && lastData[1] == TX){
            numTimesWithSameData++;

            if (numTimesWithSameData > 5){
                System.out.println("Error: Photon Vision is not updating");
                cancel();
            }
            
        } else {
            numTimesWithSameData = 0;
            lastData = new double[] {TY, TX};
        }

        // use pitch and yaw to calculate velocity values
        double Ks = LightningShuffleboard.getDouble("TestAutoAlign", "Ks", 0);

        dx_dt = controllerX.calculate(TX);
        if (!controllerX.atSetpoint()){
            if(Math.abs(dx_dt) < Ks){
                dx_dt = Ks * -Math.signum(txError);
            }
        } else{
            dx_dt = 0;
        }

        dy_dt = !DriverStation.isTeleop() ? -controllerY.calculate(TY) : // if in teleop use driver input for foreward movement unless driver is null
            (driver == null ? -controllerY.calculate(TY) : -driver.getLeftY());

        // dr_dt = -Math.floorMod((int) controllerR.calculate(rot), 360);
        dr_dt = controllerR.calculate(yawDiff);

        if(!controllerR.atSetpoint()){
            if(Math.abs(dr_dt) < Ks){
                dr_dt = Ks * ( (AutoAlignConstants.tagAngles.get(vision.getTagNum()) != 0) ? -Math.signum(txError) : 
                   (robotYaw > 180 ? -1 : 1) );
            } else {
                dr_dt = 0;
            }
        }
        

        if (!DriverStation.isFMSAttached()){
            LightningShuffleboard.setDouble("TestAutoAlign", "X speed", dx_dt);
            LightningShuffleboard.setDouble("TestAutoAlign", "Y speed", dy_dt);
            LightningShuffleboard.setDouble("TestAutoAlign", "R speed", dr_dt);

            LightningShuffleboard.setDouble("TestAutoAlign", "X error", txError);
            LightningShuffleboard.setDouble("TestAutoAlign", "Y error", tyError);
            LightningShuffleboard.setDouble("TestAutoAlign", "R error", yawDiff);
        }

        // give the new velocity values to the drivetrain
        drivetrain.setControl(DriveRequests.getRobotCentric(dx_dt, dy_dt, dr_dt));
    }



    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(DriveRequests.getRobotCentric(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(pitch) < AutoAlignConstants.AutoAlignTolerance && 
        //     Math.abs(yaw) < AutoAlignConstants.AutoAlignTolerance;
        return false;
    }
}
