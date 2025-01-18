package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;


public class MoveRobotSimple extends Command {
  private Swerve s_Swerve;    
  private double translationSup;
  private double strafeSup;
  private double rotationSup;
  //private BooleanSupplier robotCentricSup;

  public MoveRobotSimple(Swerve s_Swerve, double translationSup, double strafeSup, double rotationSup){
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       /* Get Values, Deadband*/
       double translationVal = MathUtil.applyDeadband(translationSup, Constants.stickDeadband);
       double strafeVal = MathUtil.applyDeadband(strafeSup, Constants.stickDeadband);
       double rotationVal = MathUtil.applyDeadband(rotationSup, Constants.stickDeadband);

       /* Drive */
       s_Swerve.drive(
           new Translation2d(translationVal, strafeVal).times(Constants.Swerve.superSlowSpeed), 
           rotationVal * Constants.Swerve.maxAngularVelocity, 
           //!robotCentricSup.getAsBoolean(),
           false, 
           true,
           false
       );
       }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    

    s_Swerve.drive(
           new Translation2d(0, 0), 
           0 * Constants.Swerve.maxAngularVelocity, 
           //!robotCentricSup.getAsBoolean(),
           false, 
           false, //this may be true. not sure yet. 
           false
       );
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
