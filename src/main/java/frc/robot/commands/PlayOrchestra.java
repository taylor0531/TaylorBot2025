package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;



public class PlayOrchestra extends Command {
  
 
  public PlayOrchestra(){
      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
        PlayMusic("MissionImpossible.chrp");
    
  }
  private void PlayMusic(String Filename){
        try (Orchestra orchestra = new Orchestra()) {
          orchestra.loadMusic(Filename);
          
          orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod0.angleMotorID,"Carnie"));
          orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod0.driveMotorID,"Carnie"));
          orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod1.angleMotorID,"Carnie"));
          orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod1.driveMotorID,"Carnie"));
          orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod2.angleMotorID,"Carnie"));
          orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod2.driveMotorID,"Carnie"));
          orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod3.angleMotorID,"Carnie"));
          orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod3.driveMotorID,"Carnie"));
          orchestra.play();
        }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //turn off music
      //orchestra.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
