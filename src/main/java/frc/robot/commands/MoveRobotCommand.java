package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveRobotCommand extends SequentialCommandGroup{
    public MoveRobotCommand(Swerve s_Swerve, double distanceX, double distanceY, double goThroughX, double goThroughY){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    3, // trying 3 meters per second  :) 
                    //Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                
                List.of(
                    new Translation2d(goThroughX,goThroughY)
                ), 
   
                //new Pose2d(0, Units.feetToMeters(distance), new Rotation2d(0)), 
                // distance is already in meters
                new Pose2d(distanceX, distanceY, new Rotation2d(0)), 
                
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                //new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                //1,0,0 = .866; 2,0,0 = .9279; 4,0,0 = .9804; 5,0,0=.9923 ; 5.5,0,0=.99817
                //      5.75,0,0=.99857;  5.8,0,0=1.000036
                new PIDController(50.0, 14, 0), //p was 5.8, i was 0.0
                //new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                //1,0,0= ?; 4,0,0=-1.000039
                new PIDController(5.8, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}