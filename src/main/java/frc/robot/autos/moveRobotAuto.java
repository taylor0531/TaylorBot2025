package frc.robot.autos;

import frc.robot.commands.MoveRobotCommand;
import frc.robot.commands.MoveRobotSimple;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class moveRobotAuto extends SequentialCommandGroup {
    public moveRobotAuto(Swerve s_Swerve){
        addCommands(
            // reset Odometry is already in the MoveRobotCommand class
            //new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
            // pass X (forward backwards), then pass Y (side side), go through X value, then go through Y value
            //new MoveRobotCommand(s_Swerve, 1,0),
            // moving 1 meters right backing up .5 meters
            //new MoveRobotCommand(s_Swerve, 0,-1.4478, -0.6, -0.7)

            // dkt 10-2024 .. moving forward 1 meter
            new MoveRobotSimple(s_Swerve, 1,0,0)
        );
    }
}