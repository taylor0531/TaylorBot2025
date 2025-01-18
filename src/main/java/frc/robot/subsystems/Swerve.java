package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
//dt import com.pathplanner.lib.auto.AutoBuilder;
//dt import com.pathplanner.lib.config.PIDConstants;
//dt import com.pathplanner.lib.config.RobotConfig;
//dt import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers;

import com.revrobotics.spark.SparkMax;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SparkMax m_intakeMotor; 
    public double shootSpeed;
    public double m_rotation;
    public double m_xSpeed;
    public double m_ySpeed;
    public RobotConfig config; 

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID,"Carnie");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        //        gyro.setYaw(0);
        gyro.getConfigurator().setYaw(0);// is this right or the line above??

        //m_intakeMotor = new SparkMax(Constants.intakeMotor, CANSparkLowLevel.MotorType.kBrushless);
    

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        //copied this from 2023 competition robot (will this fix pathplanner metrics? nope)
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }

        //(old)swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        //removing getGyroYaw() from the above line and replacing it
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, gyro.getRotation2d(), getModulePositions());


        //for pathplanner
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        //RobotConfig config;
         try{ 
            config = RobotConfig.fromGUISettings();
         } catch (Exception e) {
        // Handle exception as needed
             e.printStackTrace();
         }

    
        //pathplanner...
        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotRelativeSpeeds, //get relative speeds
            this::driveRobotRelative, //set speed and drive
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ), config,  
            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this //reference to this subsystem to set requirements 
        );
         
        // for rev intake motor
        
    }

    //pathplanner
    public void resetOdometry(Pose2d pose) {
        //(old)swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        swerveOdometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    //pathplanner
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return new ChassisSpeeds
        //dkt - check + and - direction.
        // Also read these two sites and definitely review the "8 Step"
        //https://www.chiefdelphi.com/t/yagsl-and-pathplanner-help/452861/12
        //https://yagsl.gitbook.io/yagsl/configuring-yagsl/the-eight-steps    
        /** Velocity along the x-axis. (Fwd is +) */
            //public double vxMetersPerSecond;
      
            /** Velocity along the y-axis. (Left is +) */
            //public double vyMetersPerSecond;
        
            /** Represents the angular velocity of the robot frame. (CCW is +) */
            //public double omegaRadiansPerSecond;
            
            (Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()).vxMetersPerSecond, 
            Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()).vyMetersPerSecond, 
            //dkt - does this next line need to return degrees?
            //Math.toDegrees(omegaRadiansPerSecond)
            Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);
    }

    //pathplanner - dig into this method. dkt
    public void driveRobotRelative(ChassisSpeeds chassis) {
        
        SwerveModuleState[] state = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassis);
    
        setModuleStates(state);
      }
    

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean limelightButtonPressed) {

        m_rotation = rotation; 
        m_xSpeed = translation.getX();
        m_ySpeed = translation.getY();
        // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
        if(limelightButtonPressed)
        {
            //final var rot_limelight = limelight_aim_proportional();
            //rot = rot_limelight;
            m_rotation = limelight_aim_proportional();

            //final var forward_limelight = limelight_range_proportional();
            //xSpeed = forward_limelight;
            m_xSpeed = limelight_range_proportional();

            //while using Limelight, turn off field-relative driving.
            fieldRelative = false;
        }   

        //variable = (condition) ? expressionTrue :  expressionFalse;
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    //translation.getX(), 
                                    //translation.getY(), 
                                    //rotation, 
                                    m_xSpeed,
                                    m_ySpeed,
                                    m_rotation,
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    m_xSpeed,
                                    m_ySpeed,
                                    m_rotation
                                    //translation.getX(), 
                                    //translation.getY(), 
                                    //rotation)
                                )
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        // dkt - use gyro.getYaw().getValue() - this is the angle
        //   turn left and yaw goes negative; turning right and value goes up. 
     //   SmartDashboard.putNumber("pigeon yaw is ", gyro.getYaw().getValue());
        //SmartDashboard.putBoolean("is field oriented ...", fieldRelative);
        SmartDashboard.putNumber("get Heading is ", getHeading().getDegrees());
        
        
        
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            //System.out.println(" Inside setModuleState. setting desired state. "+mod.moduleNumber+" is "+mod.getPosition());
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition(); //(dkt - what are these values?)
        }
        return positions;
    }

    public Pose2d getPose() {
        SmartDashboard.putNumber("pose x is ", swerveOdometry.getPoseMeters().getX());

        SmartDashboard.putNumber("pose y is ", swerveOdometry.getPoseMeters().getY());
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        //(old)swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        swerveOdometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        //(old)swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        swerveOdometry.resetPosition(gyro.getRotation2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        gyro.setYaw(0);
        resetModulesToAbsolute();
    }

    //(old) public Rotation2d getGyroYaw() {
        //dkt - this returns the same value as getYaw but with the opposite sign
        //return Rotation2d.fromDegrees(gyro.getAngle());
    //(old)    return Rotation2d.fromDegrees(gyro.getYaw());
        
    //(old)}


    //public Double getGyroYawInDegrees(){
    //    System.out.println("inside get Gyro Yaw In Degrees ---"+gyro.getYaw().getValue());
    //    return gyro.getYaw().getValue();
    // }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    // Intake Motor
     public void intakeMotor(double shootSpeed) {
        System.out.println("intake motor is "+ m_intakeMotor.toString());
        m_intakeMotor.set(shootSpeed);
    }

    @Override
    public void periodic(){
        //(old)swerveOdometry.update(getGyroYaw(), getModulePositions());
        swerveOdometry.update(gyro.getRotation2d(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  
            //swerve encoder
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Encoder Degrees ", mod.getCANcoder().getDegrees()) ; 
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Encoder Rotations ", mod.getCANcoder().getRotations()) ;
            
            SmartDashboard.putNumber("   GYRO values are:  X ", gyro.getAccumGyroX().getValueAsDouble());
            SmartDashboard.putNumber("   GYRO values are:  Y ", gyro.getAccumGyroY().getValueAsDouble());
            SmartDashboard.putNumber("   GYRO values are: acceleration Rotation2d " ,gyro.getRotation2d().getRotations());

        }
    }
    
    //Added March 25
    double limelight_aim_proportional()
    {    
      // kP (constant of proportionality)
      // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
      // if it is too high, the robot will oscillate.
      // if it is too low, the robot will never reach its target
      // if the robot never turns in the correct direction, kP should be inverted.
      double kP = .035;
  
      // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
      // your limelight 3 feed, tx should return roughly 31 degrees.
      double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
  
      // convert to radians per second for our drive method
      targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;
  
      //invert since tx is positive when the target is to the right of the crosshair
      targetingAngularVelocity *= -1.0;
  
      return targetingAngularVelocity;
    }
  
    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    double limelight_range_proportional()
    {    
      double kP = .1;
      double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
      targetingForwardSpeed *= Constants.Swerve.maxSpeed;
      targetingForwardSpeed *= -1.0;
      return targetingForwardSpeed;
    }
}