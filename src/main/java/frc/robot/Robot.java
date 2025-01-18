// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
   
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //dkt
  PowerDistribution m_PowerDistribution = new PowerDistribution(1, ModuleType.kRev);
  private double totalCurrent;
  private double totalPower;
  private double totalEnergy;
  private double current1;
  private double current2;
  private double current3;
  private double current4;
  private double current5;
  private double current6;
  private double current7;
  private double current8;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
   
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Get the total current of all channels.
    totalCurrent = m_PowerDistribution.getTotalCurrent();
    //SmartDashboard.putNumber("Total Current from PDH is ", totalCurrent);

    // Get the total power of all channels.
    // Power is the bus voltage multiplied by the current with the units Watts.
     totalPower = m_PowerDistribution.getTotalPower();
    //SmartDashboard.putNumber("Total Power from PDH is ", totalPower);

    // Get the total energy of all channels.
    // Energy is the power summed over time with units Joules.
     totalEnergy = m_PowerDistribution.getTotalEnergy();
    //SmartDashboard.putNumber("Total Energy from PDH is ", totalEnergy); 

    // Get the current going through channel 7, in Amperes.
    // The PDP returns the current in increments of 0.125A.
    // At low currents the current readings tend to be less accurate.
    current1 = m_PowerDistribution.getCurrent(1);
    //SmartDashboard.putNumber("Current Channel 1", current1);
    current2 = m_PowerDistribution.getCurrent(2);
    //SmartDashboard.putNumber("Current Channel 2", current2);
    current3 = m_PowerDistribution.getCurrent(3);
    //SmartDashboard.putNumber("Current Channel 3", current3);
    current4 = m_PowerDistribution.getCurrent(4);
    //SmartDashboard.putNumber("Current Channel 4", current4);
    current5 = m_PowerDistribution.getCurrent(5);
    //SmartDashboard.putNumber("Current Channel 5", current5);
    current6 = m_PowerDistribution.getCurrent(6);
    //SmartDashboard.putNumber("Current Channel 6", current6);
    current7 = m_PowerDistribution.getCurrent(7);
    //SmartDashboard.putNumber("Current Channel 7", current7);
    current8 = m_PowerDistribution.getCurrent(8);
    //SmartDashboard.putNumber("Current Channel 8", current8);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
