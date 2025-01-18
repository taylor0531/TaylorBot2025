package frc.robot;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

//dt import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
   
    public static double driverDivisor = Constants.fastSpeed;

    /* Drive Controls */
    // robot move forwards / backwards
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    // robot moves left / right
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private boolean isFieldOriented = false;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton resetPoseButton = new JoystickButton(driver, XboxController.Button.kX.value);
    
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowFast = new JoystickButton(driver, XboxController.Button.kStart.value);

    private final JoystickButton playmusic = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton changeColors = new JoystickButton(driver, XboxController.Button.kA.value);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final CandleSubsystem s_Candle = new CandleSubsystem();

    /* Autonomous Commands */
    private static SendableChooser<Command> m_chooser = new SendableChooser<>();

    private Command m_exampleAuto = new exampleAuto(s_Swerve);
    
    public RobotContainer() {

        m_chooser.setDefaultOption("Move X 1 yard ", (new PathPlannerAuto("moveX_Auto")));
        m_chooser.addOption("Move Y 1 yard Auto", (new PathPlannerAuto("moveY_Auto")));
       //dt m_chooser.addOption("Move Robot Simple - forward 1 Meter", (new MoveRobotCommand(s_Swerve, 1, 0, 0, 0)));
       //dt m_chooser.setDefaultOption("Example Auto", m_exampleAuto);

        SmartDashboard.putData("Autonomous choices", m_chooser);

        /* Register commands for PathPlanner */
        //dt NamedCommands.registerCommand("Queue the Music", new PlayOrchestra().withTimeout(25));

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> (-driver.getRawAxis(translationAxis)),
                        () -> (-driver.getRawAxis(strafeAxis)),
                        () -> (-driver.getRawAxis(rotationAxis)),
                        () -> isFieldOriented));

        // Configure the button bindings
        configureButtonBindings();

        // PlayMusic("EyeOfTheTiger.chrp");_choos
        // PlayMusic("Portal.chrp");
        // PlayMusic("MissionImpossible.chrp");

    }

    public void changeFieldOriented() {
        isFieldOriented = !isFieldOriented;
        SmartDashboard.putBoolean("Field oriented is ", isFieldOriented);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        robotCentric.onTrue(new InstantCommand(() -> changeFieldOriented()));
        resetPoseButton.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));
        slowFast.onTrue(new InstantCommand(() -> {
            if (driverDivisor == Constants.fastSpeed) {
                driverDivisor = Constants.slowSpeed;
            } else {
                driverDivisor = Constants.fastSpeed;
            }
        }));


        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> (-driver.getRawAxis(translationAxis) * Math.abs(driver.getRawAxis(translationAxis))
                                / driverDivisor),
                        () -> (-driver.getRawAxis(strafeAxis) * Math.abs(driver.getRawAxis(strafeAxis))
                                / driverDivisor),
                        () -> (-driver.getRawAxis(rotationAxis) / 1.5),
                        () -> isFieldOriented));

        playmusic.onTrue(new InstantCommand(() -> PlayMusic("MissionImpossible.chrp")));

        changeColors.onTrue(new InstantCommand(() -> s_Candle.decrementAnimation()))    ;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        SmartDashboard.putString("Your Selected Autonomous Is", m_chooser.getSelected().getName());
        return m_chooser.getSelected();

    }

    public void PlayMusic(String Filename) {
        Orchestra orchestra = new Orchestra();
        orchestra.loadMusic(Filename);

        orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod0.angleMotorID, "Carnie"));
        orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod0.driveMotorID, "Carnie"));
        orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod1.angleMotorID, "Carnie"));
        orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod1.driveMotorID, "Carnie"));
        orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod2.angleMotorID, "Carnie"));
        orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod2.driveMotorID, "Carnie"));
        orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod3.angleMotorID, "Carnie"));
        orchestra.addInstrument(new TalonFX(Constants.Swerve.Mod3.driveMotorID, "Carnie"));
        orchestra.play();
    }
}
