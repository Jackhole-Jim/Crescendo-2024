// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PreSpoolShooterCommand;
import frc.robot.commands.ShootNoteCommand;
import frc.robot.commands.StopDriveTrainCommand;
import frc.robot.commands.StopShooterSystem;
import frc.robot.commands.VisionOdometryHelper;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.AmpWhipperSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final AmpWhipperSubsystem ampWhipperSubsystem = new AmpWhipperSubsystem();

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;

  private int intakeBlinkCounter = 0;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    registerAndNameCommand("ShootNote", new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.SPEAKER_SHOOTING_SPEED_RPM));
    registerAndNameCommand("PreSpoolShooter", shooterSubsystem.StartShooterCommand(Constants.ShooterConstants.AUTO_SPEAKER_PRE_SPOOL_SPEED_RPM));
    registerAndNameCommand("IntakeNote", 
      intakeSubsystem.SetIntakeSpeedCommand(
        () -> Math.max(Constants.IntakeConstants.MINIMUM_DRIVETRAIN_INTAKE_SPEED_METERS_PER_SECOND, drivebase.getRobotVelocity().vxMetersPerSecond) * 3//make it so our intake runs at 3x the surface speed of the robot in the forward direction
      )
      .onlyWhile(() -> !intakeSubsystem.IsNotePresent())
      .andThen(intakeSubsystem.StopIntakeCommand())
      .andThen(new InstantCommand(() -> intakeBlinkCounter = 0))
      .andThen(
        ledSubsystem.setPercentageLitCommand(1, Color.kOrange)
        .andThen(new WaitCommand(0.1))
        .andThen(ledSubsystem.setPercentageLitCommand(0, Color.kOrange))
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> intakeBlinkCounter++))
        .repeatedly()
        .until(() -> intakeBlinkCounter >= 4)
      )
      .andThen(ledSubsystem.setPercentageLitCommand(1, Color.kOrange))
    );
    registerAndNameCommand("StopIntake", intakeSubsystem.StopIntakeCommand());
    registerAndNameCommand("StopDrivetrain", new StopDriveTrainCommand(drivebase));

    // Configure the trigger bindings
    
    configureBindings();

    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
    //                                                                                             OperatorConstants.LEFT_X_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(driverXbox.getRightX(),
    //                                                                                             OperatorConstants.RIGHT_X_DEADBAND),
    //                                                                driverXbox::getYButtonPressed,
    //                                                                driverXbox::getAButtonPressed,
    //                                                                driverXbox::getXButtonPressed,
    //                                                                driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRightX(),
    //     () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    
    LoggedPowerDistribution.getInstance(Constants.PDH_ID, ModuleType.kRev);

    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void registerAndNameCommand(String name, Command command)
  {
    NamedCommands.registerCommand(name, command);
    NamedCommands.getCommand(name).setName(name);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> scaleJoystick(MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 1),
        () -> scaleJoystick(MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 1),
        () -> scaleJoystick(-m_driverController.getRightX(), 2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> scaleJoystick(MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 1),
        () -> scaleJoystick(MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 1),
        () -> scaleJoystick(-m_driverController.getRightX(), 2));

    drivebase.setDefaultCommand(
        (!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim)
        .alongWith(new PreSpoolShooterCommand(drivebase, intakeSubsystem, shooterSubsystem))
        .alongWith(new VisionOdometryHelper(drivebase))
    );



    m_driverController.rightStick().onTrue(new InstantCommand(drivebase::zeroGyro));
    m_driverController
      .rightBumper()
      .whileTrue(NamedCommands.getCommand("ShootNote"))
      .onFalse(new StopShooterSystem(shooterSubsystem, intakeSubsystem, ledSubsystem));
    m_driverController
      .leftBumper()
      // .and(() -> !intakeSubsystem.IsNotePresent())
      .onTrue(NamedCommands.getCommand("IntakeNote"))
      .onFalse(NamedCommands.getCommand("StopIntake"));
      
    climberSubsystem.setDefaultCommand(climberSubsystem.setClimberSpeed(
      () -> m_driverController.getLeftTriggerAxis() - m_driverController.getRightTriggerAxis()
    ));
    // new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    // new JoystickButton(driverXbox,
    //                    2).whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    intakeSubsystem.setDefaultCommand(intakeSubsystem.SetIntakeSpeedCommand(
      () -> m_operatorController.getLeftTriggerAxis() - m_operatorController.getRightTriggerAxis()
    ));
    ampWhipperSubsystem.setDefaultCommand(
      ampWhipperSubsystem.setWhipperSpeed(
        () -> ampWhipperSubsystem.IsWhipperExtended() 
                ? -MathUtil.applyDeadband(m_operatorController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) 
                : 0
      )
    );
    m_operatorController.y().onTrue(ampWhipperSubsystem.extendActuators());
    m_operatorController.a().onTrue(ampWhipperSubsystem.retractActuators());
    m_operatorController
      .rightBumper()
      .whileTrue(new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.AMP_SHOOTING_SPEED_RPM))
      .onFalse(new StopShooterSystem(shooterSubsystem, intakeSubsystem, ledSubsystem));
    

    // SmartDashboard.putNumber("IntakeSpeed", 0);
    // m_driverController.b().onTrue(intakeSubsystem.SetIntakeSpeedCommand(() -> SmartDashboard.getNumber("IntakeSpeed", 0)));
  }

  public static double scaleJoystick(double value, double power)
  {
    return value >= 0 ? Math.abs(Math.pow(value, power)) : -Math.abs(Math.pow(value, power));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.get();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
