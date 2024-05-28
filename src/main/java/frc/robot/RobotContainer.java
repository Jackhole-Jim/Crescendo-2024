// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.NoteSimulator;
import frc.robot.commands.DynamicGamePieceGuider;
import frc.robot.commands.GamePieceGuider;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.PreSpoolShooterCommand;
import frc.robot.commands.ShootNoteCommand;
import frc.robot.commands.StopDriveTrainCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.StopShooterSystem;
import frc.robot.commands.VisionOdometryHelper;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.AmpWhipperSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.GamePieceVisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.List;
import java.util.Set;

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

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(() -> drivebase.getPose());

  private final AmpWhipperSubsystem ampWhipperSubsystem = new AmpWhipperSubsystem();

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private final GamePieceVisionSubsystem gamePieceVisionSubsystem = new GamePieceVisionSubsystem(() -> drivebase.getPose());

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    registerAndNameCommand("ShootNote", new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.SPEAKER_SHOOTING_SPEED_RPM, () -> drivebase.getPose()));
    // registerAndNameCommand("PreSpoolShooter", shooterSubsystem.StartShooterCommand(Constants.ShooterConstants.AUTO_SPEAKER_PRE_SPOOL_SPEED_RPM));
    registerAndNameCommand("IntakeNote", new IntakeNoteCommand(intakeSubsystem, ledSubsystem, drivebase, m_driverController));
    registerAndNameCommand("StopIntake", new StopIntakeCommand(intakeSubsystem, m_driverController));
    registerAndNameCommand("StopDrivetrain", new StopDriveTrainCommand(drivebase));
    registerAndNameCommand("LockDrivetrain", drivebase.lockDriveCommand());
    registerAndNameCommand("PreSpoolHelper", new PreSpoolShooterCommand(drivebase, intakeSubsystem, shooterSubsystem));
    registerAndNameCommand("VisionOdometryHelper", new VisionOdometryHelper(drivebase));

    registerAndNameCommand("IntakeAmpNote", new DeferredCommand(() -> new GamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController, (DriverStation.getAlliance().get() == Alliance.Blue ? Constants.FieldConstants.BLUE_1_NOTE : Constants.FieldConstants.RED_1_NOTE)), Set.of(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem)));
    registerAndNameCommand("IntakeSpeakerNote", new DeferredCommand(() -> new GamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController, (DriverStation.getAlliance().get() == Alliance.Blue ? Constants.FieldConstants.BLUE_2_NOTE : Constants.FieldConstants.RED_2_NOTE)), Set.of(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem)));
    registerAndNameCommand("IntakeSourceNote", new DeferredCommand(() -> new GamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController, (DriverStation.getAlliance().get() == Alliance.Blue ? Constants.FieldConstants.BLUE_3_NOTE : Constants.FieldConstants.RED_3_NOTE)), Set.of(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem)));
    registerAndNameCommand("IntakeCenterNote1", new GamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController, Constants.FieldConstants.CENTER_1_NOTE));
    registerAndNameCommand("IntakeCenterNote2", new GamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController, Constants.FieldConstants.CENTER_2_NOTE));
    registerAndNameCommand("IntakeCenterNote3", new GamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController, Constants.FieldConstants.CENTER_3_NOTE));
    registerAndNameCommand("IntakeCenterNote4", new GamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController, Constants.FieldConstants.CENTER_4_NOTE));
    registerAndNameCommand("IntakeCenterNote5", new GamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController, Constants.FieldConstants.CENTER_5_NOTE));
    
    registerAndNameCommand("DriveToPoseTest", drivebase.driveToPose(Constants.FieldConstants.CENTER_1_NOTE.toPose2d()));

    registerAndNameCommand("ShootNoteAmp", 
      new DeferredCommand(
        () -> { 
          return drivebase.driveToPose((DriverStation.getAlliance().get() == Alliance.Blue ? Constants.FieldConstants.BLUE_SHOOTING_POSES : Constants.FieldConstants.RED_SHOOTING_POSES).get(0)).asProxy()
          .andThen(drivebase.lockDriveCommand())
          .andThen(new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.SPEAKER_SHOOTING_SPEED_RPM, () -> drivebase.getPose()));
        }, Set.of(drivebase, shooterSubsystem, ledSubsystem, intakeSubsystem)
      )
    );
    
    registerAndNameCommand("ShootNoteCenter", 
      new DeferredCommand(
        () -> drivebase.driveToPose((DriverStation.getAlliance().get() == Alliance.Blue ? Constants.FieldConstants.BLUE_SHOOTING_POSES : Constants.FieldConstants.RED_SHOOTING_POSES).get(1))
          .andThen(drivebase.lockDriveCommand())
          .andThen(new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.SPEAKER_SHOOTING_SPEED_RPM, () -> drivebase.getPose()))
        , Set.of(drivebase, shooterSubsystem, intakeSubsystem, ledSubsystem)
      )
    );

    registerAndNameCommand("ShootNoteSource", 
      new DeferredCommand(
        () -> drivebase.driveToPose((DriverStation.getAlliance().get() == Alliance.Blue ? Constants.FieldConstants.BLUE_SHOOTING_POSES : Constants.FieldConstants.RED_SHOOTING_POSES).get(2))
          .andThen(drivebase.lockDriveCommand())
          .andThen(new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.SPEAKER_SHOOTING_SPEED_RPM, () -> drivebase.getPose()))
        , Set.of(drivebase, shooterSubsystem, intakeSubsystem, ledSubsystem)
      )
    );

    registerAndNameCommand("RunUntilNote", new WaitUntilCommand(() -> intakeSubsystem.IsNotePresent()));

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
    nameCommand(name, NamedCommands.getCommand(name));
  }

  public Command nameCommand(String name, Command command)
  {
    command.setName(name);
    return command;
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
        () -> scaleJoystick(MathUtil.applyDeadband(-m_driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND), 2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> scaleJoystick(MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 1),
        () -> scaleJoystick(MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 1),
        () -> scaleJoystick(-m_driverController.getRightX(), 2));

    drivebase.setDefaultCommand(
        nameCommand("DriveWithController", (!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim)
        // .alongWith(NamedCommands.getCommand("PreSpoolHelper"))
        .alongWith(new VisionOdometryHelper(drivebase)))
    );
    
    m_driverController
      .b()
      .and(() -> intakeSubsystem.IsNotePresent())
      .whileTrue(
        new DeferredCommand(() ->
          drivebase.driveToPose(
            (DriverStation.getAlliance().get() == Alliance.Blue ? Constants.FieldConstants.BLUE_SHOOTING_POSES : Constants.FieldConstants.RED_SHOOTING_POSES)
            .stream()
            .map((pose) -> new Pair<>(pose, drivebase.getPose()))
            .min((posePair1, posePair2) -> { 
              return (int)((posePair1.getFirst().getTranslation().getDistance(posePair1.getSecond().getTranslation())
                      - posePair2.getFirst().getTranslation().getDistance(posePair2.getSecond().getTranslation()))
                      * 1000); 
            })
            .map((posePair) -> posePair.getFirst())
            .get()
            // Constants.FieldConstants.BLUE_SHOOTING_POSES.get(0)
          ), Set.of(drivebase)
        )
        .andThen(new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.SPEAKER_SHOOTING_SPEED_RPM, () -> drivebase.getPose()))
    )
    .onFalse(new StopShooterSystem(shooterSubsystem, intakeSubsystem, ledSubsystem));

    m_driverController
      .x()
      // .and(() -> !intakeSubsystem.IsNotePresent())
      .whileTrue(new DynamicGamePieceGuider(drivebase, intakeSubsystem, ledSubsystem, gamePieceVisionSubsystem, m_driverController))
      .onFalse(new StopIntakeCommand(intakeSubsystem, m_driverController));      

    m_driverController
      .leftBumper()
      // .and(() -> !intakeSubsystem.IsNotePresent())
      .onTrue(new IntakeNoteCommand(intakeSubsystem, ledSubsystem, drivebase, m_driverController))
      .onFalse(new StopIntakeCommand(intakeSubsystem, m_driverController));
    m_driverController
      .rightBumper()
      .whileTrue(new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.SPEAKER_SHOOTING_SPEED_RPM, () -> drivebase.getPose()))
      .onFalse(new StopShooterSystem(shooterSubsystem, intakeSubsystem, ledSubsystem));
      
    m_driverController.rightStick().onTrue(new InstantCommand(drivebase::zeroGyro));
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
        () -> //ampWhipperSubsystem.IsWhipperExtended() 
               /* ?*/ -MathUtil.applyDeadband(m_operatorController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) 
                // : 0
      )
    );
    m_operatorController.y().onTrue(ampWhipperSubsystem.extendActuators());
    m_operatorController.a().onTrue(ampWhipperSubsystem.retractActuators());
    m_operatorController
      .rightBumper()
      .whileTrue(new ShootNoteCommand(shooterSubsystem, intakeSubsystem, ledSubsystem, Constants.ShooterConstants.AMP_SHOOTING_SPEED_RPM, () -> drivebase.getPose()))
      .onFalse(new StopShooterSystem(shooterSubsystem, intakeSubsystem, ledSubsystem));
    


    if(RobotBase.isSimulation())
    {
      NoteSimulator noteSimulator = new NoteSimulator(() -> drivebase.getPose(), () -> intakeSubsystem.GetSetPoint() > 0, () -> shooterSubsystem.GetIndexSetpoint() > 0);
      intakeSubsystem.SetBeamBreakSimulator(noteSimulator.noteBeamBreakSimulation());
      noteSimulator.schedule();
    }
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
    return autoChooser.get()
    // .handleInterrupt(() -> {
    //     System.out.println("Auto command interrupted");
    //   }
    // )
    ;//.alongWith(NamedCommands.getCommand("PreSpoolHelper"));
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public List<SubsystemBase> getSubsystems(){
    return List.of(drivebase, shooterSubsystem, intakeSubsystem, ampWhipperSubsystem, ledSubsystem, gamePieceVisionSubsystem);
  }
}
