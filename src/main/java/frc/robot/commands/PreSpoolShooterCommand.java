// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PreSpoolShooterCommand extends Command {
  private SwerveSubsystem mSwerveSubsystem;
  private IntakeSubsystem mIntakeSubsystem;
  private ShooterSubsystem mShooterSubsystem;
  /** Creates a new PreSpoolShooterCommand. */
  public PreSpoolShooterCommand(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    mSwerveSubsystem = swerveSubsystem;
    mIntakeSubsystem = intakeSubsystem;
    mShooterSubsystem = shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isTeleopEnabled() 
        && mIntakeSubsystem.IsNotePresent() 
        && mShooterSubsystem.GetSetpoint() == 0
        && (
          (DriverStation.getAlliance().get() == Alliance.Blue && mSwerveSubsystem.getPose().getTranslation().getDistance(Constants.FieldConstants.SPEAKER_POSE_BLUE) < Constants.ShooterConstants.PRE_SPOOL_DISTANCE_METERS)
          || (DriverStation.getAlliance().get() == Alliance.Red && mSwerveSubsystem.getPose().getTranslation().getDistance(Constants.FieldConstants.SPEAKER_POSE_RED) < Constants.ShooterConstants.PRE_SPOOL_DISTANCE_METERS)
        )
    )
    {
      mShooterSubsystem.StartShooterCommand(Constants.ShooterConstants.TELEOP_SPEAKER_PRE_SPOOL_SPEED_RPM).schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
