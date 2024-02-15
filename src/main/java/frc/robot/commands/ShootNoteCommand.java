// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteCommand extends SequentialCommandGroup {
  /** Creates a new ShootNoteCommand. */
  public ShootNoteCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(shooterSubsystem.StartShooter(Constants.ShooterConstants.SHOOTER_SHOOTING_SPEED_RPM));
    addCommands(new WaitUntilCommand(()-> shooterSubsystem.IsAtSetpoint()));
    addCommands(shooterSubsystem.StartIndexMotor());
    addCommands(intakeSubsystem.SetIntakeSpeedCommand(Constants.IntakeConstants.SHOOTING_SPEED));
    addCommands(new WaitCommand(2).raceWith(new WaitUntilCommand(() -> !intakeSubsystem.IsNotePresent())));
    addCommands(shooterSubsystem.StopShooter());
    addCommands(shooterSubsystem.StopIndexMotor());
    addCommands(intakeSubsystem.StopIntakeCommand());
  }
}
