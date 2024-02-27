// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteCommand extends SequentialCommandGroup {
  /** Creates a new ShootNoteCommand. */
  public ShootNoteCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, int shootingRPM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(shooterSubsystem.StartShooterCommand(shootingRPM));
    addCommands(
      ledSubsystem.setPercentageLitCommand(() -> shooterSubsystem.GetRPM() / shooterSubsystem.GetSetpoint(), Color.kRed)
      .until(()-> shooterSubsystem.IsAtSetpoint())
    );
    addCommands(shooterSubsystem.StartIndexMotor());
    addCommands(intakeSubsystem.SetIntakeSpeedCommand(Constants.IntakeConstants.SHOOTING_SPEED));
    addCommands(
      new WaitCommand(2)
        .raceWith(
          new WaitUntilCommand(() -> !intakeSubsystem.IsNotePresent())//keep shooting until note is no longer present + 0.5 seconds or timeout after 2 seconds
          .andThen(new WaitCommand(0.5))
        )
        .raceWith(//blink LEDs while note is shooting
          ledSubsystem.setPercentageLitCommand(0, Color.kGreen)
          .andThen(new WaitCommand(0.1))
          .andThen(ledSubsystem.setPercentageLitCommand(1, Color.kGreen))
          .andThen(new WaitCommand(0.1))
          .repeatedly()
        )
    );
    addCommands(new StopShooterSystem(shooterSubsystem, intakeSubsystem, ledSubsystem));
  }
}
