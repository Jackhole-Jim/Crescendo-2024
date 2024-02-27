// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopShooterSystem extends SequentialCommandGroup {
  /** Creates a new StopShooterSystem. */
  public StopShooterSystem(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(shooterSubsystem.StopShooter());
    addCommands(shooterSubsystem.StopIndexMotor());
    addCommands(intakeSubsystem.StopIntakeCommand());
    addCommands(ledSubsystem.setPercentageLitCommand(0, Color.kBlack));
  }
}
