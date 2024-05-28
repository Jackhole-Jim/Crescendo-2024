// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteCommand extends SequentialCommandGroup {
  
  private int intakeBlinkCounter = 0;
  /** Creates a new IntakeNote. */
  public IntakeNoteCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, SwerveSubsystem drivebase, CommandXboxController driverController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(intakeSubsystem.SetIntakeSpeedCommand(
        () -> Math.max(Constants.IntakeConstants.MINIMUM_DRIVETRAIN_INTAKE_SPEED_METERS_PER_SECOND, drivebase.getRobotVelocity().vxMetersPerSecond) * 3//make it so our intake runs at 3x the surface speed of the robot in the forward direction
      )
      .onlyWhile(() -> !intakeSubsystem.IsNotePresent())
    );
    addCommands(intakeSubsystem.StopIntakeCommand());
    addCommands(new InstantCommand(() -> {
          intakeBlinkCounter = 0;
        }
      )
    );
    addCommands(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)));
    addCommands(
      ledSubsystem.setPercentageLitCommand(1, Color.kOrange)
        .andThen(new WaitCommand(0.1))
        .andThen(ledSubsystem.setPercentageLitCommand(0, Color.kOrange))
        .andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> intakeBlinkCounter++))
        .repeatedly()
        .until(() -> intakeBlinkCounter >= 4)
    );
    addCommands(ledSubsystem.setPercentageLitCommand(1, Color.kOrange));
    addCommands(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }
}
