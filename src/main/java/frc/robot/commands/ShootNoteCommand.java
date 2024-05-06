// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteCommand extends SequentialCommandGroup {
  /** Creates a new ShootNoteCommand. */
  public ShootNoteCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, int shootingRPM, Supplier<Pose2d> robotPoseSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(shooterSubsystem.StartShooterCommand(shootingRPM));
    addCommands(
      ledSubsystem.setPercentageLitCommand(() -> shooterSubsystem.GetRPM() / shooterSubsystem.GetSetpoint(), Color.kRed)
      .until(()-> shooterSubsystem.IsAtSetpoint())
      .raceWith(new WaitCommand(1.5))//shoot anyways after 1.5 seconds of spool up, helps when battery voltage is too low to get shooter up to full speed
    );
    addCommands(shooterSubsystem.StartIndexMotor());
    addCommands(intakeSubsystem.SetIntakeSpeedCommand(Constants.IntakeConstants.SHOOTING_SPEED));
    addCommands(
      (new WaitCommand(2)
        .raceWith(
          new WaitUntilCommand(() -> !intakeSubsystem.IsNotePresent())//keep shooting until note is no longer present + 0.5 seconds or timeout after 2 seconds
          // .andThen(new WaitCommand(0.5))
        )
        .raceWith(//blink LEDs while note is shooting
          ledSubsystem.setPercentageLitCommand(0, Color.kGreen)
          .andThen(new WaitCommand(0.1))
          .andThen(ledSubsystem.setPercentageLitCommand(1, Color.kGreen))
          .andThen(new WaitCommand(0.1))
          .repeatedly()
        ))
        .alongWith(shoot(robotPoseSupplier))
    );
    addCommands(new StopShooterSystem(shooterSubsystem, intakeSubsystem, ledSubsystem));
  }

  

  public Command shoot(Supplier<Pose2d> robotPoseSupplier) {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(Constants.IntakeConstants.INDEXER_NOTE_TRANSFORM);
                  final boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red);
                  final Pose3d endPose =
                      new Pose3d(isRed ? Constants.FieldConstants.RED_SPEAKER_GOAL : Constants.FieldConstants.BLUE_SPEAKER_GOAL, startPose.getRotation());

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / Constants.ShooterConstants.SHOOTER_SHOT_SPEED;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "Shooter/ShootingNoteVisualizer",
                                new Pose3d[] {
                                  startPose.interpolate(endPose, timer.get() / duration)
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("Shooter/ShootingNoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }
}
