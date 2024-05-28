// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.GamePieceVisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DynamicGamePieceGuider extends DeferredCommand {
  /** Creates a new DynamicGamePieceGuider. */
  public DynamicGamePieceGuider(SwerveSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, GamePieceVisionSubsystem gamePieceVisionSubsystem, CommandXboxController driverController) {
    super(() -> {
      if(!gamePieceVisionSubsystem.getEstimatedGamePieceLocations().isEmpty())
      {
        return new GamePieceGuider(drivetrainSubsystem, 
          intakeSubsystem, 
          ledSubsystem, 
          gamePieceVisionSubsystem, 
          driverController, 
          gamePieceVisionSubsystem
            .getEstimatedGamePieceLocations()
            .stream()
            .map((pose) -> new Pair<>(pose.toPose2d(), drivetrainSubsystem.getPose()))
            .min((posePair1, posePair2) -> { 
              return (int)((posePair1.getFirst().getTranslation().getDistance(posePair1.getSecond().getTranslation())
                      - posePair2.getFirst().getTranslation().getDistance(posePair2.getSecond().getTranslation()))
                      * 1000); 
            })
            .map((posePair) -> posePair.getFirst())
            .get()
        );
      }
      else
      {
        return new InstantCommand();
      }
    }, Set.of(drivetrainSubsystem, gamePieceVisionSubsystem));
  }
}
