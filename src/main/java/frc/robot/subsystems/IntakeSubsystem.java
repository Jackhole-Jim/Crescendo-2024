// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
private final CANSparkMax Intakeupper = new CANSparkMax(14,MotorType.kBrushless);
private final CANSparkMax Intakelower = new CANSparkMax(13,MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    Intakeupper.restoreFactoryDefaults();
    Intakeupper.getPIDController().setP(0);
    Intakeupper.getPIDController().setI(0);
    Intakeupper.getPIDController().setD(0);
    Intakeupper.getPIDController().setFF(0);
    Intakeupper.setIdleMode(IdleMode.kBrake);
    Intakelower.setIdleMode(IdleMode.kBrake);
    Intakeupper.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void SetIntakeSpeed (double speed){
    Intakeupper.set(speed);
    Intakelower.set(speed);
  }
}
