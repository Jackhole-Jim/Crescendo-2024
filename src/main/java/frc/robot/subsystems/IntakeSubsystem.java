// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
private final CANSparkMax intakeUpper = new CANSparkMax(Constants.IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
private final CANSparkMax intakeLower = new CANSparkMax(Constants.IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeUpper.restoreFactoryDefaults();
    intakeUpper.getPIDController().setP(0);
    intakeUpper.getPIDController().setI(0);
    intakeUpper.getPIDController().setD(0);
    intakeUpper.getPIDController().setFF(0);
    intakeUpper.setIdleMode(IdleMode.kBrake);
    intakeUpper.setInverted(true);
    intakeUpper.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.VELOCITY_CONVERSION_TOP);
    
    intakeLower.restoreFactoryDefaults();
    intakeLower.getPIDController().setP(0);
    intakeLower.getPIDController().setI(0);
    intakeLower.getPIDController().setD(0);
    intakeLower.getPIDController().setFF(0);
    intakeLower.setIdleMode(IdleMode.kBrake);
    intakeLower.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.VELOCITY_CONVERSION_BOTTOM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // private void SetIntakeSpeed (double speed){
  //   intakeUpper.set(speed);
  //   intakeLower.set(speed);
  // }

  public void SetIntakeSpeed(double metersPerSecond)
  {
    intakeUpper.getPIDController().setReference(metersPerSecond, CANSparkMax.ControlType.kVelocity);
    intakeLower.getPIDController().setReference(metersPerSecond, CANSparkMax.ControlType.kVelocity);
  }
  
  public Command SetIntakeSpeedCommand(double metersPerSecond)
  {
    return runOnce(() -> {
      this.SetIntakeSpeed(metersPerSecond);
    });
  }

  public void StopIntake()
  {
    intakeUpper.getPIDController().setReference(0, CANSparkMax.ControlType.kVoltage);
    intakeLower.getPIDController().setReference(0, CANSparkMax.ControlType.kVoltage);
  }

  public Command StopIntakeCommand(){
    return runOnce(() -> {
      this.StopIntake();
    });
  }
}
