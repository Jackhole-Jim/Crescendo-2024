// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
private final CANSparkMax intakeUpper = new CANSparkMax(Constants.IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
private final CANSparkMax intakeLower = new CANSparkMax(Constants.IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    CANSparkBase.enableExternalUSBControl(true);
    intakeUpper.restoreFactoryDefaults();
    intakeUpper.getPIDController().setP(0);
    intakeUpper.getPIDController().setI(0);
    intakeUpper.getPIDController().setD(0);
    intakeUpper.getPIDController().setFF(Constants.IntakeConstants.UPPER_FF);
    intakeUpper.setIdleMode(IdleMode.kBrake);
    intakeUpper.setInverted(true);
    // intakeUpper.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.VELOCITY_CONVERSION_TOP);
    intakeUpper.burnFlash();
    
    intakeLower.restoreFactoryDefaults();
    // intakeLower.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.VELOCITY_CONVERSION_BOTTOM);
    intakeLower.getPIDController().setP(0);
    intakeLower.getPIDController().setI(0);
    intakeLower.getPIDController().setD(0);
    intakeLower.getPIDController().setFF(Constants.IntakeConstants.LOWER_FF);
    // intakeLower.getPIDController().setSmartMotionMaxVelocity(5500, 0);
    intakeLower.setIdleMode(IdleMode.kBrake);
    intakeLower.burnFlash();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/UpperSpeed", intakeUpper.getEncoder().getVelocity() / Constants.IntakeConstants.VELOCITY_CONVERSION_TOP);
    Logger.recordOutput("Intake/LowerSpeed", intakeLower.getEncoder().getVelocity() / Constants.IntakeConstants.VELOCITY_CONVERSION_BOTTOM);
    SmartDashboard.putNumber("IntakeSpeed2", SmartDashboard.getNumber("IntakeSpeed", 0));
    // This method will be called once per scheduler run
  }

  // private void SetIntakeSpeed (double speed){
  //   intakeUpper.set(speed);
  //   intakeLower.set(speed);
  // }

  public void SetIntakeSpeed(double metersPerSecond)
  {
    Logger.recordOutput("Intake/Setpoint", metersPerSecond);
    intakeUpper.getPIDController().setReference(metersPerSecond / Constants.IntakeConstants.VELOCITY_CONVERSION_TOP, CANSparkBase.ControlType.kVelocity);
    intakeLower.getPIDController().setReference(metersPerSecond / Constants.IntakeConstants.VELOCITY_CONVERSION_BOTTOM, CANSparkBase.ControlType.kVelocity);
  }
  
  public Command SetIntakeSpeedCommand(DoubleSupplier metersPerSecondSupplier)
  {
    return run(() -> {
      this.SetIntakeSpeed(metersPerSecondSupplier.getAsDouble());
    });
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
