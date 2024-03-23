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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util;

public class IntakeSubsystem extends SubsystemBase {
private final CANSparkMax intakeUpper = new CANSparkMax(Constants.IntakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
private final CANSparkMax intakeLower = new CANSparkMax(Constants.IntakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
private final CANSparkMax intakeLeft = new CANSparkMax(Constants.IntakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
private final CANSparkMax intakeRight = new CANSparkMax(Constants.IntakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

private final AnalogInput noteBeamBreak = new AnalogInput(Constants.IntakeConstants.NOTE_BEAM_BREAK_CHANNEL);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    CANSparkBase.enableExternalUSBControl(true);
    intakeUpper.restoreFactoryDefaults();
    intakeUpper.getPIDController().setP(0);
    intakeUpper.getPIDController().setI(0);
    intakeUpper.getPIDController().setD(0);
    intakeUpper.getPIDController().setFF(Constants.IntakeConstants.UPPER_FF / Constants.IntakeConstants.VELOCITY_CONVERSION_TOP);
    intakeUpper.setIdleMode(IdleMode.kBrake);
    intakeUpper.setInverted(true);
    intakeUpper.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.VELOCITY_CONVERSION_TOP);
    intakeUpper.burnFlash();
    
    intakeLower.restoreFactoryDefaults();
    intakeLower.getEncoder().setVelocityConversionFactor(Constants.IntakeConstants.VELOCITY_CONVERSION_BOTTOM);
    intakeLower.getPIDController().setP(0);
    intakeLower.getPIDController().setI(0);
    intakeLower.getPIDController().setD(0);
    intakeLower.getPIDController().setFF(Constants.IntakeConstants.LOWER_FF / Constants.IntakeConstants.VELOCITY_CONVERSION_BOTTOM);
    // intakeLower.getPIDController().setSmartMotionMaxVelocity(5500, 0);
    intakeLower.setIdleMode(IdleMode.kBrake);
    intakeLower.burnFlash();

    intakeLeft.restoreFactoryDefaults();
    intakeLeft.follow(intakeUpper);
    intakeLeft.burnFlash();

    intakeRight.restoreFactoryDefaults();
    intakeRight.follow(intakeUpper, true);
    intakeRight.burnFlash();

    noteBeamBreak.setAverageBits(24);
    AnalogInput.setGlobalSampleRate(1500);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/UpperSpeed", intakeUpper.getEncoder().getVelocity());
    Logger.recordOutput("Intake/UpperCurrent", intakeUpper.getOutputCurrent());
    Logger.recordOutput("Intake/UpperFault", intakeUpper.getFaults());
    Logger.recordOutput("Intake/UpperStickyFault", intakeUpper.getStickyFaults());
    Logger.recordOutput("Intake/LowerSpeed", intakeLower.getEncoder().getVelocity());
    Logger.recordOutput("Intake/LowerCurrent", intakeLower.getOutputCurrent());
    Logger.recordOutput("Intake/LowerFault", intakeLower.getFaults());
    Logger.recordOutput("Intake/LowerStickyFault", intakeLower.getStickyFaults());
    Util.LogCANSparkMax("Intake/IntakeLeft", intakeLeft);
    Util.LogCANSparkMax("Intake/IntakeRight", intakeRight);
    Logger.recordOutput("Intake/IsNotePresent", IsNotePresent());
    Logger.recordOutput("Intake/NoteBeamBreakValue", noteBeamBreak.getAverageVoltage());
    // SmartDashboard.putNumber("IntakeSpeed2", SmartDashboard.getNumber("IntakeSpeed", 0));
    // This method will be called once per scheduler run
  }

  // private void SetIntakeSpeed (double speed){
  //   intakeUpper.set(speed);
  //   intakeLower.set(speed);
  // }

  public void SetIntakeSpeed(double metersPerSecond)
  {
    Logger.recordOutput("Intake/Setpoint", metersPerSecond);
    intakeUpper.getPIDController().setReference(metersPerSecond, CANSparkBase.ControlType.kVelocity);
    intakeLower.getPIDController().setReference(metersPerSecond, CANSparkBase.ControlType.kVelocity);
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
    Logger.recordOutput("Intake/Setpoint", 0);
    intakeUpper.getPIDController().setReference(0, CANSparkMax.ControlType.kVoltage);
    intakeLower.getPIDController().setReference(0, CANSparkMax.ControlType.kVoltage);
  }

  public Command StopIntakeCommand(){
    return runOnce(() -> {
      this.StopIntake();
    });
  }

  public boolean IsNotePresent()
  {
    return noteBeamBreak.getAverageVoltage() < Constants.IntakeConstants.NOTE_BEAM_BREAK_VOLTAGE_THRESHOLD;//beam break is active low
  }
}
