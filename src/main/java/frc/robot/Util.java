package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

public class Util {
    


    public static void LogCANSparkMax(String logName, CANSparkMax sparkMax)
    {
        Logger.recordOutput(logName + "AppliedOutput", sparkMax.getAppliedOutput());
        Logger.recordOutput(logName + "Velocity", sparkMax.getEncoder().getVelocity());
        Logger.recordOutput(logName + "Position", sparkMax.getEncoder().getPosition());
        Logger.recordOutput(logName + "Current", sparkMax.getOutputCurrent());
        Logger.recordOutput(logName + "Faults", sparkMax.getFaults());
        Logger.recordOutput(logName + "StickyFaults", sparkMax.getStickyFaults());
        Logger.recordOutput(logName + "Temp", sparkMax.getMotorTemperature());
    }
}
