package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveModuleConstants{
  public final int driveMotorCANID;
  public final int turningMotorCANID;
  public final int canCoderID;
  public final InvertedValue driveMotorInverted;
  public final InvertedValue turningMotorInverted;
  public final double encOffset;

  public SwerveModuleConstants(int driveMotorCANID, int turningMotorCANID, int canCoderID, InvertedValue driveMotorInverted, InvertedValue turningMotorInverted, double encOffset){
    this.driveMotorCANID = driveMotorCANID;
    this.turningMotorCANID = turningMotorCANID;
    this.canCoderID = canCoderID;
    this.driveMotorInverted = driveMotorInverted;
    this.turningMotorInverted = turningMotorInverted;
    this.encOffset = encOffset;
  }

}