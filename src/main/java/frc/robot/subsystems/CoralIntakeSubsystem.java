// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.CoralConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  private final DigitalInput opticalSensor;
  private final TalonSRX coralIntake;

  public CoralIntakeSubsystem() {

    opticalSensor = new DigitalInput(CoralConstants.CORAL_OPTICAL_SENSOR_ID);
    coralIntake = new TalonSRX(CoralConstants.CORAL_INTAKE_ID);

    coralIntake.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, CoralConstants.CORAL_INTAKE_ID);

    coralIntake.configFactoryDefault();
    
    coralIntake.setInverted(true);
    coralIntake.setNeutralMode(NeutralMode.Brake);
  }
  
  // return current value of Optical Switch
  public boolean getOpticalSensor() {
    return opticalSensor.get();
  }

  // set Coral Intake sped to speed
  public void setIntakeSpeed(double speed) {
    coralIntake.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Optical Sensor", getOpticalSensor());
  }
}
