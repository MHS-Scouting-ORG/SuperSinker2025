package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
  private final TalonSRX coralIntake, coralPivot;
  private final DigitalInput opticalSensor, leftLimitSwitch, rightLimitSwitch;
  private boolean pidStatus;
  private double pivotSpeed;
  private double intakeSpeed;


  public CoralIntakeSubsystem() {
    pidStatus = false;

    coralIntake = new TalonSRX(CoralConstants.CORAL_INTAKE_ID);
    coralPivot = new TalonSRX(CoralConstants.CORAL_PIVOT_ID);
    opticalSensor = new DigitalInput(CoralConstants.CORAL_OPTICAL_SENSOR_ID);
    leftLimitSwitch = new DigitalInput(CoralConstants.CORAL_LIMIT_SWITCH_ID);
    rightLimitSwitch = new DigitalInput(6);

    coralIntake.setNeutralMode(NeutralMode.Brake);
    coralPivot.setNeutralMode(NeutralMode.Brake);

    coralIntake.neutralOutput();
    coralPivot.neutralOutput();

    coralPivot.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    coralPivot.config_kP(0, 0.01);
    coralPivot.config_kI(0, 0.001);
  }

  public double getCoralSwitchEnc() {
    return coralPivot.getSelectedSensorPosition();
  }

  public boolean getOpticalSensor() {
    return opticalSensor.get();
  }

  public void setPivotSpeed(double speed) {
    pivotSpeed = speed;
  }

  public void setIntakeSpeed(double speed) {
    intakeSpeed = speed;
  }

  public void setCoralPivotPIDSetpoint(double setpoint){
    coralPivot.setSelectedSensorPosition(setpoint);
  }

  public boolean getLeftLimitSwitch() {
    return leftLimitSwitch.get();
  }

  public boolean getRightLimitSwitch() {
    return rightLimitSwitch.get();
  }

  @Override
  public void periodic() {

    if (pivotSpeed < 0 && getLeftLimitSwitch()) {
      pivotSpeed = 0;
    } else if (pivotSpeed > 0 && getRightLimitSwitch()) {
      pivotSpeed = 0;
    }

     coralIntake.set(TalonSRXControlMode.PercentOutput, intakeSpeed);
   // coralPivot.set(TalonSRXControlMode.PercentOutput, pivotSpeed);
    

    SmartDashboard.putBoolean("opticalSensor", getOpticalSensor());
    SmartDashboard.putBoolean("Limit Switch", getLeftLimitSwitch());
    SmartDashboard.putNumber("Intake Pivot Enc", getCoralSwitchEnc());
  }
}