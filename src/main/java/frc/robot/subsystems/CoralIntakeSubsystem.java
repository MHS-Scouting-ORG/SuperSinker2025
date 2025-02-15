package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.CoralConstants;

public class CoralIntakeSubsystem extends SubsystemBase {

  private final TalonSRX coralIntake, coralPivot;
  private final DigitalInput opticalSensor, limitSwitch;
  private PIDController pivotPIDController;
  private int setpoint;
  private boolean pidStatus = false;

  public CoralIntakeSubsystem() {

    coralIntake = new TalonSRX(CoralConstants.CORAL_INTAKE_ID);
    coralPivot = new TalonSRX(CoralConstants.CORAL_PIVOT_ID);
    opticalSensor = new DigitalInput(CoralConstants.CORAL_OPTICAL_SENSOR_ID);
    limitSwitch = new DigitalInput(CoralConstants.CORAL_LIMIT_SWITCH_ID);
    coralIntake.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, CoralConstants.CORAL_INTAKE_ID);
    pivotPIDController = new PIDController(0.0007,0, 0);
    pivotPIDController.setTolerance(25);

    coralIntake.configFactoryDefault();
    coralPivot.configFactoryDefault();

    coralIntake.setInverted(true);
    coralIntake.setNeutralMode(NeutralMode.Brake);
    coralPivot.setNeutralMode(NeutralMode.Brake);
  }

  public void resetPivotEnc(){
    coralPivot.getSensorCollection().setQuadraturePosition(0, 0);
  }

  // set Coral Pivot speed to speed
  public void setPivotSpeed(double speed) {
    coralPivot.set(TalonSRXControlMode.PercentOutput, speed);
    if(coralPivot.getSensorCollection().getQuadraturePosition()<= -600 && speed < 0){
      coralPivot.set(TalonSRXControlMode.PercentOutput, 0);
    } else if (coralPivot.getSensorCollection().getQuadraturePosition() >=-50 && speed > 0){
      coralPivot.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  // set Coral Intake sped to speed
  public void setIntakeSpeed(double speed) {
    coralIntake.set(TalonSRXControlMode.PercentOutput, speed);
  }

  // set Coral PIDstatus to stat
  public void setPIDStatus(boolean stat){
    pidStatus = stat;
  }

  // set Coral PID setpoint to setpoint
  public void setCoralPivotPIDSetpoint(int setpoint){
    this.setpoint = setpoint;
  }

  // return Coral Encoder
  public double getCoralSwitchEnc() {
    return coralPivot.getSensorCollection().getQuadraturePosition();
  }

  // return current value of PID status
  public boolean getPIDStatus(){
    return pidStatus;
  }

  // return current value of Limit Switch
  public boolean getLimitSwitch() {
    return coralIntake.isFwdLimitSwitchClosed() == 1 ? true : false;
  }

  // return current value of Optical Switch
  public boolean getOpticalSensor() {
    return opticalSensor.get();
  }

  // return true if at setpoint
  public boolean atSetpoint(){
    return (coralPivot.getSensorCollection().getQuadraturePosition() >= setpoint - 20) && 
    (coralPivot.getSensorCollection().getQuadraturePosition() <= setpoint + 20);
  }

  @Override
  public void periodic() {

    if(getLimitSwitch()){
      resetPivotEnc();
    }

    if(pidStatus){
      double error = pivotPIDController.calculate(getCoralSwitchEnc(), setpoint);
      if(error > CoralConstants.CORAL_PIVOT_SPEED && !atSetpoint()){
        error = CoralConstants.CORAL_PIVOT_SPEED;
      }else if(error < -CoralConstants.CORAL_PIVOT_SPEED && !atSetpoint()){
        error = -CoralConstants.CORAL_PIVOT_SPEED;
      }

      setPivotSpeed(error);
    }

    SmartDashboard.putBoolean("Optical Sensor", getOpticalSensor());
    SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    SmartDashboard.putNumber("Intake Pivot Enc", getCoralSwitchEnc());
    SmartDashboard.putNumber("Pivot PID Error", pivotPIDController.calculate(getCoralSwitchEnc(), setpoint));
    SmartDashboard.putNumber("Setpoint", setpoint);
    SmartDashboard.putBoolean("PID Status", pidStatus);

  //coralIntake.set(TalonSRXControlMode.PercentOutput, intakeSpeed);
  // coralPivot.set(TalonSRXControlMode.PercentOutput, pivotSpeed);
  }
}