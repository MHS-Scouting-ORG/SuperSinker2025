package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.CoralConstants;

public class CoralPivotSubsystem extends SubsystemBase {

  private final TalonSRX coralIntake, coralPivot;
  private PIDController pivotPIDController;
  private double error, prevError;
  private boolean pidStatus, atSetpointVal;
  private Timer coralTimer;
  private final double coralTimeout;

  public CoralPivotSubsystem() {

    coralIntake = new TalonSRX(CoralConstants.CORAL_INTAKE_ID);
    coralPivot = new TalonSRX(CoralConstants.CORAL_PIVOT_ID);
    coralIntake.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen,
        CoralConstants.CORAL_INTAKE_ID);
    pivotPIDController = new PIDController(0.0005, 0.0003, 0);
    pivotPIDController.setTolerance(20);
    pidStatus = false;
    atSetpointVal = false;
    coralTimer = new Timer();
    coralTimeout = 0.125;

    coralIntake.configFactoryDefault();
    coralPivot.configFactoryDefault();

    coralIntake.setInverted(true);
    coralIntake.setNeutralMode(NeutralMode.Brake);
    coralPivot.setNeutralMode(NeutralMode.Brake);
  }

  public void resetPivotEnc() {
    coralPivot.getSensorCollection().setQuadraturePosition(0, 0);
  }

  // set Coral Pivot speed to speed
  public void setPivotSpeed(double speed) {
    coralPivot.set(TalonSRXControlMode.PercentOutput, speed);
    /*
     * (coralPivot.getSensorCollection().getQuadraturePosition()<= -600 && speed <
     * 0){
     * coralPivot.set(TalonSRXControlMode.PercentOutput, 0);
     * } else if (coralPivot.getSensorCollection().getQuadraturePosition() >=-50 &&
     * speed > 0){
     * coralPivot.set(TalonSRXControlMode.PercentOutput, 0);
     * }
     */
  }

  // set Coral PIDstatus to stat
  public void setPIDStatus(boolean stat) {
    pidStatus = stat;
  }

  // set Coral PID setpoint to setpoint
  public void setCoralPivotPIDSetpoint(double setpoint) {
    pivotPIDController.setSetpoint(setpoint);
  }

  public void pivotMiddleToLeft(){
    int count = 0;
    if(getCoralSwitchEnc() < - 300){
      if(count == 0){
        setCoralPivotPIDSetpoint(-380);
        if(atSetpoint()){
          count++;
        }
      }
      if(count == 1){
        setCoralPivotPIDSetpoint(-75);
        if(atSetpoint()){
          count++;
        }
      }
    }
  }
  
  public void pivotMiddleToRight(){
    int count = 0;
    if(getCoralSwitchEnc() > -400){
      if(count == 0){
        setCoralPivotPIDSetpoint(-380);
        if(atSetpoint()){
          count++;
        }
      }
      if(count == 1){
        setCoralPivotPIDSetpoint(-673);
        if(atSetpoint()){
          count++;
        }
      }
    }
  }

  // return Coral Encoder
  public double getCoralSwitchEnc() {
    return coralPivot.getSensorCollection().getQuadraturePosition();
  }

  public double getSetpoint() {
    return pivotPIDController.getSetpoint();
  }

  // return current value of PID status
  public boolean getPIDStatus() {
    return pidStatus;
  }

  // return current value of Limit Switch
  public boolean getLimitSwitch() {
    return coralIntake.isFwdLimitSwitchClosed() == 1;
  }

  // return true if at setpoint
  public boolean atSetpoint() {
    // if(!pivotPIDController.atSetpoint()){
    // coralTimer.stop();
    // coralTimer.reset();
    // return false;
    // }
    // if(!coralTimer.isRunning()){
    // coralTimer.start();
    // }
    // if(coralTimer.get() >= coralTimeout){
    // return true;
    // }
    // return false;
    return atSetpointVal;
  }

  @Override
  public void periodic() {

    if (getLimitSwitch()) {
      resetPivotEnc();
    }

    if (getPIDStatus()) {
      error = pivotPIDController.calculate(getCoralSwitchEnc(), getSetpoint());
      if (error > CoralConstants.CORAL_PIVOT_SPEED && !atSetpoint()) {
        error = CoralConstants.CORAL_PIVOT_SPEED;
      } else if (error < -CoralConstants.CORAL_PIVOT_SPEED && !atSetpoint()) {
        error = -CoralConstants.CORAL_PIVOT_SPEED;
      }

      if (error < 0 && prevError > 0) {
        pivotPIDController.reset();
      } else if (error > 0 && prevError < 0) {
        pivotPIDController.reset();
      }

      prevError = error;
    }

    if (!pivotPIDController.atSetpoint()) {
      coralTimer.stop();
      coralTimer.reset();
      atSetpointVal = false;
    } else {
      if (!coralTimer.isRunning()) {
        coralTimer.start();
      }
      if (coralTimer.get() >= coralTimeout) {
        atSetpointVal = true;
      }else{
        atSetpointVal = false;
      }
    }

    if (atSetpoint()) {
      error = 0;
    }

    setPivotSpeed(error);

    SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    SmartDashboard.putNumber("Intake Pivot Enc", getCoralSwitchEnc());
    SmartDashboard.putNumber("Pivot PID Error", error);
    SmartDashboard.putNumber("Setpoint", getSetpoint());
    SmartDashboard.putBoolean("PID Status", getPIDStatus());
    SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    SmartDashboard.putBoolean("Is Done", atSetpointVal);
    SmartDashboard.putNumber("Coral Timer", coralTimer.get());

    // coralIntake.set(TalonSRXControlMode.PercentOutput, intakeSpeed);
    // coralPivot.set(TalonSRXControlMode.PercentOutput, pivotSpeed);
  }
}