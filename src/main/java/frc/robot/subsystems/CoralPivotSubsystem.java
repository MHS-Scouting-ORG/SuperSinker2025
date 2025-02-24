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
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;

public class CoralPivotSubsystem extends SubsystemBase {

  private final TalonSRX coralIntake, coralPivot;
  private PIDController pivotPIDController;
  private double command, prevError;
  private boolean pidStatus;
  private Timer coralTimer, pivotTimer;
  private final double coralTimeout;

  public CoralPivotSubsystem() {

    coralIntake = new TalonSRX(CoralConstants.CORAL_INTAKE_ID);
    coralPivot = new TalonSRX(CoralConstants.CORAL_PIVOT_ID);
    coralIntake.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen,
        CoralConstants.CORAL_INTAKE_ID);
    pivotPIDController = new PIDController(0.0012, 0.001, 0);
    pivotPIDController.setTolerance(15);
    pidStatus = false;
    coralTimer = new Timer();
    pivotTimer = new Timer();
    coralTimeout = 0.25;

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
    pivotPIDController.reset();
    pivotPIDController.setSetpoint(setpoint);
  }

  public void pivotMiddleToLeft(){
    int count = 0;
    pivotTimer.reset();
    if(getCoralSwitchEnc() < - 300){
      pivotTimer.start();
      if(count == 0 && pivotTimer.get() < 0.15){
        setCoralPivotPIDSetpoint(-380);
        if(atCPivotSetpoint() && pivotTimer.get() >= 0.15){
          count++;
          pivotTimer.stop();
        }
      }
      if(count == 1){
        setCoralPivotPIDSetpoint(-75);
        if(atCPivotSetpoint() && count < 2){
          count++;
        }
      }
    }
  }

  public void setCommand(double value){
    command = value;
  }
  
  public void pivotMiddleToRight(){
    int count = 0;
    if(getCoralSwitchEnc() > -400){
      if(count == 0){
        setCoralPivotPIDSetpoint(-380);
        if(atCPivotSetpoint()){
          count++;
        }
      }
      if(count == 1){
        setCoralPivotPIDSetpoint(-673);
        if(atCPivotSetpoint()){
          count++;
        }
      }
    }
  }

  // return Coral Encoder
  public double getCoralSwitchEnc() {
    return coralPivot.getSensorCollection().getQuadraturePosition();
  }

  public double getCPivotPIDSetpoint() {
    return pivotPIDController.getSetpoint();
  }

  // return current value of PID status
  public boolean getPIDStatus() {
    return pidStatus;
  }

  // return current value of Limit Switch
  public boolean getCLimitSwitch() {
    return coralIntake.isFwdLimitSwitchClosed() == 1;
  }

  // return true if at setpoint
  public boolean atCPivotSetpoint() {
    if(!pivotPIDController.atSetpoint()){
    coralTimer.stop();
    coralTimer.reset();
    return false;
    }
    if(!coralTimer.isRunning()){
      coralTimer.start();
    }
    if(coralTimer.get() >= coralTimeout){
      return true;
    }else{
      return false;
    }
  }

  @Override
  public void periodic() {
    double currError = getCPivotPIDSetpoint() - getCoralSwitchEnc();

    if (getPIDStatus()) {
      command = pivotPIDController.calculate(getCoralSwitchEnc(), getCPivotPIDSetpoint());
      if (command > CoralConstants.CORAL_PIVOT_SPEED) {
        command = CoralConstants.CORAL_PIVOT_SPEED;
      } else if (command < -CoralConstants.CORAL_PIVOT_SPEED) {
        command = -CoralConstants.CORAL_PIVOT_SPEED;
      }

      if (currError < 0 && prevError > 0) {
        pivotPIDController.reset();
      } else if (currError > 0 && prevError < 0) {
        pivotPIDController.reset();
      }

      prevError = currError;
    } else {
      setCoralPivotPIDSetpoint(getCoralSwitchEnc());
    }

    // if (atSetpoint()) {
    //   command = 0;
    // }
    
    if(getCLimitSwitch()){
      resetPivotEnc();
      if(command > 0){
        command = 0;
        // setCoralPivotPIDSetpoint(0);
      }
    }
    setPivotSpeed(command);

    SmartDashboard.putBoolean("Limit Switch", getCLimitSwitch());
    SmartDashboard.putNumber("Intake Pivot Enc", getCoralSwitchEnc());
    SmartDashboard.putNumber("Pivot PID Output", command);
    SmartDashboard.putNumber("Setpoint", getCPivotPIDSetpoint());
    SmartDashboard.putBoolean("PID Status", getPIDStatus());
    SmartDashboard.putBoolean("At Setpoint", atCPivotSetpoint());
    SmartDashboard.putBoolean("PID At setpoint", pivotPIDController.atSetpoint());
    SmartDashboard.putNumber("Coral Timer", coralTimer.get());
    SmartDashboard.putNumber("PID Error", currError);

    // coralIntake.set(TalonSRXControlMode.PercentOutput, intakeSpeed);
    // coralPivot.set(TalonSRXControlMode.PercentOutput, pivotSpeed);
  }
}