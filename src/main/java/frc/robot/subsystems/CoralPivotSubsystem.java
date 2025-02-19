package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;

public class CoralPivotSubsystem extends SubsystemBase {

  private final TalonSRX coralIntake, coralPivot;
  private PIDController pivotPIDController;
  private int setpoint;
  private boolean pidStatus = false;

  public CoralPivotSubsystem() {

    coralIntake = new TalonSRX(CoralConstants.CORAL_INTAKE_ID);
    coralPivot = new TalonSRX(CoralConstants.CORAL_PIVOT_ID);
    coralIntake.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, CoralConstants.CORAL_INTAKE_ID);
    pivotPIDController = new PIDController(0.0005,0.0002, 0); //0.0007 p
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
    return coralIntake.isFwdLimitSwitchClosed() == 1;
  }


  // return true if at setpoint
  public boolean atSetpoint(){
    return pivotPIDController.atSetpoint();
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

    SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());
    SmartDashboard.putNumber("Intake Pivot Enc", getCoralSwitchEnc());
    SmartDashboard.putNumber("Pivot PID Error", pivotPIDController.calculate(getCoralSwitchEnc(), setpoint));
    SmartDashboard.putNumber("Setpoint", setpoint);
    SmartDashboard.putBoolean("PID Status", pidStatus);

  //coralIntake.set(TalonSRXControlMode.PercentOutput, intakeSpeed);
  // coralPivot.set(TalonSRXControlMode.PercentOutput, pivotSpeed);
  }
}