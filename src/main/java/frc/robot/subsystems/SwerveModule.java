package frc.robot.subsystems;

import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;


public class SwerveModule {
    public int moduleID; 

    private TalonFX driveMotor, turningMotor; 
    private CANcoder absoluteEncoder; 

    private PIDController turningPidController; 
    private double encOffset; 
    
    public SwerveModule (int moduleID, SwerveModuleConstants moduleConstants) {
        this.moduleID = moduleID; 

        this.encOffset = moduleConstants.encOffset;

        driveMotor = new TalonFX(moduleConstants.driveMotorCANID); 
        turningMotor = new TalonFX(moduleConstants.turningMotorCANID); 
        absoluteEncoder = new CANcoder(moduleConstants.canCoderID); 
        
        
        //CONFIGS 
        driveMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(moduleConstants.driveMotorInverted));
        driveMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)); 
        turningMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(moduleConstants.turningMotorInverted));
        turningMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5));
        absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        

        turningPidController = new PIDController(SwerveConstants.turningKP, SwerveConstants.turningKI, SwerveConstants.turningKD); 
        turningPidController.enableContinuousInput(-180, 180);
    }

    public double getDriveVelocity(){
        return driveMotor.getVelocity().getValueAsDouble();
    }
    
    public double getDrivePosition(){
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getAbsoluteDegrees(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(turningMotor.getPosition().getValueAsDouble());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteDegrees()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAbsoluteDegrees()));
    }

    public void setState(SwerveModuleState desiredState){
        desiredState.optimize(getAngle());

        double turningOutput = turningPidController.calculate(getState().angle.getDegrees(), desiredState.angle.getDegrees());

        turningMotor.set(turningOutput);
        driveMotor.set(desiredState.speedMetersPerSecond / Constants.SwerveConstants.kMaxSpeed * Constants.SwerveConstants.kVoltage);
    }

    public void setAngle(SwerveModuleState desiredState){
        desiredState.optimize(getAngle());

        double turningOutput = turningPidController.calculate(getState().angle.getDegrees(), desiredState.angle.getDegrees());

        turningMotor.set(turningOutput);
        driveMotor.set(0);
    }

    public void stop(){
        driveMotor.stopMotor();
        turningMotor.stopMotor();
    }

    public void print(){
    SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] Absolute Encoder Degrees", getAbsoluteDegrees());
    SmartDashboard.putNumber("S[" + moduleID + "] Drive Encoder", getDrivePosition());
    SmartDashboard.putNumber("S[" + moduleID + "] Turning Encoder", getAngle().getDegrees());
  }



}