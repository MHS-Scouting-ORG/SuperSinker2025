package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.SwerveModuleConstants;

public class Constants {
  public static class SwerveConstants {  

        public static final double kWheelDiameter = 4 * 2.5 / 100;
        public static final double kTrackWidth = 0.635;
        public static final double kWheelBase = 0.635;

        public static final double kGearRatio = 8.14 / 1;

        public static final double kVoltage = 7.2;

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
           //FL
           new Translation2d(kWheelBase / 2, kTrackWidth / 2),
           //BL
           new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
           //FR
           new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
           //BR
           new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

    public static class FrontLeft{
        public static final int kDriveID = 1;
        public static final int kTurningID = 5;
        public static final int kCANID = 9;
        public static final InvertedValue kDriveInverted = InvertedValue.Clockwise_Positive; 
        public static final InvertedValue kTurningInverted = InvertedValue.CounterClockwise_Positive; 
        public static final double kEncOffset = 0;

        public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveID, kTurningID, kCANID, kDriveInverted, kTurningInverted, kEncOffset);
    }

    public static class BackLeft{
      public static final int kDriveID = 2;
      public static final int kTurningID = 6;
      public static final int kCANID = 10;
      public static final InvertedValue kDriveInverted = InvertedValue.Clockwise_Positive; 
      public static final InvertedValue kTurningInverted = InvertedValue.CounterClockwise_Positive;
      public static final double kEncOffset = 0;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveID, kTurningID, kCANID, kDriveInverted, kTurningInverted, kEncOffset);
    }

    public static class FrontRight{
      public static final int kDriveID = 4;
      public static final int kTurningID = 8;
      public static final int kCANID = 12;
      public static final InvertedValue kDriveInverted = InvertedValue.Clockwise_Positive; 
      public static final InvertedValue kTurningInverted = InvertedValue.CounterClockwise_Positive; 
      public static final double kEncOffset = 0;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveID, kTurningID, kCANID, kDriveInverted, kTurningInverted, kEncOffset);
    }

    public static class BackRight{
      public static final int kDriveID = 3;
      public static final int kTurningID = 7;
      public static final int kCANID = 11;
      public static final InvertedValue kDriveInverted = InvertedValue.Clockwise_Positive; 
      public static final InvertedValue kTurningInverted = InvertedValue.CounterClockwise_Positive; 
      public static final double kEncOffset = 0;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveID, kTurningID, kCANID, kDriveInverted, kTurningInverted, kEncOffset);
    }

    public static final double kPositionConversionFactor = kGearRatio * Math.PI * kWheelDiameter;
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static final double turningKP = 0.0;
    public static final double turningKI = 0.0;
    public static final double turningKD = 0.0;


    public static final double drivingKP = 0.0;
    public static final double drivingKI = 0.0;
    public static final double drivingKD = 0.0;

    public static final double kMaxSpeed = 3.6576;
    }
      
  public static class AlgaeIntakeConstants{
    //LS = Limit Switch

    //Motor ID's
    public static final int INTAKEID = 9;
    public static final int PIVOTID = 18;

    //Sensor ID's
    public static final int OPTICALID = 1;
    public static final int LSID = 0;

    //PID Constants and Other Important Variables
    public static final double KP = 0.01;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double TOLERANCE = 15.0;

    public static final double INTAKEMAXSPEED = 0.4;
    public static final double PIVOTMAXSPEED = 0.2;
  }

  public static class ElevatorConstants{
    //Motor ID's
    public static final int LIFTID = 8;

    //Sensor ID's
    public static final int UPPERLSID = 3;
    public static final int BOTTOMLSID = 5;

    //PID Constants and Other Important Variables
    public static final double KP = 0.01;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double TOLERANCE = 5; 

    public static final double MAXSPEED = 0.2;
  }
}
  
