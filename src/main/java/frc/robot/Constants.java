package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public class Constants {
    public static class AlgaeIntakeConstants{
      //Motor ID's
      public static final int INTAKEID = 15;
      
      //Sensor ID's
      public static final int OPTICALID = 1;
  
      //Current Limiting
      public static final boolean CURRENTLIMIT = true;
  
      //Intake Max Speeds
      public static final double INTAKEMAXSPEED = 0.6;
      public static final double OUTTAKEMAXSPEED = 1.0;
    }

    public static class AlgaePivotConstants{
      //Motor ID's
      public static final int PIVOTID = 14;
  
      //Sensor ID's
      public static final int OPTICALID = 8;
      public static final int LSID = 9;
      
      //Current Limiting
      public static final boolean CURRENTLIMIT = true;
  
      //PID Constants and Other Important Variables
      public static final double KP = 0.0005;
      public static final double KI = 0.;
      public static final double KD = 0.;
  
      public static final double TOLERANCE = 100.0;
  
      //Pivot Max Speed
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

  public final class CoralConstants {

    // Speeds
    public static final double CORAL_INTAKE_SPEED = 0.8;
    public static final double CORAL_OUTTAKE_SPEED = 0.8;
    public static final double CORAL_PIVOT_SPEED = 0.8;

    // IDs
    public static final int CORAL_INTAKE_ID = 12;
    public static final int CORAL_PIVOT_ID = 0;
    public static final int CORAL_LIMIT_SWITCH_ID = 0;
    public static final int CORAL_OPTICAL_SENSOR_ID = 1;

    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
   
  }

}
}
  
