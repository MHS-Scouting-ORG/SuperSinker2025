package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
  public static class AlgaeIntakeConstants{
    //Motor ID's
    public static final int INTAKEID = 17;
    
    //Sensor ID's
    public static final int OPTICALID = 0;

    //Current Limiting
    public static final boolean CURRENTLIMIT = true;

    // Timeout for PID controller
    public static final double TIMEOUT = 0.125;

    //Intake Max Speeds
    public static final double INTAKEMAXSPEED = 1.0;
    public static final double OUTTAKEMAXSPEED = 1.0;

    public static final int PIVOTID = 18;

    //Sensor ID's  
    public static final int LSID = 3;

    //PID Constants and Other Important Variables
    //0.000000000001
    //0.0003
    public static final double KP = 0.00055;
    public static final double KI = 0.0007;//0.0005; //0.0004;
    public static final double KD = 0.000002; //0.00005;

    public static final double TOLERANCE = 45;

    //Pivot Max Speed
    public static final double MANUALPIVOTMAXSPEED = 1.0;
    public static final double PIVOTMAXSPEED = 0.5;
  }


  public static class ElevatorConstants{
    //Motor ID's
    public static final int LIFTID = 14;

    //Sensor ID's
    public static final int UPPERLSID = 3;
    public static final int BOTTOMLSID = 2;

    //PID Constants and Other Important Variables
    public static final double KP = 0.02;
    public static final double KI = 0.01;
    public static final double KD = 0.0;

    public static final double TOLERANCE = 5; 


    public static final double CURRENTLIMIT = 30;

    public static final double MAXSPEED = 1.0; //0.8
  }

  public final class CoralConstants {

    // Speeds
    public static final double CORAL_INTAKE_SPEED = 0.8;
    public static final double CORAL_DEPLOY_SPEED = 0.8;
    public static final double CORAL_OUTTAKE_SPEED = 0;
    public static final double CORAL_PIVOT_SPEED = 0.6;

    // IDs
    public static final int CORAL_INTAKE_ID = 15;
    public static final int CORAL_PIVOT_ID = 16;
    public static final int CORAL_OPTICAL_SENSOR_ID = 1;

  }

  public final class SwerveConstants {
    public final int[] BLUE_REEF_IDS = {17, 18, 19, 20, 21, 22}; 
    public final int[] RED_REEF_IDS = {6, 7, 8, 9, 10, 11}; 

    public static final double turningKp = 0.01; 
    public static final double turningKi = 0.0;
    public static final double turningKd = 0.0; 
  }
}
