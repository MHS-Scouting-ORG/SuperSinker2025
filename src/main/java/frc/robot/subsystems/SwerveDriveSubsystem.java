package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDriveSubsystem extends SubsystemBase{
    private SwerveDriveOdometry odometry;
  //private AHRS navx;
  private Pigeon2 pigeon2;
  private SwerveModule[] swerveModules;
  private RobotConfig config;

  public SwerveDriveSubsystem() {
    swerveModules = new SwerveModule[]{
      new SwerveModule(0, SwerveConstants.FrontLeft.constants),
      new SwerveModule(1, SwerveConstants.BackLeft.constants),
      new SwerveModule(2, SwerveConstants.FrontRight.constants),
      new SwerveModule(3, SwerveConstants.BackRight.constants)
    };


    pigeon2 = new Pigeon2(13);
    pigeon2.setYaw(0);

    odometry = new SwerveDriveOdometry(
      SwerveConstants.kKinematics, 
      pigeon2.getRotation2d(),  
      getCurrentSwerveModulePositions());

    
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose,
      this::resetOdometry,
      this::getChassisSpeeds, 
      this::driveRobotRelative, 
      new PPHolonomicDriveController(
        new PIDConstants(Constants.SwerveConstants.drivingKP, Constants.SwerveConstants.drivingKI, Constants.SwerveConstants.drivingKD), 
        new PIDConstants(Constants.SwerveConstants.turningKP, Constants.SwerveConstants.turningKI, Constants.SwerveConstants.turningKD)
      ),
       config, 
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;     
        },
      this);
  }


  public void resetPigeon2(){
    pigeon2.setYaw(0);
  }

  public Rotation2d getRotation2d(){
    return pigeon2.getRotation2d();
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public ChassisSpeeds getChassisSpeeds(){
    return Constants.SwerveConstants.kKinematics.toChassisSpeeds(getCurrentSwerveModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.SwerveConstants.kKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public void setPose(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getCurrentSwerveModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getCurrentSwerveModulePositions(), pose);
  }

  public SwerveModulePosition[] getCurrentSwerveModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule swerveModule : swerveModules){
      positions[swerveModule.moduleID] = swerveModule.getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getCurrentSwerveModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule swerveModule : swerveModules){
      states[swerveModule.moduleID] = swerveModule.getState();
    }
    return states;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeed);

    for(SwerveModule swerveModule : swerveModules){
      swerveModule.setState(desiredStates[swerveModule.moduleID]);
    }
  }

  public void stopModules(){
    for(SwerveModule swerveModule : swerveModules){
      swerveModule.stop();
    }
  }

 
  public void periodic() {
    odometry.update(getRotation2d().unaryMinus(), getCurrentSwerveModulePositions());
    for(SwerveModule swerveModule : swerveModules){
      swerveModule.print();
    }
    SmartDashboard.putNumber("Pigeon 2.0 Yaw", pigeon2.getRotation2d().getDegrees());
  }
}
