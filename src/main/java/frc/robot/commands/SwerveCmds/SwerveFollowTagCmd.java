package frc.robot.commands.SwerveCmds;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TunerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.SwerveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveFollowTagCmd extends Command {

  private CommandSwerveDrivetrain swerveSubs; 
  private DoubleSupplier xSupp, ySupp, zSupp;
  private double xSpeed, ySpeed, zSpeed; 
  private PhotonCamera rightArducam; 
  private List<PhotonPipelineResult> unreadResultList; 
  // private PhotonPipelineResult result; 
  private PIDController turningPid; 

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  /** Creates a new SwerveFollowTagCmd. */
  public SwerveFollowTagCmd(CommandSwerveDrivetrain swerveSubs, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier zSupp) {
    this.swerveSubs = swerveSubs; 
    this.xSupp = xSupp; 
    this.ySupp = ySupp; 
    this.zSupp = zSupp; 

    rightArducam = new PhotonCamera("Arducam_OV9281_USB_Camera");
    turningPid = new PIDController(SwerveConstants.turningKp, SwerveConstants.turningKi, SwerveConstants.turningKd); 
    turningPid.setTolerance(2);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    unreadResultList = rightArducam.getAllUnreadResults(); 

    if (!unreadResultList.isEmpty()) {
      PhotonPipelineResult result = unreadResultList.get(unreadResultList.size() - 1); 

      if (result.hasTargets()) {
        int targetID = result.getBestTarget().getFiducialId(); 
        SmartDashboard.putNumber("april tag id", targetID); 
        // close A 
        if (targetID == 19 || targetID == 6) {
          zSpeed = turningPid.calculate(Units.radiansToDegrees(swerveSubs.getRotation3d().getAngle()), 300); 
        } 
        // close B 
        else if (targetID == 18 || targetID == 7) {
          zSpeed = turningPid.calculate(Units.radiansToDegrees(swerveSubs.getRotation3d().getAngle()), 0);
        }
        //close C 
        else if (targetID == 17 || targetID == 8) {
          zSpeed = turningPid.calculate(Units.radiansToDegrees(swerveSubs.getRotation3d().getAngle()), 60);
        }
        // far A 
        else if (targetID == 20 || targetID == 11) {
          zSpeed = turningPid.calculate(Units.radiansToDegrees(swerveSubs.getRotation3d().getAngle()), 360-120);
        }
        // far B 
        else if (targetID == 21 || targetID == 10) {
          zSpeed = turningPid.calculate(Units.radiansToDegrees(swerveSubs.getRotation3d().getAngle()), 180);
        }
        // far C 
        else if (targetID == 22 || targetID == 9) {
          zSpeed = turningPid.calculate(Units.radiansToDegrees(swerveSubs.getRotation3d().getAngle()), 120);
        } else {}
      } else {
        zSpeed = zSupp.getAsDouble(); 
      }
    }

    // DRIVE 
    xSpeed = xSupp.getAsDouble(); 
    ySpeed = ySupp.getAsDouble(); 
    swerveSubs.applyRequest(() -> 
              driveRobotCentric.withVelocityX(xSpeed * MaxSpeed * 0.5)
              .withVelocityY(ySpeed * MaxSpeed * 0.5)
              .withRotationalRate(zSpeed * MaxAngularRate)); 

    SmartDashboard.putNumber("getAngle", Units.radiansToDegrees(swerveSubs.getRotation3d().getAngle())); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
