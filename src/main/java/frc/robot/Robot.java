// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;



import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.net.PortForwarder;



/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  
  // Constants such as camera and target height stored. Change per robot and goa!

  // Angle between horizontal and the camera.
  // How far from the target we want to be
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(9);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(3);
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  final double GOAL_RANGE_METERS = Units.feetToMeters(2);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("Metrobots3324");
  PhotonCamera camera2 = new PhotonCamera("Metrobots3324_2");

  // PID constants should be tuned per robot
  final double P_GAIN = 0.1;
  final double D_GAIN = 0.0;
  PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);


  // Drive motors
  private final WPI_TalonSRX m_leftMotor1 = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_rightMotor1 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(4);
  private final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(5);

  private final DifferentialDrive robotDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
  
  public final static XboxController primaryDriver = new XboxController(0);
  public final static XboxController secondaryDriver = new XboxController(1);

  @Override
  public void robotInit() {

    PortForwarder.add(5800, "photonvision.local", 5800);

    m_rightMotor1.setInverted(false);
    m_leftMotor1.setInverted(false);

    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);

  }

  @Override
  public void teleopPeriodic() {

      //Variable declarations
      var result = camera.getLatestResult();
      PhotonTrackedTarget target = result.getBestTarget();
      camera.setDriverMode(false);
      double latencySeconds = result.getLatencyMillis() / 1000.0;


      double forwardSpeed = 0;
      double rotationSpeed = 0;


      if (primaryDriver.getXButton()) {

        camera.setPipelineIndex(1);
          // Vision-alignment mode
          // Query the latest result from PhotonVision

          if (result.hasTargets()) {
              // First calculate range
              double range =
                      PhotonUtils.calculateDistanceToTargetMeters(
                              CAMERA_HEIGHT_METERS,
                              TARGET_HEIGHT_METERS,
                              CAMERA_PITCH_RADIANS,
                              Units.degreesToRadians(result.getBestTarget().getPitch()));

              // Use this range as the measurement we give to the PID controller.
              // -1.0 required to ensure positive PID controller effort _increases_ range
              forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);
              rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

          } else {
              // If we have no targets, stay still.
              forwardSpeed = 0;
              rotationSpeed = 0;
          }
      } else {
          // Manual Driver Mode
            double throttle = primaryDriver.getRightTriggerAxis() - primaryDriver.getLeftTriggerAxis();

            robotDrive.arcadeDrive(primaryDriver.getLeftX(), -throttle);

      }

      // Use our forward/turn speeds to control the drivetrain
      robotDrive.arcadeDrive(forwardSpeed, rotationSpeed);



      if (result.hasTargets()) {
            double poseAmbiguity = target.getPoseAmbiguity();
            int targetID = target.getFiducialId();

            SmartDashboard.putNumber("TargetID", targetID);
            SmartDashboard.putNumber("Ambiguity", poseAmbiguity);

            
      } else {

        SmartDashboard.putNumber("TargetID", 0);
        SmartDashboard.putNumber("Ambiguity", 0);

      }

      SmartDashboard.putNumber("Latency", latencySeconds);

      if (primaryDriver.getYButton()) {
        
        camera.setDriverMode(false);
        camera.setPipelineIndex(2);

      }

      if (primaryDriver.getBButton()) {

        camera.setDriverMode(false);
        camera.setPipelineIndex(3);

      }

      if (primaryDriver.getAButton()) {

        camera.setDriverMode(true);

      }

  }
}




//Old drivetrain code

/*public class Robot extends TimedRobot {

  private final WPI_TalonSRX m_leftMotor1 = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_rightMotor1 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(4);
  private final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(5);

  private final DifferentialDrive robotDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
  
  public final static CommandXboxController primaryDriver = new CommandXboxController(0);
  

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor1.setInverted(false);
    m_leftMotor1.setInverted(false);

    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);

    

  }

  

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    double throttle = primaryDriver.getRightTriggerAxis() - primaryDriver.getLeftTriggerAxis();

    robotDrive.arcadeDrive(primaryDriver.getLeftX(), -throttle);
    // visionmovement.vision();

  }
}  

*/
