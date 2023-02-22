// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot;



import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.vision.visionmovement;



/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(9);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(3);

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(2);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("Metrobots3324");

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

  @Override
  public void robotInit() {

    m_rightMotor1.setInverted(false);
    m_leftMotor1.setInverted(false);

    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);

  }

  @Override
  public void teleopPeriodic() {

      double forwardSpeed;
      double rotationSpeed;


      if (primaryDriver.getYButton()) {
          // Vision-alignment mode
          // Query the latest result from PhotonVision
          var result = camera.getLatestResult();

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

  }
}

