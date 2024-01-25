// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;



/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

    // Constants for PID control
    private final double kP = 0.1;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kTolerance = 1.0;

    private double integralError = 0.0;
    private double previousError = 0.0;

    // Constants for Limelight camera
    private final double kImageHeight = 240.0; // Height of the camera image in pixels
    private final double kVerticalFOV = 49.7; // Vertical field of view of the camera in degrees
    private final double kTagSize = 0.1778; // Size of the AprilTag in meters (7 inches)

    double calculateDistanceFromArea(double area) {
      // Calculate the height of the target in pixels based on the target area
      double targetHeight = 2.0 * kTagSize / Math.sqrt(area);
      
      // Calculate the distance to the target based on the height and the camera's vertical FOV
      double distance = kImageHeight / (2.0 * targetHeight * Math.tan(Math.toRadians(kVerticalFOV / 2.0)));
      
      return distance;
  }



  

  public static final WPI_TalonSRX m_leftMotor1 = new WPI_TalonSRX(2);
  public static final WPI_TalonSRX m_rightMotor1 = new WPI_TalonSRX(3);
  public static final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(4);
  public static final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(5);

  private final DifferentialDrive robotDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
  
  public final static CommandXboxController primaryDriver = new CommandXboxController(0);


  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to inve`rt the left side instead.
    m_rightMotor1.setInverted(true);
    m_rightMotor1.setInverted(false);

    m_rightMotor2.follow(m_rightMotor1);
    m_leftMotor2.follow(m_leftMotor1);

  }



  
  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    

    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);


    if (tv == 1){

     // Calculate distance to target based on target area
     double targetDistance = calculateDistanceFromArea(ta);

    // Calculate target position relative to robot
    double targetX = targetDistance * Math.sin(Math.toRadians(tx));
    double targetY = targetDistance * Math.sin(Math.toRadians(ty));

    // Calculate error in target position
    double targetXError = 0.0; // Set desired target X position here
    double targetYError = 0.0; // Set desired target Y position here
    double xError = targetX - targetXError;
    double yError = targetY - targetYError;

    // PID control for steering
    double steeringError = -tx;
    integralError += steeringError;
    double derivativeError = steeringError - previousError;
    double steeringAdjustment = kP * steeringError + kI * integralError + kD * derivativeError;

    // PID control for distance
    double distanceError = targetDistance - kTolerance;
    double distanceAdjustment = kP * distanceError;

    // PID control for X position
    double xAdjustment = kP * xError;

    // PID control for Y position
    double yAdjustment = kP * yError;

    robotDrive.arcadeDrive(distanceAdjustment + xAdjustment, steeringAdjustment + yAdjustment);

      
    }


    if (tv == 0) {

      double speed = primaryDriver.getRightY();
      double steering = primaryDriver.getLeftX();

      robotDrive.arcadeDrive(speed, steering);

    }

    }
     

  }



