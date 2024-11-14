// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.List;

import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // PID Controller for heading
    private PIDController headingPID;
    private static final double kP = 0.1; // Tuning constant
    private static final double kI = 0.0;   // Tuning constant
    private static final double kD = 0.008;   // Tuning constant
    private double setpoint; // Desired heading

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        
        headingPID = new PIDController(kP, kI, kD);
        headingPID.enableContinuousInput(-180, 180); // Assuming -180 to 180 degree heading
        
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true),
                m_robotDrive));
            
            
    }

    

    public void returnToHeading() {
        double currentAngle = m_robotDrive.getHeading(); // Get the current heading using the method you defined
        double setpoint = 0; // Desired heading, e.g., 0 for straight ahead
    
        double output = headingPID.calculate(currentAngle, setpoint);
    
        //System.out.println(currentAngle);
        // You can implement a tolerance check to avoid unnecessary driving
        double tolerance = 0.2; // Adjust this value as needed
        if (Math.abs(output) > tolerance) {
            // Drive with the PID output as the rotation speed, scaled for responsiveness
            m_robotDrive.drive(0, 0, output, true, false);
        } else  {
            // Optionally stop the robot if within tolerance
            m_robotDrive.drive(0, 0, 0, true, false);
        }
    }
    public void faceAngle(float setpoint) {
        double currentAngle = m_robotDrive.getHeading(); // Get the current heading using the method you defined
        
    
        double output = headingPID.calculate(currentAngle, setpoint);
    
        System.out.println(currentAngle + " " + setpoint);
        // You can implement a tolerance check to avoid unnecessary driving
        double tolerance = 0.2; // Adjust this value as needed
        if (Math.abs(output) > tolerance) {
            // Drive with the PID output as the rotation speed, scaled for responsiveness
            m_robotDrive.drive(0, 0, output, true, false);
        } else {
            // Optionally stop the robot if within tolerance
            m_robotDrive.drive(0, 0, 0, true, false);
        }
    }
    public void checkDPad() {
        int pov = m_driverController.getPOV();
        if (pov != -1)
            faceAngle(-pov);
        }

    

    // Call this method in your robot's periodic methods

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a {@link JoystickButton}.
     */

        

    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kR1.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));
        new JoystickButton(m_driverController, 2)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(), 
                m_robotDrive));
        new JoystickButton(m_driverController, 1)
            .whileTrue(new RunCommand(
                () -> returnToHeading(), 
                m_robotDrive));
        new POVButton(m_driverController, 0)
            .whileTrue(new RunCommand(
                () -> checkDPad(),
                m_robotDrive));
        new POVButton(m_driverController, 90)
            .whileTrue(new RunCommand(
                () -> checkDPad(),
                m_robotDrive));
        new POVButton(m_driverController, 180)
            .whileTrue(new RunCommand(
                () -> checkDPad(),
                m_robotDrive));
        new POVButton(m_driverController, 270)
            .whileTrue(new RunCommand(
                () -> checkDPad(),
                m_robotDrive));

        
        


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
}
