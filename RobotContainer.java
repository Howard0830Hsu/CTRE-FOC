// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.pivot;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Joystick joystick = new Joystick(0);
private pivot pivot = new pivot();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRawAxis(1) *0.5* MaxSpeed) // Drive forward with Axis 1 (Y-Axis, forward/backward)
                    .withVelocityY(-joystick.getRawAxis(0) *0.5* MaxSpeed) // Drive left with Axis 0 (X-Axis, left/right)
                    .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate) // Rotate with Axis 4 (Right stick X-axis)
            )
        );

        // Button mappings for Logitech F310
        new JoystickButton(joystick, 3) // Button A
                .whileTrue(drivetrain.applyRequest(() -> brake));
        new JoystickButton(joystick, 2) // Button B
                .whileTrue(drivetrain.applyRequest(() ->
                        point.withModuleDirection(new Rotation2d(-joystick.getRawAxis(1), -joystick.getRawAxis(0)))
                ));

        // POV (D-Pad) mappings
        new POVButton(joystick, 0) // POV Up
                .whileTrue(drivetrain.applyRequest(() ->
                        forwardStraight.withVelocityX(0.5).withVelocityY(0))
                );
        new POVButton(joystick, 180) // POV Down
                .whileTrue(drivetrain.applyRequest(() ->
                        forwardStraight.withVelocityX(-0.5).withVelocityY(0))
                );

        // Run SysId routines when holding Back and X/Y.
        new Trigger(() -> joystick.getRawButton(7) && joystick.getRawButton(4)) // Back + Y
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        new Trigger(() -> joystick.getRawButton(7) && joystick.getRawButton(3)) // Back + X
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        new Trigger(() -> joystick.getRawButton(8) && joystick.getRawButton(4)) // Start + Y
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        new Trigger(() -> joystick.getRawButton(8) && joystick.getRawButton(3)) // Start + X
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        new JoystickButton(joystick, 1) // Left Bumper
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
       /*  new Trigger(()->operatorJoystick.getRawButton(5)).whileTrue(elevator.CoralL0Command());   
        new Trigger(()->operatorJoystick.getRawButton(4)).whileTrue(elevator.CoralL1Command());
        new Trigger(()->operatorJoystick.getRawButton(6)).whileTrue(elevator.CoralL2Command());
        new Trigger(()->operatorJoystick.getRawButton(8)).whileTrue(elevator.CoralL3Command());
        new Trigger(()->operatorJoystick.getRawButton(3)).whileTrue(elevator.CoralL4Command());
        new Trigger(()->operatorJoystick.getRawButton(7)).whileTrue(Commands.parallel(left.shoot1Command(),coralshooter.shoot1Command()));
        new POVButton(operatorJoystick,0) .whileTrue(Commands.parallel(left.shoot35Command(),coralshooter.shoot35Command()));
        new POVButton(operatorJoystick,180) .whileTrue(Commands.parallel(left.shoot3Command(),coralshooter.shoot3Command()));*/
        new Trigger(()->joystick.getRawButton(6)).whileTrue(pivot.intakeCoral());
}

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
