package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoAlignTurret;
import frc.robot.commands.RunIntakeIn;
import frc.robot.commands.RunKickerUp;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunSpindexer;
import frc.robot.commands.Score;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;

public class RobotContainer {

    private final Intake    intake    = new Intake();
    private final Turret    turret    = new Turret();
    private final TurretShooter shooter = new TurretShooter();
    private final Kicker    kicker    = new Kicker();
    private final Indexer   indexer   = new Indexer();

    private final double MaxSpeed       = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point    = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick  = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Drivetrain
        //joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        //joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        // Turret auto-align (joystick2 left bumper)
        joystick2.leftBumper().whileTrue(new AutoAlignTurret(turret, drivetrain));

        // Mechanisms
        joystick.x().whileTrue(new RunIntakeIn(intake));
        joystick.y().whileTrue(new RunKickerUp(kicker));
        joystick.b().whileTrue(new RunSpindexer(indexer));
        joystick.a().whileTrue(new RunShooter(shooter));

        // Score
        joystick.rightBumper().whileTrue(new Score(intake, indexer, kicker, shooter));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
