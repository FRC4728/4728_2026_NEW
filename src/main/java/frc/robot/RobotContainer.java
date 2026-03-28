package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoAlignTurret;
import frc.robot.commands.CheckHubStatus;
import frc.robot.commands.DropIntake;
import frc.robot.commands.EmergencyScore;
import frc.robot.commands.JogTurretNegative;
import frc.robot.commands.JogTurretPositive;
import frc.robot.commands.ReverseAll;
import frc.robot.commands.RunIntakeIn;
import frc.robot.commands.RunSpindexerRev;
import frc.robot.commands.Score;
import frc.robot.commands.SetHoodMax;
import frc.robot.commands.SetHoodMid;
import frc.robot.commands.SetHoodMin;
import frc.robot.commands.SetTurretCenter;
import frc.robot.commands.SetTurretZeroish;
import frc.robot.commands.UnjamIndexer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;

public class RobotContainer {
    private final Intake intake = new Intake();
    private final Turret turret = new Turret();
    private final TurretShooter shooter = new TurretShooter();
    private final Kicker kicker = new Kicker();
    private final Indexer indexer = new Indexer();
    private final LED led = new LED();

    private double translationMultiplier = 0.85;
    private double strafeMultiplier = 0.85;
    private double rotateMultiplier = 0.85;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.08)
        .withRotationalDeadband(MaxAngularRate * 0.08)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser autoChooser;

    public void periodic() {}

    public RobotContainer() {
        configureDefaultCommands();
        configureDriverBindings();
        configureOperatorBindings();
        configureAutomation();

        drivetrain.registerTelemetry(logger::telemeterize);

        new EventTrigger("AutoAlignTurret").whileTrue(new AutoAlignTurret(turret, drivetrain).withTimeout(20));
        new EventTrigger("DropIntake").onTrue(new DropIntake(intake));
        new EventTrigger("ZeroTurret").onTrue(new SetTurretZeroish(turret));
        new EventTrigger("CenterTurret").onTrue(new SetTurretCenter(turret));
        new EventTrigger("Score").whileTrue(new Score(indexer, kicker, shooter, turret, drivetrain).withTimeout(7));

        NamedCommands.registerCommand("Score", new Score(indexer, kicker, shooter, turret, drivetrain).withTimeout(7));
        NamedCommands.registerCommand("RunIntake", new RunIntakeIn(intake).withTimeout(20));

        autoChooser = AutoBuilder.buildAutoChooser("Main");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed * translationMultiplier)
                .withVelocityY(-driver.getLeftX() * MaxSpeed * strafeMultiplier)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate * rotateMultiplier))
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        turret.setDefaultCommand(new AutoAlignTurret(turret, drivetrain));
        intake.setDefaultCommand(new RunIntakeIn(intake));
        led.setDefaultCommand(new CheckHubStatus(led));
    }

    private void configureDriverBindings() {
        driver.rightBumper().whileTrue(new Score(indexer, kicker, shooter, turret, drivetrain));
        driver.leftBumper().whileTrue(new EmergencyScore(intake, indexer, kicker, shooter, turret));
        driver.a().whileTrue(new RunSpindexerRev(indexer));
        driver.start().whileTrue(new ReverseAll(kicker, intake));
        driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.rightBumper().whileTrue(new InstantCommand(() -> translationMultiplier = .11));
        driver.rightBumper().whileFalse(new InstantCommand(() -> translationMultiplier = 0.85));
        driver.rightBumper().whileTrue(new InstantCommand(() -> strafeMultiplier = .11));
        driver.rightBumper().whileFalse(new InstantCommand(() -> strafeMultiplier = 0.85));
        driver.rightBumper().whileTrue(new InstantCommand(() -> rotateMultiplier = .2));
        driver.rightBumper().whileFalse(new InstantCommand(() -> rotateMultiplier = 0.85));
    }

    private void configureOperatorBindings() {
        operator.b().onTrue(new SetHoodMax(shooter));
        operator.a().onTrue(new SetHoodMid(shooter));
        operator.x().onTrue(new SetHoodMin(shooter));
        operator.rightBumper().whileTrue(new JogTurretPositive(turret));
        operator.leftBumper().whileTrue(new JogTurretNegative(turret));
        operator.y().onTrue(new SetTurretZeroish(turret));
        operator.start().onTrue(new SetTurretCenter(turret));
    }

    private void configureAutomation() {
        indexer.getJamTrigger().onTrue(new UnjamIndexer(indexer));
        // Removed Limelight-based SearchForTarget trigger for pose-only turret control.
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
