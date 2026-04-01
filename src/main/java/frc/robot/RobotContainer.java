// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlignTurret;
import frc.robot.commands.CheckHubStatus;
import frc.robot.commands.DropIntake;
import frc.robot.commands.EmergencyScore;
import frc.robot.commands.JogTurretNegative;
import frc.robot.commands.JogTurretPositive;
import frc.robot.commands.ReverseAll;
import frc.robot.commands.RunIntakeIn;
import frc.robot.commands.RunIntakeOut;
import frc.robot.commands.RunKickerUp;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunSpindexer;
import frc.robot.commands.RunSpindexerRev;
import frc.robot.commands.Score;
import frc.robot.commands.ScoreDyn;
import frc.robot.commands.SearchForTarget;
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
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {

    // Subsystems
    private final Intake intake = new Intake();
    private final Turret turret = new Turret();
    private final TurretShooter shooter = new TurretShooter();
    private final Kicker kicker = new Kicker();
    private final Indexer indexer = new Indexer();
    private final LED led = new LED();

    // Drive speed multipliers
    private double translationMultiplier = 0.85;
    private double strafeMultiplier = 0.85;
    private double rotateMultiplier = 0.85;

    // Drivetrain speed limits
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Swerve drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.08)
            .withRotationalDeadband(MaxAngularRate * 0.08)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    // Controllers
    private final CommandXboxController driver   = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Drivetrain and telemetry
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    public void periodic(){}

    public RobotContainer() {
        configureDefaultCommands();
        configureDriverBindings();
        configureOperatorBindings();
        configureAutomation();
        drivetrain.registerTelemetry(logger::telemeterize);

        new EventTrigger("AutoAlignTurret").whileTrue(new AutoAlignTurret(turret).withTimeout(20));
        new EventTrigger("DropIntake").onTrue(new DropIntake(intake));
        new EventTrigger("ZeroTurret").onTrue(new SetTurretZeroish(turret));
        new EventTrigger("CenterTurret").onTrue(new SetTurretCenter(turret));
        new EventTrigger("Score").whileTrue(new Score(indexer, kicker, shooter, turret, drivetrain).withTimeout(7));

        NamedCommands.registerCommand("Score",new Score(indexer, kicker, shooter, turret, drivetrain).withTimeout(7));
        NamedCommands.registerCommand("RunIntake",new RunIntakeIn(intake).withTimeout(20));
 
        //create auto chooser in dashboard
        autoChooser = AutoBuilder.buildAutoChooser("Main"); 
        SmartDashboard.putData("Auto Mode", autoChooser); 
    }

    // ── Default Commands ─────────────────────────────────────────────────────

    private void configureDefaultCommands() {
        // Drivetrain: field-centric drive
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * translationMultiplier)
                     .withVelocityY(-driver.getLeftX() * MaxSpeed * strafeMultiplier)
                     .withRotationalRate(-driver.getRightX() * MaxAngularRate * rotateMultiplier)
            )
        );

        // Idle drivetrain when disabled
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Turret: always auto-aligning when no other command is running
        turret.setDefaultCommand(new AutoAlignTurret(turret));

        // Intake: always running in unless interrupted
        intake.setDefaultCommand(new RunIntakeIn(intake));

        // LED: always check what the status of the hub is and update Elastic accordingly
        led.setDefaultCommand(new CheckHubStatus(led));
    }

    // ── Driver Controller (port 0) ────────────────────────────────────────────

    private void configureDriverBindings() {

        //driver.x().whileTrue(new ScoreDyn(intake, indexer, kicker, shooter, turret));
        //driver.b().whileTrue(new RunIntakeOut(intake));
        //driver.leftTrigger().whileTrue(new RunKickerUp(kicker));
        //driver.rightTrigger().whileTrue(new RunShooter(shooter));
        //driver.leftBumper().whileTrue(new RunIntakeIn(intake));

        driver.rightBumper().whileTrue(new Score(indexer, kicker, shooter, turret, drivetrain));
        driver.leftBumper().whileTrue(new EmergencyScore(intake, indexer, kicker, shooter, turret));
        driver.a().whileTrue(new RunSpindexerRev(indexer));
        driver.start().whileTrue(new ReverseAll(kicker, intake, indexer));
        driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //left bumper to toggle drvetrain to low speed
        driver.rightBumper().whileTrue(new InstantCommand(() -> translationMultiplier = .11));

        driver.rightBumper().whileFalse(new InstantCommand(() -> translationMultiplier = 0.85));
        driver.rightBumper().whileTrue(new InstantCommand(() -> strafeMultiplier = .11));
        driver.rightBumper().whileFalse(new InstantCommand(() -> strafeMultiplier = 0.85));
        driver.rightBumper().whileTrue(new InstantCommand(() -> rotateMultiplier = .2));
        driver.rightBumper().whileFalse(new InstantCommand(() -> rotateMultiplier = 0.85));

    }

    // ── Operator Controller (port 1) ──────────────────────────────────────────

    private void configureOperatorBindings() {
        operator.b().onTrue(new SetHoodMax(shooter));
        operator.a().onTrue(new SetHoodMid(shooter));
        operator.x().onTrue(new SetHoodMin(shooter));

        operator.rightBumper().whileTrue(new JogTurretPositive(turret));
        operator.leftBumper().whileTrue(new JogTurretNegative(turret));

        operator.y().onTrue(new SetTurretZeroish(turret));
        operator.start().onTrue(new SetTurretCenter(turret));
    }

    // ── Automated Triggers ────────────────────────────────────────────────────

    private void configureAutomation() {
    // Auto-unjam indexer when jam is detected
    indexer.getJamTrigger().onTrue(new UnjamIndexer(indexer));

        new Trigger(() -> !LimelightHelpers.getTV("limelight-turret"))
        .debounce(0.75)
        .whileTrue(new SearchForTarget(turret));
    }

    // ── Autonomous ───────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
