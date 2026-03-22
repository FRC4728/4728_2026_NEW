package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HubTracker;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import frc.robot.Constants;

public class LED extends SubsystemBase {
    private final CANdle candle = new CANdle(Constants.CandleConstants.canID);

    private static final int START_LED = 0;
    private static final int END_LED = 7;
    private static final int ANIMATION_SLOT = 0;

    private enum LEDState {
        OFF,
        RED,
        BLUE,
        FLASH_RED,
        FLASH_BLUE,
        RAINBOW
    }

    private LEDState currentState = null;

    public LED() {
        var config = new CANdleConfiguration()
            .withLED(new LEDConfigs()
                .withBrightnessScalar(1.0)
                .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs))
            .withCANdleFeatures(new CANdleFeaturesConfigs()
                .withEnable5VRail(Enable5VRailValue.Enabled)
                .withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled));

        candle.getConfigurator().apply(config);
        off();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Hub Active", HubTracker.isAllianceHubActive());

        Optional<Alliance> alliance = DriverStation.getAlliance();
        SmartDashboard.putString("Alliance",
            alliance.map(Enum::name).orElse("Unknown"));
    }

    private void clearAnimation() {
        candle.setControl(new EmptyAnimation(ANIMATION_SLOT));
    }

    public void setRed() {
        if (currentState == LEDState.RED) return;
        currentState = LEDState.RED;

        clearAnimation();
        candle.setControl(
            new SolidColor(START_LED, END_LED)
                .withColor(new RGBWColor(255, 0, 0))
        );
    }

    public void setBlue() {
        if (currentState == LEDState.BLUE) return;
        currentState = LEDState.BLUE;

        clearAnimation();
        candle.setControl(
            new SolidColor(START_LED, END_LED)
                .withColor(new RGBWColor(0, 0, 255))
        );
    }

    public void flashRed() {
        if (currentState == LEDState.FLASH_RED) return;
        currentState = LEDState.FLASH_RED;

        candle.setControl(
            new StrobeAnimation(START_LED, END_LED)
                .withSlot(ANIMATION_SLOT)
                .withColor(new RGBWColor(255, 0, 0))
                .withFrameRate(8)
        );
    }

    public void flashBlue() {
        if (currentState == LEDState.FLASH_BLUE) return;
        currentState = LEDState.FLASH_BLUE;

        candle.setControl(
            new StrobeAnimation(START_LED, END_LED)
                .withSlot(ANIMATION_SLOT)
                .withColor(new RGBWColor(0, 0, 255))
                .withFrameRate(8)
        );
    }

    public void setRainbow() {
        if (currentState == LEDState.RAINBOW) return;
        currentState = LEDState.RAINBOW;

        candle.setControl(
            new RainbowAnimation(START_LED, END_LED)
                .withSlot(ANIMATION_SLOT)
                .withFrameRate(8)
        );
    }

    public void off() {
        if (currentState == LEDState.OFF) return;
        currentState = LEDState.OFF;

        clearAnimation();
        candle.setControl(
            new SolidColor(START_LED, END_LED)
                .withColor(new RGBWColor(0, 0, 0))
        );
    }

    public void checkHubStatusAndSetLED() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (!RobotState.isAutonomous()) {
            if (alliance.isPresent() && alliance.get() == Alliance.Blue && HubTracker.isAllianceHubActive()) {
                setBlue();
            }
            else if (alliance.isPresent() && alliance.get() == Alliance.Red && HubTracker.isAllianceHubActive()) {
                setRed();
            }
            else if (alliance.isPresent() && !HubTracker.isAllianceHubActive()) {
                off();
            }
            else {
                setRainbow();
            }
        } else {
            if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
                flashBlue();
            }
            else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                flashRed();
            }
            else {
                setRainbow();
            }
        }
    }
}