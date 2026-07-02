package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;

@Config
@TeleOp(group = "tests")
public class FlyWheelTester extends ActionOpMode {
    GamepadEx gamepad;
    GamepadButton increaseRPMButton, decreaseRPMButton, increasePowerButton, decreasePowerButton, increaseWantedAngleButton, decreaseWantedAngleButton;

    private double power = 0;
    public static double wantedRpm = 6000;
    private double wantedAngle = 28;

    private DcMotor m2;
    private DischargeSubsystem dischargeSubsystem;


    private void setupBindings() {
        increaseRPMButton = new GamepadButton(gamepad, GamepadKeys.Button.A);
        decreaseRPMButton = new GamepadButton(gamepad, GamepadKeys.Button.B);
        increasePowerButton = new GamepadButton(gamepad, GamepadKeys.Button.X);
        decreasePowerButton = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        increaseWantedAngleButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
        decreaseWantedAngleButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);

        increaseWantedAngleButton.whenPressed(() -> wantedAngle += 1);
        decreaseWantedAngleButton.whenPressed(() -> wantedAngle -= 1);
        increaseRPMButton.whenPressed(() -> wantedRpm += 100);
        decreaseRPMButton.whenPressed(() -> wantedRpm -= 100);
        increasePowerButton.whenPressed(() -> power += 0.1);
        decreasePowerButton.whenPressed(() -> power -= 0.1);
    }

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        setupBindings();
    }

    @Override
    public void run() {
        super.run();

        dischargeSubsystem.setRampDegree(wantedAngle);
        dischargeSubsystem.setFlyWheelRPM(wantedRpm);

        multipleTelemetry.addData("rpm", dischargeSubsystem.getRPM());
        multipleTelemetry.addData("power", dischargeSubsystem.getFlyWheelPower());
        multipleTelemetry.addData("wanted", wantedRpm);
        multipleTelemetry.addData("angle", wantedAngle);
        multipleTelemetry.update();
    }
}
