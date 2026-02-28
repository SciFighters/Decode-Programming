package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
@TeleOp(group = "tests")
public class TurretTester extends ActionOpMode {
    GamepadEx gamepad;
    GamepadButton A, B;
    DischargeSubsystem dischargeSubsystem;
    boolean manual = true;
    double power = 0;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        dischargeSubsystem.resetTurret();
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        A.whenPressed(() -> manual = !manual);
        B.whenPressed(() -> power += 0.01);
    }

    @Override
    public void run() {
        if(manual){
            dischargeSubsystem.setTurretPower(gamepad.getRightX() *(0.5 + 0.5 * gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        } else{
            dischargeSubsystem.setTurretPower(power);
        }
        multipleTelemetry.addData("power", power);
        multipleTelemetry.addData("speed", dischargeSubsystem.getRPS());
        multipleTelemetry.addData("ticks",dischargeSubsystem.getTurretPosition());
        multipleTelemetry.addData("angle",dischargeSubsystem.getTurretAngle());
        multipleTelemetry.update();
        super.run();
    }
}
