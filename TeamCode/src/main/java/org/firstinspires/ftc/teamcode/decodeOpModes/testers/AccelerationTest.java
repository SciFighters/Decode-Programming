package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;

@Config
@TeleOp(group = "tests")
public class AccelerationTest extends ActionOpMode {
    GamepadEx gamepad;
    GamepadButton A, B, X, Y, up, down;
    ElapsedTime time;
    DischargeSubsystem dischargeSubsystem;
    public static double wantedRpm = 3900;
    double timeTaken = 0;
    boolean reached;

    @Override
    public void initialize() {
        reached = false;
        time = new ElapsedTime();
//        m2 = hardwareMap.dcMotor.get("turretMotor");
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);

    }

    @Override
    public void initialize_loop() {
        time.reset();
    }

    @Override
    public void run() {
        if (!reached && Math.abs(dischargeSubsystem.getRPM() - wantedRpm) < 100) {
            timeTaken = time.seconds();
            reached = true;
        }
        multipleTelemetry.addData("time taken", timeTaken);
        dischargeSubsystem.setFlyWheelRPM(wantedRpm);
        multipleTelemetry.update();
        super.run();
    }


}
