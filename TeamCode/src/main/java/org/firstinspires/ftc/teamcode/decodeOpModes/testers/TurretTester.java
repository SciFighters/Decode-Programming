package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;

@TeleOp(group = "tests")
public class TurretTester extends ActionOpMode {
    GamepadEx gamepad;
    GamepadButton A, B, X;
    DischargeSubsystem dischargeSubsystem;
    boolean manual = true;
    double power = 1;
    ElapsedTime time;
    double velocity, lastVelocity, lastTime, currentTime;
    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        dischargeSubsystem.resetTurret();
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        A.whenPressed(() -> manual = !manual);
        B.whenPressed(() -> power += 0.01);
        X.whenPressed(new SequentialCommandGroup( new InstantCommand((() ->{
            lastVelocity = dischargeSubsystem.getRPS();
            lastTime = time.seconds();
            dischargeSubsystem.setTurretPower(0.5);
                } )),
                new WaitCommand(500),
                new InstantCommand(() -> {
                    velocity = dischargeSubsystem.getRPS();
                    currentTime = time.seconds();
                    dischargeSubsystem.setTurretPower(0);
                })));
        time = new ElapsedTime();
        lastTime = 0;
        lastVelocity = 0;
        currentTime = 1;
        velocity = 1;
    }

    @Override
    public void run() {
//        if (manual) {
//            dischargeSubsystem.setTurretPower(gamepad.getRightX() * (0.5 + 0.5 * gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
//        } else {
//            dischargeSubsystem.setTurretPower(power);
//        }
//        velocity = dischargeSubsystem.getRPS();
//        currentTime = time.seconds();
        double acceleration = (velocity - lastVelocity) / (currentTime - lastTime);
        multipleTelemetry.addData("power", power);
        multipleTelemetry.addData("speed", dischargeSubsystem.getRPS());
        multipleTelemetry.addData("acceleration",acceleration);
        multipleTelemetry.addData("ticks", dischargeSubsystem.getTurretPosition());
        multipleTelemetry.addData("angle", dischargeSubsystem.getTurretAngle());
        multipleTelemetry.update();
        super.run();

//        lastVelocity = velocity;
//        lastTime = currentTime;
    }
}
