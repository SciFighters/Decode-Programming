package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
@Config
@TeleOp
public class ManualCarouselTest extends ActionOpMode {
    DcMotor carousel, intake;
    double spinPower = 0, intakePower = 0;
    GamepadEx gamepad;
    GamepadButton up, down, left, right, a;
    boolean work = true;
    public static double extraIntake = 0;
    @Override
    public void initialize() {
        carousel = hardwareMap.dcMotor.get("carousel");
        intake = hardwareMap.dcMotor.get("intakeMotor");
        gamepad = new GamepadEx(gamepad1);
//        up = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
//        down = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);
        left = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
        right = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT);
        a = new GamepadButton(gamepad, GamepadKeys.Button.A);
//        up.whenPressed(() -> spinPower += 0.1);
//        down.whenPressed(() -> spinPower -= 0.1);
        left.whenPressed(() -> intakePower -= 0.1);
        right.whenPressed(() -> intakePower += 0.1);
        a.whenPressed(() -> work = !work);
    }

    @Override
    public void run() {
        super.run();
        if (work){
            intake.setPower(intakePower + extraIntake);
            carousel.setPower(gamepad.getRightX()/2);
        }else {
            intake.setPower(0);
            carousel.setPower(0);
        }

        multipleTelemetry.addData("intakePower", intakePower);
        multipleTelemetry.addData("carouselPower", spinPower);
        multipleTelemetry.update();
    }
}
