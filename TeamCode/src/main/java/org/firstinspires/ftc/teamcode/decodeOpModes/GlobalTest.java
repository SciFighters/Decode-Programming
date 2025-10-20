package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;

@TeleOp
public class GlobalTest extends ActionOpMode {
    DcMotorEx carousel, intake, discharge;
    double carouselPower = 0, intakePower = 0, dischargePower = 0;
    int motorCount = 0;
    GamepadEx gamepad;

    int currentMotor = 0;

    @Override
    public void initialize() {
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        discharge = hardwareMap.get(DcMotorEx.class, "discharge");

        gamepad = new GamepadEx(gamepad1);

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            currentMotor = (currentMotor + 1) % motorCount; });
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            currentMotor = (currentMotor - 1) % motorCount; });

        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP).whenPressed(() -> changePower(0.1));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> changePower(-0.1));
    }

    private void changePower(double power) {
        switch (currentMotor) {
            case 0: carouselPower += power;
            break;
            case 1: intakePower += power;
            break;
            case 2: dischargePower += power;
            break;
        }
    }

    @Override
    public void run() {
        super.run();
        carousel.setPower(carouselPower);
        intake.setPower(intakePower);
        discharge.setPower(dischargePower);

        telemetry.addData("Current Motor", currentMotor);
        telemetry.addData("Carousel Power", carouselPower);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Discharge Power", dischargePower);
    }
}
