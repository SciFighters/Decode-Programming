package org.firstinspires.ftc.teamcode.needle.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.needle.commands.ArmAxisCommands;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.needle.commands.TelescopicArmCommands;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

@TeleOp
public class Needle extends ActionOpMode {
//    ArmAxisSubsystem armAxisSubsystem = new ArmAxisSubsystem(hardwareMap);
//    TelescopicArmSubsystem telescopicArmSubsystem= new TelescopicArmSubsystem(hardwareMap);

    MecanumDrive mecanumDrive;
    GamepadEx gamepad;
    Button a;
    Button b;
    Button y;
    Button x;
    Button dPadDown;
    Button dPadUp;
    Button dPadRight;
    Button dPadLeft;
    Button rightBumper;
    Button leftBumper;
    Button start;
    Button back;
    Button leftStick, rightStick;
    @Override
    public void initialize() {
        mecanumDrive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,-64),Math.PI/2));
        gamepad = new GamepadEx(gamepad1);
        initButtons();
//        armAxisSubsystem.setDefaultCommand(new ArmAxisCommands.AxisManual(armAxisSubsystem, gamepad::getRightY));
//        telescopicArmSubsystem.setDefaultCommand(new TelescopicArmCommands.ExtensionManual(telescopicArmSubsystem, () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive,() -> gamepad.getLeftX(),() -> gamepad.getLeftY(),() -> gamepad.getRightX()));
    }

    @Override
    public void run() {
        super.run();
        mecanumDrive.updatePoseEstimate();
        multipleTelemetry.addData("heading deg",Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
        multipleTelemetry.update();
    }

    private void initButtons() {

        a = new GamepadButton(gamepad, GamepadKeys.Button.A);
        b = new GamepadButton(gamepad, GamepadKeys.Button.B);
        y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        x = new GamepadButton(gamepad, GamepadKeys.Button.X);
        dPadDown = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);
        dPadUp = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
        dPadRight = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT);
        dPadLeft = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
        rightBumper = new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        leftBumper = new GamepadButton(gamepad, GamepadKeys.Button.LEFT_BUMPER);
        start = new GamepadButton(gamepad, GamepadKeys.Button.START);
        leftStick = new GamepadButton(gamepad, GamepadKeys.Button.LEFT_STICK_BUTTON);
        rightStick = new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        back = new GamepadButton(gamepad, GamepadKeys.Button.BACK);


    }
}
