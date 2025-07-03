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
    ArmAxisSubsystem armAxisSubsystem = new ArmAxisSubsystem(hardwareMap);
    TelescopicArmSubsystem telescopicArmSubsystem= new TelescopicArmSubsystem(hardwareMap);
    MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,0),Math.PI/2));
    GamepadEx driverGamepad;
    Button driverA;
    Button driverB;
    Button driverY;
    Button driverX;
    Button driverDPadDown;
    Button driverDPadUp;
    Button driverDPadRight;
    Button driverDPadLeft;
    Button driverRightBumper;
    Button driverLeftBumper;
    Button driverStart;
    Button driverBack;
    Button driverLeftStick, driverRightStick;
    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        initButtons();
        armAxisSubsystem.setDefaultCommand(new ArmAxisCommands.AxisManual(armAxisSubsystem, driverGamepad::getRightY));
        telescopicArmSubsystem.setDefaultCommand(new TelescopicArmCommands.ExtensionManual(telescopicArmSubsystem, () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive,() -> driverGamepad.getLeftY(),() -> -driverGamepad.getLeftX(),() -> driverGamepad.getRightX()));
        driverA.whenPressed(new ArmAxisCommands.AxisGoTo(armAxisSubsystem,120));
    }
    private void initButtons() {

        driverA = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        driverB = new GamepadButton(driverGamepad, GamepadKeys.Button.B);
        driverY = new GamepadButton(driverGamepad, GamepadKeys.Button.Y);
        driverX = new GamepadButton(driverGamepad, GamepadKeys.Button.X);
        driverDPadDown = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN);
        driverDPadUp = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP);
        driverDPadRight = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT);
        driverDPadLeft = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT);
        driverRightBumper = new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        driverLeftBumper = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        driverStart = new GamepadButton(driverGamepad, GamepadKeys.Button.START);
        driverLeftStick = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON);
        driverRightStick = new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        driverBack = new GamepadButton(driverGamepad, GamepadKeys.Button.BACK);


    }
}
