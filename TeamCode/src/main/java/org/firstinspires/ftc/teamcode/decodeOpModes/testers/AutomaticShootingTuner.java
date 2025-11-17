package org.firstinspires.ftc.teamcode.decodeOpModes.testers;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;
import org.firstinspires.ftc.teamcode.decodeCommands.DischargeCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

@TeleOp(group = "tests")
public class AutomaticShootingTuner extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    GamepadEx driver, system;
    Button driverA, systemA;
    Button driverB, systemB;
    Button driverY, systemY;
    Button driverX, systemX;
    Button driverDPadDown, systemDPadDown;
    Button driverDPadUp, systemDPadUp;
    Button driverDPadRight, systemDPadRight;
    Button driverDPadLeft, systemDPadLeft;
    Button driverRightBumper, systemRightBumper;
    Button driverLeftBumper, systemLeftBumper;
    Button driverStart, systemStart;
    Button driverBack, systemBack;
    Button driverLeftStick, systemLeftStick, driverRightStick, systemRightStick;
    double wantedRPM = 1000, wantedDegree = 50;

    @Override
    public void initialize() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, -64), Math.PI));
        mecanumDrive.lazyImu.get().resetYaw();
        limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED, mecanumDrive);

        driver = new GamepadEx(gamepad1);
        system = new GamepadEx(gamepad2);
        initButtons();
        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), () -> 0.6 + 0.4 * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        driverA.whenPressed(new IntakeCommands.IntakeState(intakeSubsystem));
        driverB.whenPressed(new CarouselCommands.MoveToPos(carouselSubsystem, carouselSubsystem.spinConversion * 1.1));
        driverX.whenPressed(new SequentialCommandGroup(
                new IntakeCommands.TransferState(intakeSubsystem),
                new WaitCommand(2000),
                new CarouselCommands.Discharge(carouselSubsystem)
//                new IntakeCommands.TransferState(intakeSubsystem)
        ));
        systemDPadUp.whenPressed(() -> wantedRPM += 100);
        systemDPadDown.whenPressed(() -> wantedRPM -= 100);
        systemA.whenPressed(() -> wantedDegree += 2);
        systemY.whenPressed(() -> wantedDegree -= 2);
        systemX.whenPressed(() -> wantedDegree += 0.2);
        systemB.whenPressed(() -> wantedDegree -= 0.2);
        driverRightStick.whenPressed(
                new ParallelCommandGroup(
//                        new DischargeCommands.setState(dischargeSubsystem, 0, 54),
                        new IntakeCommands.IntakeState(intakeSubsystem),
                        new CarouselCommands.MoveToPos(carouselSubsystem, carouselSubsystem.spinConversion * 1)
                ));
        driverY.whenPressed(new IntakeCommands.OutTakeState(intakeSubsystem));
        driverDPadDown.whenPressed(new DischargeCommands.setState(dischargeSubsystem, 0, 54));
        driverDPadUp.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));

        limelightSubsystem.startLimelight();
    }

    @Override
    public void run() {
//        com.seattlesolvers.solverslib.geometry.Vector2d pos =
//                limelightSubsystem.getRobotPos(mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getYaw() / 180 * Math.PI, dischargeSubsystem.getTurretAngle());
//        double launchAngle =
//                AutoShooter.getLaunchAngle(new Pose2d(pos.getX(), pos.getY(), mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getYaw() / 180 * Math.PI), AutoShooter.TeamColor.RED);
//        if (pos.getX() < 1000) {
//            double power = -((launchAngle + 360) % 360 - dischargeSubsystem.getTurretAngle()) * 0.028;
//            power += Math.signum(power) * 0.04;
//            power = Range.clip(power, -0.4, 0.4);
//            dischargeSubsystem.setTurretPower(power);
//        } else {
//            dischargeSubsystem.setTurretPower(0);
//        }
        dischargeSubsystem.setFlyWheelRPM(wantedRPM);
        dischargeSubsystem.setRampDegree(wantedDegree);
        super.run();
        multipleTelemetry.addData("turretAngle", dischargeSubsystem.getTurretAngle());
//        multipleTelemetry.addData("x", pos.getX());
//        multipleTelemetry.addData("y", pos.getY());
//        multipleTelemetry.addData("angle", launchAngle);
        multipleTelemetry.addData("rampDegree", wantedDegree);
        multipleTelemetry.addData("wantedRPM", wantedRPM);
        multipleTelemetry.addData("rpm", dischargeSubsystem.getRPM());
        multipleTelemetry.addData("left artifact", carouselSubsystem.colorIdentifier(carouselSubsystem.leftColorSensor));
        multipleTelemetry.addData("middle artifact", carouselSubsystem.colorIdentifier(carouselSubsystem.middleColorSensor));
        multipleTelemetry.addData("right artifact", carouselSubsystem.colorIdentifier(carouselSubsystem.rightColorSensor));
        multipleTelemetry.update();
    }

    @Override
    public void end() {
        limelightSubsystem.stopLimelight();
    }

    public void initButtons() {
        driverA = new GamepadButton(driver, GamepadKeys.Button.A);
        driverB = new GamepadButton(driver, GamepadKeys.Button.B);
        driverY = new GamepadButton(driver, GamepadKeys.Button.Y);
        driverX = new GamepadButton(driver, GamepadKeys.Button.X);
        driverDPadDown = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        driverDPadUp = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        driverDPadRight = new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT);
        driverDPadLeft = new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT);
        driverRightBumper = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        driverLeftBumper = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        driverStart = new GamepadButton(driver, GamepadKeys.Button.START);
        driverLeftStick = new GamepadButton(driver, GamepadKeys.Button.LEFT_STICK_BUTTON);
        driverRightStick = new GamepadButton(driver, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        driverBack = new GamepadButton(driver, GamepadKeys.Button.BACK);
        systemA = new GamepadButton(system, GamepadKeys.Button.A);
        systemB = new GamepadButton(system, GamepadKeys.Button.B);
        systemY = new GamepadButton(system, GamepadKeys.Button.Y);
        systemX = new GamepadButton(system, GamepadKeys.Button.X);
        systemDPadDown = new GamepadButton(system, GamepadKeys.Button.DPAD_DOWN);
        systemDPadUp = new GamepadButton(system, GamepadKeys.Button.DPAD_UP);
        systemDPadRight = new GamepadButton(system, GamepadKeys.Button.DPAD_RIGHT);
        systemDPadLeft = new GamepadButton(system, GamepadKeys.Button.DPAD_LEFT);
        systemRightBumper = new GamepadButton(system, GamepadKeys.Button.RIGHT_BUMPER);
        systemLeftBumper = new GamepadButton(system, GamepadKeys.Button.LEFT_BUMPER);
        systemStart = new GamepadButton(system, GamepadKeys.Button.START);
        systemLeftStick = new GamepadButton(system, GamepadKeys.Button.LEFT_STICK_BUTTON);
        systemRightStick = new GamepadButton(system, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        systemBack = new GamepadButton(system, GamepadKeys.Button.BACK);
    }
}

