package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;
import org.firstinspires.ftc.teamcode.decodeCommands.DischargeCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.LimelightCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;
import org.opencv.video.KalmanFilter;

@TeleOp
public class WheatleyOpMode extends ActionOpMode {
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
    Motif motif = Motif.GPP;
    AutoShooter.TeamColor teamColor;

    @Override
    public void initialize() {
        teamColor = SavedValues.teamColor;
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
//        carouselSubsystem.setTicks(SavedValues.carouselTicks);

        mecanumDrive = new MecanumDrive(hardwareMap, SavedValues.position);

        limelightSubsystem = new LimelightSubsystem(hardwareMap, teamColor, mecanumDrive);

        driver = new GamepadEx(gamepad1);
        system = new GamepadEx(gamepad2);
        initButtons();

        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), teamColor));
        dischargeSubsystem.setDefaultCommand(new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, teamColor));
        driverA.whenPressed(new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem));
        driverB.whenPressed(new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive, teamColor));
        driverBack.whenPressed(new InstantCommand(() -> mecanumDrive.localizer.setPose(new Pose2d(mecanumDrive.localizer.getPose().position, Math.PI))));
        driverX.whenPressed(new SequentialCommandGroup(
                new CommandGroups.Shoot(intakeSubsystem, carouselSubsystem),
                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(800)));

        driverY.whenPressed(new IntakeCommands.OutTakeState(intakeSubsystem));
        driverDPadUp.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));
        mecanumDrive.lazyImu.get().resetYaw();
        systemB.whenPressed(() -> carouselSubsystem.artifactsInGoal = (carouselSubsystem.artifactsInGoal + 1) % 3);
        systemA.whenPressed(() -> carouselSubsystem.resetEncoders());
        schedule(new LimelightCommands.KalmanFilter(limelightSubsystem,mecanumDrive,dischargeSubsystem::getTurretAngle, SavedValues.covariances.getX(),SavedValues.covariances.getY()));
    }

    @Override
    public void run() {
        super.run();
        multipleTelemetry.addData("current", carouselSubsystem.getCurrent());
        multipleTelemetry.addData("turretAngle", dischargeSubsystem.getTurretAngle());
        multipleTelemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.addData("angle", mecanumDrive.localizer.getPose().heading.toDouble() * 180 / Math.PI);
        multipleTelemetry.addData("carouselAngle", carouselSubsystem.getAngle());
        multipleTelemetry.addData("carouselPosition", carouselSubsystem.getPosition());
        multipleTelemetry.addData("kalmanPos", LimelightCommands.KalmanFilter.position);
        multipleTelemetry.addData("rpm", dischargeSubsystem.getRPM());
        multipleTelemetry.addData("flyWheelPower", dischargeSubsystem.flyWheelMotor.motorEx.getPower());
        multipleTelemetry.update();
    }

    @Override
    public void end() {
        carouselSubsystem.resetEncoders();
        limelightSubsystem.stopLimelight();
        SavedValues.position = new Pose2d(63, 0, Math.PI);
        SavedValues.carouselTicks = 0;
        SavedValues.covariances = new Vector2d(0,0);

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
