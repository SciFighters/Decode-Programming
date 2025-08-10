package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.sql.Array;
import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 2197.7932709725296; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -3701.757769091782; // x position of the perpendicular encoder (in tick units)
//        -2170.3645688210336
//        -3762.3781495719995
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        double mmPerTick = inPerTick *25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM); // Not Accepting inPerTick. Use mmPerTick
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

//        driver.setEncoderResolution(1.0 / inPerTick, DistanceUnit.INCH);
//        driver.setOffsets(inPerTick * PARAMS.parYTicks, inPerTick * PARAMS.perpXTicks, DistanceUnit.INCH);
//        driver.resetDeviceConfigurationForOpMode();

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();
        driver.recalibrateIMU();

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    public double[] getRawValues() {
        double[] arr = {driver.getEncoderX(), driver.getEncoderY()};
        return arr;
    }

    public void resetPosAndIMU() {
        driver.resetPosAndIMU();
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return driver.getDeviceStatus();
    }

    public double getFreq() {
        return driver.getFrequency();
    }
}
