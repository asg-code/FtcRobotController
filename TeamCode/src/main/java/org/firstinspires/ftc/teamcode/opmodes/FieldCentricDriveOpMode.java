package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp(name="Field Centric Drive OpMode", group="Tests")
public class FieldCentricDriveOpMode extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private IMU imu;

    public void runOpMode() {
        // Initialize the mecanum drive train
        mecanumDrive.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot (
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Run the loop while the opmode is active
        while (opModeIsActive()) {
            // Get the gamepad inputs
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Set the powers for the mecanum drive train
            driveFieldRelative(forward, strafe, rotate);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void driveFieldRelative(double forward, double strafe, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // convert to polar
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(forward, strafe);

        // rotate theta by robot angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        mecanumDrive.drive(newForward, newStrafe, rotate);
    }
}
