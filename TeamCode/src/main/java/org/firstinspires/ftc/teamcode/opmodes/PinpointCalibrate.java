package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PinpointOdometryRobot;

/**
 * PinpointCalibrate - A simple teleop opmode for calibrating the Pinpoint Odometry system.
 * You can use this opmode to check if goBilda Pinpoint Odometry is working correctly.
 * This opmode will display the drive, strafe values and the heading value.
 *
 * Drive is robot movement in the forward/back direction and strafe is robot movement in the
 * left/right direction. Heading is rotation of the robot in degrees along the z axis (vertical line
 * running through the pin point odometer or imu). This is also known as the yaw value.
 *
 * Move the robot in a straight line and check if the values increase. If they descrease,
 * adjust INVERT_DRIVE_ODOMETRY flag in PinpointOdometryRobot.java
 *
 * Move the robot left and check if the strafe value increases. If it decreases, adjust
 * INVERT_STRAFE_ODOMETRY flag in PinpointOdometryRobot.java
 *
 * Rotate the robot counter clockwise and check if the heading value increases. +ve = CCW, -ve = CW.
 * If using imu on goBilda Pinpoint, make sure it mounted correctly. If not, adjust the orientation.
 */
@TeleOp(name="Pinpoint Calibrate", group="Tests")
public class PinpointCalibrate extends LinearOpMode {
    // get an instance of the "Robot" class.
    private PinpointOdometryRobot robot = new PinpointOdometryRobot(this);
    private final MecanumDrive mecanumDrive = new MecanumDrive();

    @Override public void runOpMode() {

        // Initialize the robot hardware & Turn on telemetry
        // We are only using this to read the sensors
        robot.initialize(true);

        // Initialize the mecanum drive train
        // We use this for driving the robot using motors
        mecanumDrive.init(hardwareMap);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto

        // Read drive / strafe and heading values and display them
        while (opModeIsActive() & robot.readSensors()) {

            if (gamepad1.a) {
                robot.resetOdometry();
            }

            if (gamepad1.b) {
                robot.turnTo(90, 0.45, 0.25);
            }
            else if (gamepad1.y) {
                robot.drive(6, 0.60, 0.25);
            }
            else if (gamepad1.x) {
                robot.strafe(6, 0.60, 0.25);
            }
            else {
                // drive using the gamepad inputs
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;

                // Set the powers for the mecanum drive train
                mecanumDrive.drive(forward, strafe, rotate);
            }

            telemetry.addData("Status", "Running");
            telemetry.addLine("Press and hold 'A' for a little bit to reset imu");
            telemetry.addLine("Press 'B' to rotate 90 degrees counter clockwise");
            telemetry.addLine("Press 'Y' to go forward 6 inches");
            telemetry.addLine("Press 'X' to strafe left 6 inches");

            telemetry.update();
            sleep(100);
        }
    }
}
