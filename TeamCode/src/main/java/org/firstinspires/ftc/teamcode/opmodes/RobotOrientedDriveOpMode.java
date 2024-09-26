package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * SimpleRobotOrientedDriveOpMode - A simple teleop opmode for a mecanum drive train
 * OpMode for a mecanum drive traiith robot oriented drive
 */
@TeleOp(name="Robot Oriented Drive OpMode", group="Tests")
public class RobotOrientedDriveOpMode extends LinearOpMode {
    // Initialize the mecanum drive train
    private MecanumDrive mecanumDrive = new MecanumDrive();
    @Override
    public void runOpMode() {

        mecanumDrive.init(hardwareMap);
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

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);

            // Set the powers for the mecanum drive train
            mecanumDrive.drive(forward, strafe, rotate);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}