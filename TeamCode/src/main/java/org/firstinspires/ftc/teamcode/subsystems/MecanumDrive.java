package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * MecanumDrive - A class for a mecanum drive train
 */
public class MecanumDrive {
    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    /**
     * Initialize the mecanum drive train motors from hardware map
     */
    public void init(HardwareMap hardwareMap) {

        // Get the motors from the hardware map
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Set the motor directions
        // Motors should make a cross
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Set the motor run mode
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set motor powers for mecanum drive train
     * @param frontLeftPower - Power for front left motor
     * @param frontRightPower - Power for front right motor
     * @param backLeftPower - Power for back left motor
     * @param backRightPower - Power for back right motor
     */
    private void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        // Maximum speed of the motor
        double maxSpeed = 1.0;

        // This code normalizes the power values for the motors to ensure that none of them exceed
        // the maximum allowable speed. This is done by finding the maximum power value,
        // and then dividing all the power values by that maximum value.
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        // By dividing each power value by `maxSpeed`, the code ensures that the highest power value
        // is scaled down to 1.0, and all other values are scaled proportionally. This prevents any
        // motor from exceeding its maximum speed.
        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Drive the mecanum drive train
     * @param forward - Forward power
     * @param strafe - Strafe power
     * @param rotate - Rotate power
     */
    public void drive(double forward, double strafe, double rotate) {
        // Calculate the power for each motor
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Set the motor powers
        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

}