package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class AutoDriveCommand {
    Pose2D targetPose;
    double speed;

    public AutoDriveCommand(Pose2D pose, double speed) {
        this.targetPose = pose;
        this.speed = speed;
    }

    public boolean isFinished(Pose2D pos) {
        // If new position is same as old position, then we are done
        // These will never be exactly the same, so we need to check if they are close enough

        boolean xReached = Math.abs(targetPose.getX(DistanceUnit.INCH) - pos.getX(DistanceUnit.INCH)) <= 0.1 * Math.abs(targetPose.getX(DistanceUnit.INCH));
        boolean yReached = Math.abs(targetPose.getY(DistanceUnit.INCH) - pos.getY(DistanceUnit.INCH)) <= 0.1 * Math.abs(targetPose.getY(DistanceUnit.INCH));
        boolean headingReached = Math.abs(targetPose.getHeading(AngleUnit.DEGREES) - pos.getHeading(AngleUnit.DEGREES)) <= 0.1 * Math.abs(targetPose.getHeading(AngleUnit.DEGREES));

        return xReached && yReached && headingReached;
    }

    public void execute(Telemetry telemetry, MecanumDrive mecanumDrive, Pose2D pos) {
        double rotate = targetPose.getHeading(AngleUnit.DEGREES) - pos.getHeading(AngleUnit.DEGREES);
        double forward = targetPose.getX(DistanceUnit.INCH) - pos.getX(DistanceUnit.INCH);
        double strafe = targetPose.getY(DistanceUnit.INCH) - pos.getY(DistanceUnit.INCH);

        double forwardPower, strafePower, rotatePower;
        if (Math.abs(forward) > 0.1) {
            forwardPower = Math.signum(forward) * speed;
        } else {
            forwardPower = 0;
        }

        if (Math.abs(strafe) > 0.1) {
            strafePower = Math.signum(strafe) * speed;
        } else {
            strafePower = 0;
        }

        telemetry.addData("Forward", forwardPower);
        telemetry.addData("Strafe", strafePower);
        telemetry.addData("Rotate", rotate);

        // Set the powers for the mecanum drive train
        mecanumDrive.drive(forwardPower, strafePower, 0.0);
    }
}
