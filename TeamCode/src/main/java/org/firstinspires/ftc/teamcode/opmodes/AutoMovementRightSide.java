package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Chassis.blueRight;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

@Autonomous
public class AutoMovementRightSide extends LinearOpMode {
    Chassis bobot;
    TwoPointServo claw;
    TwoPointServo clawWrist;
    Slide slides;
    Motor slideMotor;
    PIDFController armController;

    // PID coefficients
    public static double p = -0.002, i = 0, d = 0.0001, f = 0.01;
    private final double ticks_in_degree = Math.round(1993.6 / 360);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize chassis, claw, wrist, slides, and arm motor
        bobot = new Chassis(hardwareMap, blueRight);
        claw = new TwoPointServo(0.35, 0.0, "claw", hardwareMap);
        clawWrist = new TwoPointServo(0.4, 0.0, "clawWrist", hardwareMap);
        slides = new Slide(hardwareMap);
        slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));

        // Reverse the slide motor and set it to run without an encoder
        slideMotor.reverse();
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Set initial positions for the claw and wrist
        claw.positionB(); // Close claw
        clawWrist.positionA(); // Default wrist position

        // Initialize PID controller for the arm motor
        FeedforwardFun armFF = (position, velocity) -> Math.cos(Math.toRadians(position) / ticks_in_degree) * f;
        PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p, i, d);
        armController = new PIDFController(coefficients, armFF);

        // Wait for the start of the match
        waitForStart();

        // Sample positions (example values)
        Pose2d sample1Position = new Pose2d(-120, 30, 0);
        Pose2d sample2Position = new Pose2d(-145, 30, 0);
        Pose2d sample3Position = new Pose2d(-165, 30, 0);

        // Specimen and clipping positions
        Pose2d specimenPosition = new Pose2d(-120, 165, 0);
        Pose2d clipStartPosition = new Pose2d(-20, 80, Math.toRadians(90));
        Pose2d clipPushPosition = new Pose2d(-30, 85, Math.toRadians(90)); // Move slightly toward the wall for clipping

        if (opModeIsActive()) {
            // Push the three samples into the zone
            bobot.moveToPoint(sample1Position);
            sleep(500);
            bobot.moveToPoint(sample2Position);
            sleep(500);
            bobot.moveToPoint(sample3Position);
            sleep(500);

            // Perform grab and place sequence twice
            for (int i = 0; i < 2; i++) {
                // Grab the specimen
                bobot.moveToPoint(specimenPosition);
                grabSpecimen();

                // Move to initial clipping position
                bobot.moveToPoint(clipStartPosition);

                // Clip the specimen securely
                placeSpecimen();

                // Slightly adjust position for subsequent placements
                clipStartPosition = new Pose2d(
                        clipStartPosition.position.x + 5, // Add 5 to the x-coordinate
                        clipStartPosition.position.y,     // Keep the y-coordinate the same
                        clipStartPosition.heading.toDouble() // Keep the heading unchanged
                );
            }
        }
    }

    private void grabSpecimen() {
        telemetry.addData("Action", "Grabbing Specimen");
        telemetry.update();

        // Rotate arm to target position (400) using PID
        rotateArmToTarget(400);

        // Open claw to grab specimen
        claw.positionA();
        sleep(500);

        // Close claw to secure specimen
        claw.positionB();
        sleep(500);
    }

    private void placeSpecimen() {
        telemetry.addData("Action", "Placing Specimen");
        telemetry.update();

        // Rotate arm to target position (875) for clipping
        rotateArmToTarget(875);

        // Rotate wrist for proper angle
        clawWrist.positionB();
        sleep(500);

        // Move slightly toward the wall to clip specimen securely
        bobot.moveToPoint(new Pose2d(bobot.getPose().position.y - 5, bobot.getPose().position.y, bobot.getPose().heading.toDouble()));

        // Open claw to release specimen
        claw.positionA();
        sleep(500);

        // Move back to ensure the clip is secure
        bobot.moveToPoint(new Pose2d(bobot.getPose().position.x + 5, bobot.getPose().position.y, bobot.getPose().heading.toDouble()));

        // Close claw after releasing
        claw.positionB();

        // Reset wrist to default position
        clawWrist.positionA();
        sleep(500);
    }

    private void rotateArmToTarget(int target) {
        armController.setTargetPosition(target);
        int armPos = slideMotor.getPosition();

        while (Math.abs(target - armPos) > 10 && opModeIsActive()) { // Allow a tolerance of 10
            double power = armController.update(System.nanoTime(), armPos, slideMotor.getVelocity());
            slideMotor.setPower(power);
            armPos = slideMotor.getPosition();

            telemetry.addData("Target", target);
            telemetry.addData("Current Position", armPos);
            telemetry.addData("Error", target - armPos);
            telemetry.update();
        }

        slideMotor.setPower(0); // Stop the motor once the target is reached
    }
}