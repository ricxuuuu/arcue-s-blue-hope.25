package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TELEOP driver-c", group = "Linear Op-mode")
public class TELEOP_driver_c extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); // Initialize robot hardware

        //-----------------------------------------------INTAKE/FLY PREP
        boolean fly = false;
        boolean in = false;
        boolean out = false;

        //start up a timer for flicker use
        ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        //-----------------------------------------------INTAKE/FLY PREP

        //get ready to start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // runs until the stop button is pressed ---------------------------------------------------
        while (opModeIsActive()) {
            //-----------------------------------------------DRIVETRAIN
            double axial = -gamepad1.left_stick_y; // forward/back
            double lateral = -gamepad1.left_stick_x; // strafe
            double yaw = gamepad1.right_stick_x; // turn

            //drivetrain joystick movement
            robot.drive(axial, lateral, yaw);
            //-----------------------------------------------DRIVETRAIN

            //-----------------------------------------------INTAKE/FLY
            //player manual control of variables
            if (gamepad1.yWasPressed()) {
                fly = !fly;
            }
            if (gamepad1.aWasPressed()) {
                in = !in;
                out = false;
            }
            if (gamepad1.bWasPressed()) {
                out = !out;
                in = false;
            }

            //robot control based off variables
            if (fly) {
                robot.flywheel.setPower(1);
            } else {
                robot.flywheel.setPower(0);
            }
            if (in) {
                robot.intake.setPower(1);
            } else if (out) {
                robot.intake.setPower(-1);
            } else {
                robot.intake.setPower(0);
            }
            //-----------------------------------------------INTAKE/FLY

            //-----------------------------------------------FLICK SERVO FSM
            switch (robot.flickState) {
                case START:
                    if (gamepad1.xWasPressed()) {
                        flickerTime.reset();
                        robot.flicker.setPosition(1);
                        robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                    }
                    break;
                case UPWARDS:
                    if (flickerTime.seconds() >= 2) {
                        flickerTime.reset();
                        robot.flicker.setPosition(0);
                        robot.flickState = Invokation_of_a_False_Life.flickStates.DOWNWARDS;
                    }
                    break;
                case DOWNWARDS:
                    if (flickerTime.seconds() >= 2) {
                        robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                    }
                    break;
                default:
                    robot.flickState = Invokation_of_a_False_Life.flickStates.START;
            }

            //restart if button is re-pressed
            if (gamepad1.xWasPressed() && robot.flickState != Invokation_of_a_False_Life.flickStates.START) {
                robot.flickState = Invokation_of_a_False_Life.flickStates.START;
                robot.flicker.setPosition(0);
            }
            //-----------------------------------------------FLICK SERVO

            //-----------------------------------------------TELEMETRY
            telemetry.addData("Front Left Power", robot.frontLeft.getPower());
            telemetry.addData("Front Right Power", robot.frontRight.getPower());
            telemetry.addData("Back Left Power", robot.backLeft.getPower());
            telemetry.addData("Back Right Power", robot.backRight.getPower());
            telemetry.addData("fly RPM", robot.getFlywheelRPM());
            telemetry.update();
            //-----------------------------------------------TELEMETRY
        }
    }
}
