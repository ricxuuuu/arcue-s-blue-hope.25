package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;


@TeleOp(name = "TELEOP driver-c", group = "Linear Op-mode")
public class TELEOP_driver_c extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); // Initialize robot hardware
        boolean is_blue_alliance = false; //to know which alliance you are on

        //-----------------------------------------------INTAKE/FLY PREP
        boolean fly = false;
        boolean in = false;
        boolean out = false;
        boolean inta_stalled = false;
        boolean follower_control = false;

        //start up a timer for flicker use
        ElapsedTime flickerTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        //-----------------------------------------------INTAKE/FLY PREP

        //get ready to start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // runs until the stop button is pressed ---------------------------------------------------
        while (opModeIsActive()) {
            robot.follower.updatePose();
            follower_control = gamepad1.left_trigger > 0.13 ^ gamepad1.right_trigger > 0.13;

            if (!follower_control) {


                //-----------------------------------------------ALLIANCE PICK
                if (gamepad1.leftBumperWasPressed()) {
                    is_blue_alliance = true;
                }
                if (gamepad1.rightBumperWasPressed()) {
                    is_blue_alliance = false;
                }
                //-----------------------------------------------ALLIANCE PICK

                //-----------------------------------------------DRIVETRAIN
                double axial = -gamepad1.left_stick_y; // forward/back
                double lateral = gamepad1.left_stick_x; // strafe
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
                    robot.flywheel2.setPower(1);
                } else {
                    robot.flywheel.setPower(0);
                    robot.flywheel2.setPower(0);
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
                            robot.flicker.setPosition(1); //go up
                            robot.flickState = Invokation_of_a_False_Life.flickStates.UPWARDS;
                        }
                        break;
                    case UPWARDS:
                        if (flickerTime.seconds() >= 0.2) {
                            flickerTime.reset();
                            robot.flicker.setPosition(0); //go down
                            robot.flickState = Invokation_of_a_False_Life.flickStates.DOWNWARDS;
                        }
                        break;
                    case DOWNWARDS:
                        if (flickerTime.seconds() >= 0.2) {
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
                //-----------------------------------------------FLICK SERVO FSM

                //-----------------------------------------------HOOD SERVO AUTO ADJ
                //use the function findIdealLaunchAngle to change servo position,
                //once testing has been done to see what works (will need to graph)
                //-----------------------------------------------HOOD SERVO AUTO ADJ


            } else {


                //-----------------------------------------------BOT HOLD ADJ GOAL
                if (gamepad1.left_trigger > 0.13) {
                    follower.turnTo(robot.findIdealGoalAngle(true));
                }
                if (gamepad1.right_trigger > 0.13) {
                    follower.turnTo(robot.findIdealGoalAngle(false));
                }
                follower.update();
                //-----------------------------------------------BOT HOLD ADJ GOAL


            }

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
