package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TELEOP driver-c", group = "Linear Op-mode")
public class TELEOP_driver_c extends LinearOpMode {

    private final Invokation_of_a_False_Life robot = new Invokation_of_a_False_Life();

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        boolean fly = false;
        boolean preFly = false;

        waitForStart();


        // Run until the stop button is pressed
        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y; // forward/back
            double lateral = -gamepad1.left_stick_x; // strafe
            double yaw = gamepad1.right_stick_x; // turn

            robot.drive(axial, lateral, yaw);

            boolean flyButton = gamepad1.x;
            if (flyButton && !preFly) {
                fly = !fly;
            }
            preFly = flyButton;

            if (fly) {
                robot.flywheel.setPower(1);
            } else {
                robot.flywheel.setPower(0);
            }

            telemetry.addData("Front Left Power", robot.frontLeft.getPower());
            telemetry.addData("Front Right Power", robot.frontRight.getPower());
            telemetry.addData("Back Left Power", robot.backLeft.getPower());
            telemetry.addData("Back Right Power", robot.backRight.getPower());
            telemetry.addData("fly RPM", robot.getFlywheelRPM());
            telemetry.update();
        }
    }
}