package org.firstinspires.ftc.teamcode.drive.opmode;// Import the necessary classes
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Declare the op mode name
@TeleOp(name = "defaultTankMode", group = "Linear Opmode")

// Extend LinearOpMode
public class Test extends LinearOpMode {

    // Declare a MecanumDrive object
    MecanumDrive drive;
    DcMotor slideM2, slideM1, arm;
    Servo rotServo;
    Servo grabServo;

    // Override the runOpMode method
    @Override
    public void runOpMode() {

        // Initialize the MecanumDrive object with hardware map
        drive = new SampleMecanumDrive(hardwareMap);
        slideM2 = hardwareMap.dcMotor.get("slideM2");
        slideM1=hardwareMap.dcMotor.get("slideM1");
        arm=hardwareMap.dcMotor.get("grabber180");
        slideM1.setDirection(DcMotorSimple.Direction.REVERSE);
        rotServo = hardwareMap.servo.get("rotServo");
        grabServo = hardwareMap.servo.get("grabServo");
        boolean grabOpen = true;
        // Wait for the start button to be pressed
        waitForStart();

        // Loop until stop button is pressed
        while (opModeIsActive()) {

            // Read the left stick for translation and right stick for rotation
            double x = gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Convert translation to field-centric if desired
            if (true) {
                double heading = drive.getExternalHeading();
                double temp = y * Math.cos(heading) - x * Math.sin(heading);
                x = y * Math.sin(heading) + x * Math.cos(heading);
                y = temp;
            }

            // Create a vector from the translation components
            Vector2d input = new Vector2d(x, y);

            // Normalize the input vector if it exceeds magnitude of 1
            if (input.norm() > 1) {
          //      input = (1, 1);
            }

            // Set the drive velocity and angular velocity
            drive.setDrivePower(new Pose2d(input.getX(), input.getY(), turn));


            // Update telemetry data
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("turn", turn);


//            int slideUpPow = slideUp ? 1 : 0;
//            int slideDownPow = slideDown ? 1 : 0;
//
//            verticalSlide.setPower(slideUpPow-slideDownPow);
//            verticalSlide2.setPower(slideUpPow-slideDownPow);
//
//            int position = verticalSlide.getCurrentPosition();
//            telemetry.addData("Vertical Slide", position);


            if(gamepad1.x){grabOpen=!grabOpen;}//toggle for the grabber
            grabServo.setPosition(grabOpen ? .2 : 0);//sets the grabber to the right position
           telemetry.addData("grabber open? = ",grabOpen);
           telemetry.addData("grabber position",grabServo.getPosition());
            if(gamepad1.y){slidesUp();}
            if(gamepad1.a){slidesDown();}
            if(gamepad1.dpad_left){servoRotate(1);}
            if(gamepad1.dpad_right)servoRotate(0);
            if(gamepad1.right_bumper) runMotorToPosition(arm,0);
            if(gamepad1.left_bumper) runMotorToPosition(arm, -814);
            if(gamepad1.b){
                slideM1.setDirection(DcMotorSimple.Direction.FORWARD);
                slideM2.setDirection(DcMotorSimple.Direction.FORWARD);
                runMotorToPosition(slideM1,-245);
                runMotorToPosition(slideM2, -610);
                slideM1.setDirection(DcMotorSimple.Direction.REVERSE);
                slideM2.setDirection(DcMotorSimple.Direction.REVERSE);
            }
           // if()


            telemetry.addData("vertPos", slideM1.getCurrentPosition());
            telemetry.addData("vertPos2",slideM2.getCurrentPosition());
            telemetry.addData("armPos", arm.getCurrentPosition());
            telemetry.addData("servoPos",rotServo.getPosition());
            telemetry.update();
        }
    }

    private void slidesUp() {
        slideM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("vertPos", slideM1.getCurrentPosition());
        slideM1.setTargetPosition(-260);
        while(slideM1.getCurrentPosition() > slideM1.getTargetPosition()){
            slideM1.setPower(.75);
            slideM2.setPower(.75);
            telemetry.update();
        }
        slideM1.setPower(0);
        slideM2.setPower(0);
    }
    private void slidesDown(){
        slideM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("vertPos", slideM1.getCurrentPosition());
        slideM1.setTargetPosition(0);
        while(slideM1.getCurrentPosition() < slideM1.getTargetPosition()){
            slideM1.setPower(-.5);
            slideM2.setPower(-.5);
            telemetry.update();
        }
        slideM1.setPower(0);
        slideM2.setPower(0);
    }
    private void servoRotate(double position){
        rotServo.setPosition(position);
    }

    public void runMotorToPosition(DcMotor motor, double targetPosition) {
        // PID constants
        final double kp = 0.01; // Proportional gain
        final double ki = 0.00; // Integral gain
        final double kd = 0.00; // Derivative gain

        final double dt = 0.01; // Update interval (seconds)

        double previousError = 0;
        double integral = 0;

        while (opModeIsActive()) {
            // Read the current position of the motor
            double currentPosition = motor.getCurrentPosition();

            // Calculate the error (difference between target and current position)
            double error = targetPosition - currentPosition;

            // Calculate the integral term
            integral += error * dt;

            // Calculate the derivative term
            double derivative = (error - previousError) / dt;

            // Calculate the output of the PID controller
            double output = kp * error + ki * integral + kd * derivative;

            // Apply the output to the motor power
            motor.setPower(output);

            // Check if the motor has reached the target position
            if (Math.abs(error) < 10.0) {
                break; // Exit the loop if close enough to the target position
            }

            // Store the current error for the next iteration
            previousError = error;

            // Delay to control the update rate of the PID controller
            sleep((long) (dt * 1000));
        }

        // Stop the motor after reaching the target position
        motor.setPower(0);
    }
    private void slidesOut(){

    }
    private void slidesIn(){

    }

}
