package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Outtake {
   // Motor verticalSlide = new Motor(hardwareMap,"verticalSlide",103.6,1780);
    DcMotorEx verticalSlide = hardwareMap.get(DcMotorEx.class, "leftFront");

}
