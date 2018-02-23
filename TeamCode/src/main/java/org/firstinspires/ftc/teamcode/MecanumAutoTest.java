package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by yuying75618 on 18/2/22.
 */
@Autonomous(name = "MecAutoTest",group = "Concept")

public class MecanumAutoTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Mecanum mecanum = new Mecanum();

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        mecanum.MotorL1 = hardwareMap.get(DcMotor.class,"motorLF");
        mecanum.MotorL2 = hardwareMap.get(DcMotor.class,"motorLB");
        mecanum.MotorR1 = hardwareMap.get(DcMotor.class,"motorRF");
        mecanum.MotorR2 = hardwareMap.get(DcMotor.class,"motorRB");
        telemetry.addData("MotorDeclare", "Complete");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            mecanum.AutoMove(0.5,0);
            sleep(1000);
            mecanum.AutoMove(0.5,180);
            sleep(1000);
            mecanum.AutoMove(0.5,90);
            sleep(1000);
            mecanum.AutoMove(0.5,270);
            sleep(1000);
            mecanum.AutoMove(0.5,45);
            sleep(1000);
            mecanum.AutoMove(0.5,315);
            sleep(1000);
            mecanum.AutoMove(0.5,225);
            sleep(2000);
            mecanum.AutoMove(0.5,135);
            sleep(1000);
        }
    }

}
