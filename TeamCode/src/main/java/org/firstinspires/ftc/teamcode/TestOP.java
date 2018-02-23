/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "BasicMove", group = "Concept")
//@Disabled
public class TestOP extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime LF,LB,RF,RB;
    private DcMotor MotorLF, MotorLB, MotorRF, MotorRB;
    double speedLF,speedLB,speedRF,speedRB;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("AAAA", "BBBBB");

        //底盘四电机
        MotorLF = hardwareMap.get(DcMotor.class, "motorLF");//左前
        MotorLB = hardwareMap.get(DcMotor.class, "motorLB");//左后
        MotorRF = hardwareMap.get(DcMotor.class, "motorRF");//右前
        MotorRB = hardwareMap.get(DcMotor.class, "motorRB");//右后
        telemetry.addData("MotorDeclare", "Complete");
        MotorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRB.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDClass.resetEncode(MotorLB);
        PIDClass.resetEncode(MotorLF);
        PIDClass.resetEncode(MotorRB);
        PIDClass.resetEncode(MotorRF);
    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
        //MotorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        MotorLF.setPower(1);
        MotorLF.setTargetPosition(1000);
        MotorRF.setPower(1);
        MotorRF.setTargetPosition(1000);
        MotorLB.setPower(1);
        MotorLB.setTargetPosition(1000);
        MotorRB.setPower(1);
        MotorRB.setTargetPosition(1000);
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        speedLF = MotorLF.getCurrentPosition()/runtime.time();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("\nLFsp","%f",speedLF);
        telemetry.addData("LFCP","%d",MotorLF.getCurrentPosition());
        telemetry.addData("LFTP","%d",MotorLF.getTargetPosition());

        telemetry.addData("\nLBsp","%f",speedLB);
        telemetry.addData("LBCP","%d",MotorLB.getCurrentPosition());
        telemetry.addData("LBTP","%d",MotorLB.getTargetPosition());

        telemetry.addData("\nRFsp","%f",speedRF);
        telemetry.addData("RFCP","%d",MotorRF.getCurrentPosition());
        telemetry.addData("RFTP","%d",MotorLF.getTargetPosition());

        telemetry.addData("\nRBsp","%f",speedRB);
        telemetry.addData("RBCP","%d",MotorRB.getCurrentPosition());
        telemetry.addData("RBTP","%d",MotorLF.getTargetPosition());
        telemetry.update();
        //if(this.gamepad1.x){

            //MotorLF.setPower(1);
            //if (!MotorLF.isBusy()||MotorLF.getCurrentPosition()==1000)
        speedLF = MotorLF.getCurrentPosition()/runtime.time();
        speedLB = MotorLB.getCurrentPosition()/runtime.time();
        speedRF = MotorRF.getCurrentPosition()/runtime.time();
        speedRB = MotorRB.getCurrentPosition()/runtime.time();


        //}
        //else
        /*if(this.gamepad1.a)
            MotorLB.setPower(1);
        if(this.gamepad1.y)
            MotorRF.setPower(1);
        if(this.gamepad1.b)
            MotorRB.setPower(1);*/


    }
}
