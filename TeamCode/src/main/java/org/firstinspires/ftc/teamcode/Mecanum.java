package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import javax.lang.model.type.NullType;


@TeleOp


public class Mecanum{
    protected double L1 = 0;//左前
    protected double L2 = 0;//左后
    protected double R1 = 0;//右前
    protected double R2 = 0;//右后

    protected DcMotor MotorL1,MotorL2,MotorR1,MotorR2;

    private void setMacanum(){
        MotorL1.setPower(L1);
        MotorL2.setPower(L2);
        MotorR1.setPower(R1);
        MotorR2.setPower(R2);
    }



    //平移
    public void Panning(double v,double a){//v-速度，a-角度
        double v_max = 1;
        double aa = a / 180 * Math.PI;
        double vx = v * Math.cos(aa);
        double vy = v * Math.sin(aa);

        L1 = vy - vx;
        L2 = vy + vx;
        R1 = vy + vx;
        R2 = vy - vx;

        double p = 1 / Math.max(L1,L2);
        L1 *= p;
        L2 *= p;
        R1 *= -p;
        R2 *= -p;
        setMacanum();
        return;
    }

    //手柄控制
    public void Stick(double vx,double vy){
        L1 = vy - vx;
        L2 = vy + vx;
        R1 = vy + vx;
        R2 = vy - vx;

        double p = 1 / Math.max(L1,L2);
        L1 *= p;
        L2 *= p;
        R1 *= -p;
        R2 *= -p;
        setMacanum();
        return;
    }
    //原地旋转
    public void Circle(double w){//w-角速度
        double r = 1;

        L1 = w * r;
        L2 = w * r;
        R1 = w * r;
        R2 = w * r;
        setMacanum();
        return;
    }

    public void Stop(){
        L1 = 0;
        L2 = 0;
        R1 = 0;
        R2 = 0;
        setMacanum();
        return;
    }


}
