using System.Collections;
using System.Collections.Generic;
using System.IO;
using System;
using UnityEngine;
using UnityEditor;

public class Drone_Fail : MonoBehaviour
{
    //  ┌--------------------┐
    //  │   Basic Paramter   │
    //  └--------------------┘
    public int Error_Flag = 0;
    public double dt = 0.01;
    public double Sa_Bias = 5;
    public double Sa_Noise = 0.3;
    public double Sg_Bias = 0 / 3600;
    public double Sg_Noise = 1 / 3600;
    public double Sm_Noise = 0.0005;
    public double init_Pos_N = 0;
    public double init_Pos_E = 0;
    public double init_Pos_D = 0;
    public double init_Att_Roll = 0;
    public double init_Att_Pitch = 0;
    public double init_Att_Yaw = 0;
    string S_MX,S_MY,S_MZ,Str_Mag;
    double init_Mag_Pos_N = -12.2;
    double init_Mag_Pos_E = 3;
    double init_Mag_Pos_D = -2.5;
    int Counter;
    double g = 9.81;
    double mg = 0.00981;
    double D2R = Math.PI / 180;
    double R2D = 180 / Math.PI;
    double R0 = 6369121.969018592;
    double[] VecT;
    double[,] Gvec = new double[3, 1];
    double[,] sigma = new double[3, 1];
    double[,] Vec3_Temp = new double[3, 1];
    double[,] Mat3_Temp = new double[3, 3];
    double sigma_norm, a_c, a_s;

    //  ┌--------------------┐
    //  │     Sensor Data    │
    //  └--------------------┘
    public double[,] S_Acc = new double[3, 1];
    public double[,] S_Gyro = new double[3, 1];
    public double[,] S_Mag = new double[3,1];
    double[,] Sg_Bias_Vec = new double[3, 1];
    double[,] Sa_Bias_Vec = new double[3, 1];
    double[,] Sg_Noise_Vec = new double[3, 1];
    double[,] Sa_Noise_Vec = new double[3, 1];
    public double[,] Sm_Noise_Vec = new double[3,1];


    //  ┌--------------------┐
    //  │  Filter Variables  │
    //  └--------------------┘
    public double[,] F_Euler = new double[3, 1];
    public double[,] F_Avel = new double[3, 1];
    //  ┌--------------------┐
    //  │       Drone        │
    //  └--------------------┘
    double[,] init_Pos = new double[3, 1];

    double[,] Pre_B_dW = new double[3, 1];
    double[,] B_dW = new double[3, 1];
    double[,] B_W = new double[3, 1];
    double[,] Pre_N_dW = new double[3, 1];
    double[,] N_dW = new double[3, 1];
    double[,] N_W = new double[3, 1];
    double[,] T_Acc = new double[3, 1];
    double[,] T_bAcc = new double[3, 1];
    public double[,] T_Euler = new double[3, 1];
    double[,] T_Cbn = new double[3, 3];
    double[,] T_Quat = new double[4, 1];
    double[,] T_dQuat = new double[4, 1];
    double[,] T_Quat_Update = new double[4, 1];
    double[,] Pre_T_Acc = new double[3, 1];
    double[,] Pre_T_Vel = new double[3, 1];
    public double[,] T_Vel = new double[3, 1];
    double[,] Pre_T_Pos = new double[3, 1];
    public double[,] T_Pos = new double[3, 1];
    double Thrust, m, L, Ixx, Iyy, Izz, Irr;
    double[,] W = new double[4, 1];
    public double[,] D_W = new double[4, 1];
    public double df = 0.98;
    //  ┌--------------------┐
    //  │     CP Filter      │
    //  └--------------------┘
    public double Ca = 0.998;
    double Ac_Roll, Ac_Pitch, Gy_dRoll, Gy_dPitch, Gy_dYaw, Gy_Roll, Gy_Pitch, Gy_Yaw, X_h, Y_h, Mag_Yaw;
    double LPF_Roll, LPF_Pitch, LPF_Yaw, HPF_Roll, HPF_Pitch, HPF_Yaw ;
    
    //  ┌--------------------┐
    //  │      Control       │
    //  └--------------------┘
    double[,] C_Att = new double[3, 1];
    double[,] Target_Att = new double[3, 1];

    double[,] CMD_Pos = new double[3, 1];
    double[,] CMD_Att = new double[3, 1];

    double[,] E_Att  = new double[3, 1];
    double[,] E_Avel = new double[3, 1];

    public double A_Kp = 0.0001;
    public double A_Kd = 0.0001;
    public double alpha = 1;
    public double Max_Att = 20;
    public double Cut_Pos = 20;
    public double Max_Vel = 5;
    public double Max_W = 100;

    
    //  ┌--------------------┐
    //  │   Visualization    │
    //  └--------------------┘
    Vector3 V_Att = new Vector3();
    Vector3 V_rAtt = new Vector3();
    Vector3 V_Pos = new Vector3();

    // Start is called before the first frame update
    void Start()
    {
        Application.targetFrameRate = (int)(1 / dt);
        Counter = 0;

        init_State();
        init_Dynamics();

        Vector3 pos;

        pos =this.gameObject.transform.position;

        Debug.Log(pos);
        Debug.Log(pos.y);
    }

    // Update is called once per frame
    void Update()
    {
        Drone_Dynamics();
        Make_Sdata();


       
           
        
            
        
      


        //CP_Filter();
        CP_Filter2();
        //T_Filter();
       
        RC_Input_A();
        Drone_Att_Control();


        Visualization();


    }


    //  ┌--------------------┐
    //  │     CP Filter      │
    //  └--------------------┘
    public void CP_Filter()
    {
        
        double Att_noise = 0.1*Math.PI/180;
        System.Random rn = new System.Random();
        double Rn_Vec ,Pn_Vec;

        Rn_Vec = (double)rn.Next(-100, 100) / 100 * Att_noise;
        Pn_Vec = (double)rn.Next(-100, 100) / 100 * Att_noise;
        
        Gy_dYaw   = (S_Gyro[1 , 0] * Math.Sin(F_Euler[0, 0]) + S_Gyro[2 , 0] * Math.Cos(F_Euler[0, 0])) * (1/Math.Cos(F_Euler[1, 0]));

        X_h = S_Mag[0, 0] * Math.Cos(F_Euler[1, 0]) + S_Mag[1, 0] * Math.Sin(F_Euler[0, 0]) * Math.Sin(F_Euler[1, 0]) + S_Mag[2, 0] * Math.Cos(F_Euler[0, 0]) * Math.Sin(F_Euler[1, 0]);
        Y_h = S_Mag[1, 0] * Math.Cos(F_Euler[0, 0]) - S_Mag[2, 0] * Math.Sin(F_Euler[0, 0]);//x_h = mag는 지자기 
        Mag_Yaw = Math.Atan2(-Y_h,X_h);

        LPF_Yaw = (Ca * LPF_Yaw) + (1 - Ca) * Mag_Yaw;
        HPF_Yaw = Ca * HPF_Yaw   + Ca * Gy_dYaw * dt;
        

        //Debug.Log("LPF Att : " + (LPF_Roll * R2D).ToString() + "\t" + (LPF_Pitch * R2D).ToString() + "\t" + (F_Euler[2, 0] * R2D).ToString());

        
        F_Euler[0, 0] = T_Euler[0, 0] + Rn_Vec;//필터링된 롤
        F_Euler[1, 0] = T_Euler[1, 0] + Pn_Vec;
        F_Euler[2, 0] = (LPF_Yaw   + HPF_Yaw);
        //Debug.Log("F Att : " + (F_Euler[0, 0] * R2D).ToString() + "\t" + (F_Euler[1, 0] * R2D).ToString() + "\t" + (F_Euler[2, 0] * R2D).ToString());
        Debug.Log("F Att : " + ((T_Euler[0, 0] - F_Euler[0, 0] )* R2D).ToString() + "\t" + ((T_Euler[1, 0] - F_Euler[1, 0] )* R2D).ToString() + "\t" + ((T_Euler[2, 0] - F_Euler[2, 0] )* R2D).ToString());
       
        
    }
        public void CP_Filter2()
    {
        
        Ac_Roll   = Math.Atan2(S_Acc[1, 0], -S_Acc[2, 0]);
        Ac_Pitch  = Math.Atan2(S_Acc[0, 0], Math.Sqrt(S_Acc[1, 0] * S_Acc[1, 0] + S_Acc[2, 0] * S_Acc[2, 0]));

        Gy_dRoll  = (S_Gyro[1 , 0] * Math.Sin(F_Euler[0, 0]) + S_Gyro[2 , 0] * Math.Cos(F_Euler[0, 0])) * Math.Tan(F_Euler[1, 0]) + S_Gyro[0 , 0];
        Gy_dPitch = S_Gyro[1 , 0] * Math.Cos(F_Euler[0, 0]) - S_Gyro[2 , 0] * Math.Sin(F_Euler[0, 0]);
        Gy_dYaw   = (S_Gyro[1 , 0] * Math.Sin(F_Euler[0, 0]) + S_Gyro[2 , 0] * Math.Cos(F_Euler[0, 0])) * (1/Math.Cos(F_Euler[1, 0]));

        LPF_Roll  = Ca * LPF_Roll  + (1 - Ca) * Ac_Roll;
        LPF_Pitch = Ca * LPF_Pitch + (1 - Ca) * Ac_Pitch;

        HPF_Roll  = Ca * HPF_Roll  + Ca * Gy_dRoll  * dt;
        HPF_Pitch = Ca * HPF_Pitch + Ca * Gy_dPitch * dt;
        //Debug.Log("HPF Att : " + (HPF_Roll * R2D).ToString() + "\t" + (HPF_Pitch * R2D).ToString() + "\t" + (F_Euler[2, 0] * R2D).ToString());


        X_h = S_Mag[0, 0] * Math.Cos(F_Euler[1, 0]) + S_Mag[1, 0] * Math.Sin(F_Euler[0, 0]) * Math.Sin(F_Euler[1, 0]) + S_Mag[2, 0] * Math.Cos(F_Euler[0, 0]) * Math.Sin(F_Euler[1, 0]);
        Y_h = S_Mag[1, 0] * Math.Cos(F_Euler[0, 0]) - S_Mag[2, 0] * Math.Sin(F_Euler[0, 0]);
        Mag_Yaw = Math.Atan2(-Y_h,X_h);

        LPF_Yaw = (Ca * LPF_Yaw) + (1 - Ca) * Mag_Yaw;
        HPF_Yaw = Ca * HPF_Yaw   + Ca * Gy_dYaw * dt;
        

        //Debug.Log("LPF Att : " + (LPF_Roll * R2D).ToString() + "\t" + (LPF_Pitch * R2D).ToString() + "\t" + (F_Euler[2, 0] * R2D).ToString());

        
        F_Euler[0, 0] = (LPF_Roll  + HPF_Roll);
        F_Euler[1, 0] = (LPF_Pitch + HPF_Pitch);
        F_Euler[2, 0] = (LPF_Yaw   + HPF_Yaw);
        //Debug.Log("F Att : " + (F_Euler[0, 0] * R2D).ToString() + "\t" + (F_Euler[1, 0] * R2D).ToString() + "\t" + (F_Euler[2, 0] * R2D).ToString());
        Debug.Log("F Att : " + ((T_Euler[0, 0] - F_Euler[0, 0] )* R2D).ToString() + "\t" + ((T_Euler[1, 0] - F_Euler[1, 0] )* R2D).ToString() + "\t" + ((T_Euler[2, 0] - F_Euler[2, 0] )* R2D).ToString());

    }
    public void init_State()
    {
        F_Euler = Make_Vec(VecT = new double[3] { init_Att_Roll, init_Att_Pitch, init_Att_Yaw });
    }
    
    //  ┌--------------------┐
    //  │   Drone Dynamics   │
    //  └--------------------┘
    public void init_Dynamics()
    {
        E_Att = Make_Vec(VecT = new double[3] { 0, 0, 0 });
        inte_E_Att = Make_Vec(VecT = new double[3] { 0, 0, 0 });

        init_Pos = Make_Vec(VecT = new double[3] { init_Pos_N, init_Pos_E, init_Pos_D });

        T_Pos = init_Pos;
        T_Vel = Make_Vec(VecT = new double[3] { 0, 0, 0 });
        T_Euler = Make_Vec(VecT = new double[3] { init_Att_Roll, init_Att_Pitch, init_Att_Yaw });
        T_Cbn = Cal_Euler_2_Cbn(T_Euler);
        T_Quat = Cal_Cbn_2_Quat(T_Cbn);

        Ixx = 0.006;
        Iyy = 0.006;
        Izz = 0.011; //kg/m^2
        Irr = 0.026; // kg/m^2/%
        L = 0.2;
        m = 0.727;

        double C_Thrust = m * g / Irr / (Math.Cos(T_Euler[0, 0]) * Math.Cos(T_Euler[1, 0]));
        W[0, 0] = C_Thrust / 4;
        W[1, 0] = C_Thrust / 4;
        W[2, 0] = C_Thrust / 4;
        W[3, 0] = C_Thrust / 4;

        System.Random rn = new System.Random();
        double[,] tmp_Vec = new double[3, 1];
        tmp_Vec = Make_Vec(VecT = new double[3] { (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100 });
        Sa_Bias_Vec = Mat_Scalar(Sa_Bias * mg, tmp_Vec);
        tmp_Vec = Make_Vec(VecT = new double[3] { (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100 });
        Sg_Bias_Vec = Mat_Scalar(Sg_Bias * D2R, tmp_Vec);
    }
    public void Drone_Dynamics()
    {
        double[,] F_Vec;
        Pre_B_dW = B_dW;

        Gvec[2, 0] = 9.81;

        //D_W = Mat_Cal(Mat_Scalar(alpha, D_W), '+', Mat_Scalar(1 - alpha, W));
        D_W = W;
        // Attitude
        Thrust = Irr / m * (D_W[0, 0] + D_W[1, 0] + D_W[2, 0] + D_W[3, 0]);
        B_dW[0, 0] = L * Irr / Ixx * (D_W[0, 0] - D_W[1, 0] - D_W[2, 0] + D_W[3, 0]);
        B_dW[1, 0] = L * Irr / Iyy * (D_W[0, 0] + D_W[1, 0] - D_W[2, 0] - D_W[3, 0]);
        B_dW[2, 0] = L * Irr / Izz * (D_W[0, 0] - D_W[1, 0] + D_W[2, 0] - D_W[3, 0]);
        B_W = Vec_integate(B_W, Pre_B_dW, B_dW, dt);
        S_Gyro = B_W;
        sigma = Mat_Scalar(dt, B_W);
        sigma_norm = Vec_Norm(sigma);
        a_c = Math.Cos(sigma_norm / 2);
        if (sigma_norm == 0)
        {
            a_s = 0.5;
        }
        else
        {
            a_s = Math.Sin(sigma_norm / 2) / sigma_norm;
        }
        T_Quat_Update = Make_Vec(VecT = new double[4] { a_c, a_s * sigma[0, 0], a_s * sigma[1, 0], a_s * sigma[2, 0] });
        T_Quat = Cal_Quat_Multi(T_Quat, T_Quat_Update);
        T_Quat = Mat_Scalar(1 / Vec_Norm(T_Quat), T_Quat);
        T_Cbn = Cal_Quat_2_Cbn(T_Quat);
        T_Euler = Cal_Cbn_2_Euler(T_Cbn);

        Pre_T_Acc = T_Acc;
        Pre_T_Vel = T_Vel;
        // Position
        F_Vec = Make_Vec(VecT = new double[3]{0, 0, -Thrust});
        T_Acc = Mat_Multi(T_Cbn,F_Vec);
        T_Acc = Mat_Cal(T_Acc,'+',Gvec);
        T_Acc = Mat_Cal(T_Acc, '-',Mat_Scalar(df, T_Vel));
        S_Acc = Mat_Multi(Mat_Inv(T_Cbn), Mat_Cal(T_Acc, '-', Gvec));
        T_Vel = Vec_integate(T_Vel, Pre_T_Acc, T_Acc, dt);
        T_Pos = Vec_integate(T_Pos, Pre_T_Vel, T_Vel, dt);
    }
    
    double[,] Pre_E_Att = new double[3,1];
    double[,] inte_E_Att = new double[3,1];
    double A_Ki = 0;
    public void Drone_Att_Control()
    {
        double C_Thrust;
        double E_Pos,E_Vel,C_Pos;
       
        Target_Att = Mat_Scalar(D2R, CMD_Att);
        double L_Kp, L_Kd;
        L_Kp = 3;
        L_Kd = 3;

        //Target_Att[2, 0] = 0;

        Pre_E_Att = E_Att;
        E_Att = Mat_Cal(Target_Att, '-', F_Euler);
        inte_E_Att = Mat_Cal(inte_E_Att, '+', Mat_Scalar(dt, E_Att));

        for (int i = 0; i < 3; i++)
        {
            C_Att[i, 0] = A_Kp * E_Att[i, 0] + A_Ki * inte_E_Att[i, 0] + A_Kd * (E_Att[i, 0] - Pre_E_Att[i, 0]) / dt;
        }


        /*
        E_Att = Mat_Scalar(A_Kp,Mat_Cal(Target_Att, '-', F_Euler));
        E_Avel = Mat_Scalar(A_Kd,Mat_Cal(E_Att, '-', F_Avel));
        C_Att = E_Avel;
        */
        E_Pos = T_Pos[2,0] - CMD_Pos[2, 0];
        E_Vel = L_Kp*E_Pos - T_Vel[2, 0];
        C_Pos = L_Kd*E_Vel;
        
        C_Thrust = m * g / Irr / (Math.Cos(F_Euler[0, 0]) * Math.Cos(F_Euler[1, 0])) + C_Pos;

        W[0, 0] = C_Thrust / 4 + C_Att[0, 0] / 4 + C_Att[1, 0] / 4 + C_Att[2, 0] / 4;
        W[1, 0] = C_Thrust / 4 - C_Att[0, 0] / 4 + C_Att[1, 0] / 4 - C_Att[2, 0] / 4;
        W[2, 0] = C_Thrust / 4 - C_Att[0, 0] / 4 - C_Att[1, 0] / 4 + C_Att[2, 0] / 4;//오일러 각가속도 W각속도
        W[3, 0] = C_Thrust / 4 + C_Att[0, 0] / 4 - C_Att[1, 0] / 4 - C_Att[2, 0] / 4;
        for(int i=0; i<4; i++)
        {
            if (W[i, 0] > Max_W)
                W[i, 0] = Max_W;
            else if (W[i, 0] <= 0)
                W[i, 0] = 0;
        }



    }
    public void T_Filter()
    {
        F_Euler = T_Euler;
        F_Avel = B_W;
    }
    public void Make_Sdata()
    {
        System.Random rn = new System.Random();
        double[,] tmp_Vec = new double[3, 1];
        tmp_Vec = Make_Vec(VecT = new double[3] { (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100 });
        Sa_Noise_Vec = Mat_Scalar(Sa_Noise, tmp_Vec);
        tmp_Vec = Make_Vec(VecT = new double[3] { (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100 });
        Sg_Noise_Vec = Mat_Scalar(Sg_Noise, tmp_Vec);
        tmp_Vec = Make_Vec(VecT = new double[3] { (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100, (double)rn.Next(-100, 100) / 100 });
        Sm_Noise_Vec = Mat_Scalar(Sm_Noise, tmp_Vec);
        //Bias
        //S_Gyro = Mat_Cal(B_W, '+', Sg_Bias_Vec);
        //S_Acc = Mat_Cal(T_bAcc, '+', Sa_Bias_Vec);
        //Noise
        S_Gyro = Mat_Cal(S_Gyro, '+', Sg_Noise_Vec);
        S_Acc = Mat_Cal(S_Acc, '+', Sa_Noise_Vec);
        S_Mag = Mat_Cal(Cal_S_Mag(), '+', Sm_Noise_Vec);
    }
    public double[,] Mag_Dipole_Moment()
    {
        double u0 = 90;
        double[,] Mag_Pos = Make_Vec(VecT = new double[3] {init_Mag_Pos_N,init_Mag_Pos_E,init_Mag_Pos_D });
        double[,] M_Vec = Make_Vec(VecT = new double[3] { 0, 0, -1 } );
        double[,] R_Vec = new double[3, 1];
        double[,] Temp_Vec3 = new double[3, 1];
        double[,] Result = new double[3, 1];
        double Temp_Scalar;
        double R_Norm;
        R_Vec = Mat_Cal(T_Pos, '-', Mag_Pos);
        R_Norm = Vec_Norm(R_Vec);
        R_Vec = Mat_Scalar(1/R_Norm,R_Vec);
        Temp_Scalar = u0/(4*Math.PI*R_Norm*R_Norm);
        Temp_Vec3 = Vec3_Cross(M_Vec,R_Vec);
        Result = Mat_Scalar(Temp_Scalar,Temp_Vec3);

        return Result;
    }
    public double[,] Vec3_Cross(double[,] Vec1,double[,] Vec2)
    {
        double[,] result = new double[3, 1];

        double a1, a2, a3, b1, b2, b3;

        a1 = Vec1[0, 0];
        a2 = Vec1[1, 0];
        a3 = Vec1[2, 0];
        b1 = Vec2[0, 0];
        b2 = Vec2[1, 0];
        b3 = Vec2[2, 0];

        result[0,0] = a2*b3 - a3*b2;
        result[1,0] = a1*b3 - a3*b1;
        result[2,0] = a1*b2 - a2*b1;

        return result;
    }
    public double[,] Cal_GeoMag()
    {
        double u0 = 0.4;
        double[,] Geo_M_Vec = Make_Vec(VecT = new double[3] { 1, 0, 0 } );
        double[,] Result = new double[3, 1];
        
        Geo_M_Vec = Mat_Scalar(u0,Geo_M_Vec);
        Result = Mat_Multi(Mat_Inv(T_Cbn), Geo_M_Vec);

        return Result;
    }
    public double[,] Cal_S_Mag()
    {
        double[,] Result = new double[3, 1];

        Result = Mat_Cal(Cal_GeoMag(), '+', Mag_Dipole_Moment());
        //Result = Cal_GeoMag();

        return Result;
    }
    



    //  ┌--------------------┐
    //  │  Vector Calculate  │
    //  └--------------------┘
    public double[,] Make_Vec(double[] List)
    {
        int n = List.GetLength(0);
        double[,] result = new double[n, 1];
        for (int i = 0; i < n; i++)
        {
            result[i, 0] = List[i];
        }
        return result;
    }
    public double Vec_Norm(double[,] Vec)
    {
        int n = Vec.GetLength(0);

        double result = 0.0;

        for (int i = 0; i < n; i++)
        {
            result += Vec[i, 0] * Vec[i, 0];
        }
        result = Math.Sqrt(result);

        return result;
    }
    public double[,] Vec_integate(double[,] Pre_Vec, double[,] Pre_dVec, double[,] dVec, double dt)
    {
        int n = Pre_Vec.GetLength(0);
        double[,] result = new double[n, 1];

        for (int i = 0; i < n; i++)
        {
            result[i, 0] = Pre_Vec[i, 0] + (0.5 * (Pre_dVec[i, 0] + dVec[i, 0]) * dt);
        }

        return result;
    }
    //  ┌--------------------┐
    //  │  Matrix Calculate  │
    //  └--------------------┘
    public double[,] Mat_Multi(double[,] m1, double[,] m2)
    {
        double[,] result = new double[m1.GetLength(0), m2.GetLength(1)];

        if (m1.GetLength(1) == m2.GetLength(0))
        {
            for (int i = 0; i < result.GetLength(0); i++)
            {
                for (int j = 0; j < result.GetLength(1); j++)
                {
                    result[i, j] = 0;
                    for (int k = 0; k < m1.GetLength(1); k++)
                        result[i, j] = result[i, j] + m1[i, k] * m2[k, j];
                }
            }
        }
        return result;
    }
    public double[,] Mat_Inv(double[,] matrix)
    {
        int n = matrix.GetLength(0);

        double[,] result = new double[n, n];
        double[,] tmpWork = new double[n, n];
        Array.Copy(matrix, tmpWork, n * n);  // 기존값을 보존하기 위함.  

        // 계산 결과가 저장되는 result 행렬을 단위행렬로 초기화
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                result[i, j] = (i == j) ? 1 : 0;

        // 대각 요소를 0 이 아닌 수로 만듦
        const double ERROR = 1.0e-10;
        for (int i = 0; i < n; i++)
            if (-ERROR < tmpWork[i, i] && tmpWork[i, i] < ERROR) //if (-ERROR < tmpWork[i, i] && tmpWork[i, i] < ERROR)
            {
                for (int k = 0; k < n; k++)
                {
                    if (-ERROR < tmpWork[k, i] && tmpWork[k, i] < ERROR) continue;
                    for (int j = 0; j < n; j++)
                    {
                        tmpWork[i, j] += tmpWork[k, j];
                        result[i, j] += result[k, j];  // result[i*n+j] += result[k*n+j];
                    }
                    break;
                }
                if (-ERROR < tmpWork[i, i] && tmpWork[i, i] < ERROR) return result;
            }

        // Gauss-Jordan eliminatio
        for (int i = 0; i < n; i++)
        {
            // 대각 요소를 1로 만듦
            double constant = tmpWork[i, i];      // 대각 요소의 값 저장
            for (int j = 0; j < n; j++)
            {
                tmpWork[i, j] /= constant;   // tmpWork[i][i] 를 1 로 만드는 작업
                result[i, j] /= constant; // result[i*n+j] /= constant;   // i 행 전체를 tmpWork[i][i] 로 나눔
            }

            // i 행을 제외한 k 행에서 tmpWork[k][i] 를 0 으로 만드는 단계
            for (int k = 0; k < n; k++)
            {
                if (k == i) continue;      // 자기 자신의 행은 건너뜀
                if (tmpWork[k, i] == 0) continue;   // 이미 0 이 되어 있으면 건너뜀

                // tmpWork[k][i] 행을 0 으로 만듦
                constant = tmpWork[k, i];
                for (int j = 0; j < n; j++)
                {
                    tmpWork[k, j] = tmpWork[k, j] - tmpWork[i, j] * constant;
                    result[k, j] = result[k, j] - result[i, j] * constant;  // result[k*n+j] = result[k*n+j] - result[i*n+j] * constant;
                }
            }
        }
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if(double.IsNaN(result[i,i]))
                {
                    Debug.Log("Error in Inverse");  
                }
            }
        }   

        return result;
    }
    public double[,] Mat_Transpose(double[,] Mat)
    {
        int m = Mat.GetLength(0);
        int n = Mat.GetLength(1);
        double[,] result = new double[n, m];

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                result[j, i] = Mat[i, j];
            }
        }

        return result;
    }
    public double[,] Mat_Scalar(double K, double[,] Mat)
    {
        int m = Mat.GetLength(0);
        int n = Mat.GetLength(1);
        double[,] result = new double[m, n];

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                result[i, j] = K * Mat[i, j];
            }
        }

        return result;
    }
    public double[,] Mat_Cal(double[,] Mat1, char Cal, double[,] Mat2)
    {
        int m = Mat1.GetLength(0);
        int n = Mat1.GetLength(1);
        double[,] result = new double[m, n];

        if (Cal == '+')
        {

            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[i, j] = Mat1[i, j] + Mat2[i, j];
                }
            }
        }
        else if (Cal == '-')
        {
            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    result[i, j] = Mat1[i, j] - Mat2[i, j];
                }
            }
        }
        return result;
    }


    //  ┌--------------------┐
    //  │   Nav Calculate    │
    //  └--------------------┘
    public double[,] Cal_Euler_2_Cbn(double[,] In)
    {
        double[,] Out = new double[3, 3];
        double R, P, Y;

        R = In[0, 0];
        P = In[1, 0];
        Y = In[2, 0];

        Out[0, 0] = (Math.Cos(P) * Math.Cos(Y));
        Out[0, 1] = (-Math.Cos(R) * Math.Sin(Y)) + (Math.Sin(R) * Math.Sin(P) * Math.Cos(Y));
        Out[0, 2] = (Math.Sin(R) * Math.Sin(Y)) + (Math.Cos(R) * Math.Sin(P) * Math.Cos(Y));

        Out[1, 0] = (Math.Cos(P) * Math.Sin(Y));
        Out[1, 1] = (Math.Cos(R) * Math.Cos(Y)) + (Math.Sin(R) * Math.Sin(P) * Math.Sin(Y));
        Out[1, 2] = (-Math.Sin(R) * Math.Cos(Y)) + (Math.Cos(R) * Math.Sin(P) * Math.Sin(Y));

        Out[2, 0] = (-Math.Sin(P));
        Out[2, 1] = (Math.Sin(R) * Math.Cos(P));
        Out[2, 2] = (Math.Cos(R) * Math.Cos(P));


        return Out;
    }
    public double[,] Cal_Cbn_2_Euler(double[,] In)
    {
        double[,] Out = new double[3, 1];

        Out[0, 0] = Math.Atan(In[2, 1] / In[2, 2]);
        Out[1, 0] = Math.Asin(-In[2, 0]);
        Out[2, 0] = Math.Atan2(In[1, 0], In[0, 0]);

        return Out;
    }
    public double[,] Cal_Quat_2_Cbn(double[,] In)
    {
        double[,] Out = new double[3, 3];
        double a, b, c, d;
        a = In[0, 0];
        b = In[1, 0];
        c = In[2, 0];
        d = In[3, 0];

        Out[0, 0] = (a * a) + (b * b) - (c * c) - (d * d);
        Out[0, 1] = 2 * ((b * c) - (a * d));
        Out[0, 2] = 2 * ((b * d) + (a * c));

        Out[1, 0] = 2 * ((b * c) + (a * d));
        Out[1, 1] = (a * a) - (b * b) + (c * c) - (d * d);
        Out[1, 2] = 2 * ((c * d) + (a * b));

        Out[2, 0] = 2 * ((b * d) - (a * c));
        Out[2, 1] = 2 * ((c * d) + (a * b));
        Out[2, 2] = (a * a) - (b * b) - (c * c) + (d * d);

        return Out;
    }
    public double[,] Cal_Cbn_2_Quat(double[,] In)
    {
        double[,] Out = new double[4, 1];
        double a, b, c, d;
        a = 0.5 * Math.Sqrt(1 + In[0, 0] + In[1, 1] + In[2, 2]);
        b = (1 / (4 * a)) * (In[2, 1] - In[1, 2]);
        c = (1 / (4 * a)) * (In[0, 2] - In[2, 0]);
        d = (1 / (4 * a)) * (In[1, 0] - In[0, 1]);

        Out[0, 0] = a;
        Out[1, 0] = b;
        Out[2, 0] = c;
        Out[3, 0] = d;

        return Out;
    }
    public double[,] Cal_Quat_Multi(double[,] Q1, double[,] Q2)
    {
        double[,] Out = new double[4, 1];
        double[,] Temp = new double[4, 4];

        Temp[0, 0] = Q1[0, 0];
        Temp[1, 0] = Q1[1, 0];
        Temp[2, 0] = Q1[2, 0];
        Temp[3, 0] = Q1[3, 0];

        Temp[0, 1] = -Q1[1, 0];
        Temp[1, 1] = Q1[0, 0];
        Temp[2, 1] = Q1[3, 0];
        Temp[3, 1] = -Q1[2, 0];

        Temp[0, 2] = -Q1[2, 0];
        Temp[1, 2] = -Q1[3, 0];
        Temp[2, 2] = Q1[0, 0];
        Temp[3, 2] = Q1[1, 0];

        Temp[0, 3] = -Q1[3, 0];
        Temp[1, 3] = Q1[2, 0];
        Temp[2, 3] = -Q1[1, 0];
        Temp[3, 3] = Q1[0, 0];

        Out = Mat_Multi(Temp, Q2);
        return Out;
    }

    //  ┌--------------------┐
    //  │  Vector Calculate  │
    //  └--------------------┘
    public void Visualization()
    {
        V_Pos = new Vector3((float)(T_Pos[1, 0]), -(float)(T_Pos[2, 0]), -(float)(T_Pos[0, 0]));
        transform.position = V_Pos;
        V_Att = new Vector3((float)(R2D * T_Euler[1, 0]), (float)(R2D * T_Euler[2, 0]), (float)(R2D * T_Euler[0, 0]));
        V_rAtt = transform.rotation.eulerAngles;
        transform.Rotate(V_Att - V_rAtt);
    }

    //  ┌--------------------┐
    //  │   Command  Input   │
    //  └--------------------┘
    public void RC_Input_A()
    {
        if (Input.GetKey(KeyCode.W))
        {
            CMD_Att[1, 0] = 15;
            
        }
        else if (Input.GetKey(KeyCode.X))
        {
            CMD_Att[1, 0] = -15;
            

        }
        else if (Input.GetKey(KeyCode.A))
        {
            CMD_Att[0, 0] = 15;
        }
        else if (Input.GetKey(KeyCode.D))
        {
            CMD_Att[0, 0] = -15;
        }
        else if (Input.GetKey(KeyCode.Q))
        {
            CMD_Att[2, 0] -= 1;
           
            
        }
        else if (Input.GetKey(KeyCode.E))
        {
            CMD_Att[2, 0] += 1;
         
            
        }
        else if (Input.GetKey(KeyCode.S))
        {
            CMD_Att[0, 0] = 0;
            CMD_Att[1, 0] = 0;
            CMD_Att[2, 0] = 0;
            
        }
        else if (Input.GetKey(KeyCode.UpArrow))
        {
           CMD_Pos[2, 0] = -5;
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            CMD_Pos[2, 0] = 0;
        }
        else if (Input.GetKey(KeyCode.Space))
        {
            CMD_Pos[2, 0] = -2.5;
        }
    }
  
}