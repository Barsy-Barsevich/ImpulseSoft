//<Коэффициенты формулы момента инерции>
#define INER_Moment_1_koef0     0.05111816524578927101
#define INER_Moment_1_koef1     -0.263696806593884502951
#define INER_Moment_1_koef2     0.01126117035721563298
#define INER_Moment_1_koef3     5.98586219865491386827
#define INER_Moment_2_koef0     0.00478001376677639200
#define INER_Moment_2_koef1     -0.15239480305262986803
#define INER_Moment_2_koef2     1.45047905319370329380
#define INER_Moment_2_koef3     -1.54991495460853911936
//<Коэффициенты формулы плеча сил тангажа и рыскания>
#define POW_SHOULDER_1_koef0    0.00355929122432030454
#define POW_SHOULDER_1_koef1    -0.01836562658371576617
#define POW_SHOULDER_1_koef2    0.00079841025863558457
#define POW_SHOULDER_1_koef3    0.37022625308120638010
#define POW_SHOULDER_2_koef0    0.00078288704315365720
#define POW_SHOULDER_2_koef1    -0.02498261410372037972
#define POW_SHOULDER_2_koef2    0.23890333014128373179
#define POW_SHOULDER_2_koef3    -0.60041156685929308878
//<Константы плеча сил тангажа и рыскания и момента инерции>
#define CONST_1_INER_MOMENT     4.965
#define CONST_1_POW_SHOULDER    0.29918
#define CONST_2_INER_MOMENT     2.057
#define CONST_2_POW_SHOULDER    0.373
//<Плечо силы крена>
#define POW_SHOULDER_ROLL       0.062
//<Константы для вычисления углов отклонения>
#define AIR_CONST               573.1
#define STAB_POV                0.011437929//must be 0.004
#define CYtoANGLE_koef0         61866.37182810902595520020
#define CYtoANGLE_koef1         -5314.59035779116675257683
#define CYtoANGLE_koef2         398.96077448246069252491
#define CYtoANGLE_koef3         0.01249933278211301513
//<>
#define tpar_py                 0.9//2
#define tpar_roll               0.9//2
//<>
#define ANGLE_FILTER_QUEUE      15//5//15
#define DELTA_FILTER_QUEUE      20//5//20
//<>
#define Cy_krit                 0.052

void CALC_IM_PS_PARAMETERS(void);
void CALC_1_INER_MOMENT(void);
void CALC_2_INER_MOMENT(void);
void CALC_1_POW_SHOULDER(void);
void CALC_2_POW_SHOULDER(void);
void CALC_STAB_ANGLE(void);
void CALC_RPY_Forces(void);
void CALC_Stab_Forces(void);
void CALC_DRPY_AND_DELTA_FILTER(void);


float afil[ANGLE_FILTER_QUEUE];
float bfil[ANGLE_FILTER_QUEUE];
float cfil[ANGLE_FILTER_QUEUE];
float dfil[ANGLE_FILTER_QUEUE];

float dp_fil[DELTA_FILTER_QUEUE];
float dy_fil[DELTA_FILTER_QUEUE];
float dr_fil[DELTA_FILTER_QUEUE];


void CALC_IM_PS_PARAMETERS() {
    if (system_mode < 5/*!Stage_2_flag*/) {
        //1 ступень
        if (eng_time < 3500000) {
            //использовать 1 функцию плеча силы и момента инерции
            CALC_1_INER_MOMENT();
            CALC_1_POW_SHOULDER();
        }
        else {
            //использовать 1 константу плеча силы и момента инерции
            InertionMoment = CONST_1_INER_MOMENT;
            PowShoulder = CONST_1_POW_SHOULDER;
        }
    }
    else {
        //2 ступень
        if (eng_time < 5600000) {
            //использовать 2 функцию плеча силы и момента инерции
            CALC_2_INER_MOMENT();
            CALC_2_POW_SHOULDER();
        }
        else {
            //использовать 2 константу плеча силы и момента инерции
            InertionMoment = CONST_2_INER_MOMENT;
            PowShoulder = CONST_2_POW_SHOULDER;
        }
    }
}
void CALC_1_INER_MOMENT() {
    InertionMoment = (INER_Moment_1_koef0 * eng_time3) +
                     (INER_Moment_1_koef1 * eng_time2) +
                     (INER_Moment_1_koef2 * eng_time) +
                     (INER_Moment_1_koef3);
}
void CALC_2_INER_MOMENT() {
    InertionMoment = (INER_Moment_2_koef0 * eng_time3) +
                     (INER_Moment_2_koef1 * eng_time2) +
                     (INER_Moment_2_koef2 * eng_time) +
                     (INER_Moment_2_koef3);
}
void CALC_1_POW_SHOULDER() {
    PowShoulder = (POW_SHOULDER_1_koef0 * eng_time3) +
                  (POW_SHOULDER_1_koef1 * eng_time2) +
                  (POW_SHOULDER_1_koef2 * eng_time) +
                  (POW_SHOULDER_1_koef3);
}
void CALC_2_POW_SHOULDER() {
    PowShoulder = (POW_SHOULDER_2_koef0 * eng_time3) +
                  (POW_SHOULDER_2_koef1 * eng_time2) +
                  (POW_SHOULDER_2_koef2 * eng_time) +
                  (POW_SHOULDER_2_koef3);
}

void CALC_STAB_ANGLE() {
    // Нахождение знаков сил всех стабов
    float Force_Sign_A;
    float Force_Sign_B;
    float Force_Sign_C;
    float Force_Sign_D;
    if (Stab_A_Force < 0) {
        Force_Sign_A = -1;
        Stab_A_Force = -Stab_A_Force;
    }
    else Force_Sign_A = 1;
    if (Stab_B_Force < 0) {
        Force_Sign_B = -1;
        Stab_B_Force = -Stab_B_Force;
    }
    else Force_Sign_B = 1;
    if (Stab_C_Force < 0) {
        Force_Sign_C = -1;
        Stab_C_Force = -Stab_C_Force;
    }
    else Force_Sign_C = 1;
    if (Stab_D_Force < 0) {
        Force_Sign_D = -1;
        Stab_D_Force = -Stab_D_Force;
    }
    else Force_Sign_D = 1;
    
    // Вычисление коэффициента подъемной силы для стабилизаторов
    float prop_val = ((Temperature + 273.16) * AIR_CONST)/
        (Pressure * STAB_POV * VertSpeed * VertSpeed);
    float Cy_A = prop_val * Stab_A_Force;
    float Cy_B = prop_val * Stab_B_Force;
    float Cy_C = prop_val * Stab_C_Force;
    float Cy_D = prop_val * Stab_D_Force;
    
    // Нахождение максимального значения Cy для проведения коррекции
    float Cy_max;
    if (Cy_A > Cy_B) {
        if (Cy_C > Cy_D) {
            if (Cy_A > Cy_C) Cy_max = Cy_A; // Cy_A максимальное
            else Cy_max = Cy_C;             // Cy_C максимальное
        }
        else {
            if (Cy_A > Cy_D) Cy_max = Cy_A; // Cy_A максимальное
            else Cy_max = Cy_D;             // Cy_D максимальное
        }
    }
    else {
        if (Cy_C > Cy_D) {
            if (Cy_B > Cy_C) Cy_max = Cy_B; // Cy_B максимальное
            else Cy_max = Cy_C;             // Cy_C максимальное
        }
        else {
            if (Cy_B > Cy_D) Cy_max = Cy_B; // Cy_B максимальное
            else Cy_max = Cy_D;             // Cy_D максимальное
        }
    }
    // Коррекция
    if (Cy_max > Cy_krit) {
        float correction = Cy_krit / Cy_max;
        Cy_A *= correction;
        Cy_B *= correction;
        Cy_C *= correction;
        Cy_D *= correction;
        /*UART_TRANSMIT(' ');
        dtostrf(correction, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }//*/
    }
    else {
        /*UART_TRANSMIT(' ');
        dtostrf(1, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }//*/
    }//*/
    
    // Вычисление углов
    float Cy_A2 = Cy_A * Cy_A;
    StabA = ((CYtoANGLE_koef0 * Cy_A2 * Cy_A) + (CYtoANGLE_koef1 * Cy_A2) +
        (CYtoANGLE_koef2 * Cy_A) + CYtoANGLE_koef3) * Force_Sign_A;
    float Cy_B2 = Cy_B * Cy_B;
    StabB = ((CYtoANGLE_koef0 * Cy_B2 * Cy_B) + (CYtoANGLE_koef1 * Cy_B2) +
        (CYtoANGLE_koef2 * Cy_B) + CYtoANGLE_koef3) * Force_Sign_B;
    float Cy_C2 = Cy_C * Cy_C;
    StabC = ((CYtoANGLE_koef0 * Cy_C2 * Cy_C) + (CYtoANGLE_koef1 * Cy_C2) +
        (CYtoANGLE_koef2 * Cy_C) + CYtoANGLE_koef3) * Force_Sign_C;
    float Cy_D2 = Cy_D * Cy_D;
    StabD = ((CYtoANGLE_koef0 * Cy_D2 * Cy_D) + (CYtoANGLE_koef1 * Cy_D2) +
        (CYtoANGLE_koef2 * Cy_D) + CYtoANGLE_koef3) * Force_Sign_D;
        
    // Фильтрация углов
    float sum_a = 0;
    float sum_b = 0;
    float sum_c = 0;
    float sum_d = 0;
    for (uint8_t i=(ANGLE_FILTER_QUEUE-1); i>0; i--) {
        afil[i] = afil[i-1];
        bfil[i] = bfil[i-1];
        cfil[i] = cfil[i-1];
        dfil[i] = dfil[i-1];
        sum_a += afil[i];
        sum_b += bfil[i];
        sum_c += cfil[i];
        sum_d += dfil[i];
    }
    afil[0] = StabA;
    bfil[0] = StabB;
    cfil[0] = StabC;
    dfil[0] = StabD;
    sum_a += StabA;
    sum_b += StabB;
    sum_c += StabC;
    sum_d += StabD;
    StabA = sum_a / ANGLE_FILTER_QUEUE;
    StabB = sum_b / ANGLE_FILTER_QUEUE;
    StabC = sum_c / ANGLE_FILTER_QUEUE;
    StabD = sum_d / ANGLE_FILTER_QUEUE;
}


void CALC_RPY_Forces() { //PowShoulder 0.4
    Force_Pitch = -(2 * InertionMoment)*(Pitch + (dPitch/deltat) * tpar_py)/
        (tpar_py * tpar_py * PowShoulder);
    Force_Yaw = -(2 * InertionMoment)*(Yaw + (dYaw/deltat) * tpar_py)/
        (tpar_py * tpar_py * PowShoulder);
    Force_Roll = -(2 * InertionMoment * dRoll)/(tpar_roll * deltat *
        POW_SHOULDER_ROLL);
}

void CALC_Stab_Forces() {
    Stab_A_Force = (Force_Pitch / 2) + (Force_Roll / 4); //крен должен быть
    Stab_B_Force = (Force_Yaw / 2) + (Force_Roll / 4);
    Stab_C_Force = -(Force_Pitch / 2) + (Force_Roll / 4);
    Stab_D_Force = -(Force_Yaw / 2) + (Force_Roll / 4);
}

void CALC_DRPY_AND_DELTA_FILTER() {
    dRoll = Roll - pre_Roll;
    dPitch = Pitch - pre_Pitch;
    dYaw = Yaw - pre_Yaw;
    pre_Roll = Roll;
    pre_Pitch = Pitch;
    pre_Yaw = Yaw;
    // delta-filter
    float sum_a = 0;
    float sum_b = 0;
    float sum_c = 0;
    for (uint8_t i=(DELTA_FILTER_QUEUE-1); i>0; i--) {
        dp_fil[i] = dp_fil[i-1];
        dy_fil[i] = dy_fil[i-1];
        dr_fil[i] = dr_fil[i-1];
        sum_a += dp_fil[i];
        sum_b += dy_fil[i];
        sum_c += dr_fil[i];
    }
    dp_fil[0] = dPitch;
    dy_fil[0] = dYaw;
    dr_fil[0] = dRoll;
    sum_a += dPitch;
    sum_b += dYaw;
    sum_c += dRoll;
    dPitch = sum_a / DELTA_FILTER_QUEUE;
    dYaw = sum_b / DELTA_FILTER_QUEUE;
    dRoll = sum_c / DELTA_FILTER_QUEUE;
}
