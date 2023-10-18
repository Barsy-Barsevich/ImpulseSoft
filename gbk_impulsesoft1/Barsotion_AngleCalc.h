//<Коэффициенты формулы момента инерции>
#define INER_Moment_1_koef0     1
#define INER_Moment_1_koef1     1
#define INER_Moment_1_koef2     1
#define INER_Moment_1_koef3     1
#define INER_Moment_2_koef0     1
#define INER_Moment_2_koef1     1
#define INER_Moment_2_koef2     1
#define INER_Moment_2_koef3     1
//<Коэффициенты формулы плеча сил тангажа и рыскания>
#define POW_SHOULDER_1_koef0    1
#define POW_SHOULDER_1_koef1    1
#define POW_SHOULDER_1_koef2    1
#define POW_SHOULDER_1_koef3    1
#define POW_SHOULDER_2_koef0    1
#define POW_SHOULDER_2_koef1    1
#define POW_SHOULDER_2_koef2    1
#define POW_SHOULDER_2_koef3    1
//<Константы плеча сил тангажа и рыскания и момента инерции>
#define CONST_1_INER_MOMENT     1
#define CONST_1_POW_SHOULDER    1
#define CONST_2_INER_MOMENT     1
#define CONST_2_POW_SHOULDER    1
//<Плечо силы крена>
#define POW_SHOULDER_ROLL       0.06
//<Константы для вычисления углов отклонения>
#define AIR_CONST               573.1
#define STAB_POV                0.01//must be 0.004
#define CYtoANGLE_koef0         19500.8333
#define CYtoANGLE_koef1         -5035.4957
#define CYtoANGLE_koef2         618.9478
#define CYtoANGLE_koef3         0.3391
//<>
#define tpar_py                 2//0.9
#define tpar_roll               2//0.9
//<>
#define ANGLE_FILTER_QUEUE      15
#define DELTA_FILTER_QUEUE      20
//<>
#define Cy_krit                 0.15

void CALC_IM_PS_PARAMETERS(void);
void CALC_1_INER_MOMENT(void);
void CALC_2_INER_MOMENT(void);
void CALC_1_POW_SHOULDER(void);
void CALC_2_POW_SHOULDER(void);
void CALC_STAB_ANGLE(void);
void CALC_RPY_Forces(void);
void CALC_Stab_Forces(void);
void CALC_DRPY_AND_DELTA_FILTER(void);


float afil[30];
float bfil[30];
float cfil[30];
float dfil[30];


void CALC_IM_PS_PARAMETERS() {
    if (!Stage_2_flag) {
        //1 ступень
        if (eng_time < 3.5) {
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
        if (eng_time < 5.6) {
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
        (Pressure * STAB_POV * /*VertSpeed*/115 * /*VertSpeed*/115);
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
        UART_TRANSMIT(' ');
        dtostrf(correction, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }//*/
    }
    else {
        UART_TRANSMIT(' ');
        dtostrf(1, 7, 5, str);
        for (uint8_t i = 0; i<6; i++) {
            UART_TRANSMIT(str[i]);
        }
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
    Force_Pitch = -(2 * /*InertionMoment*/3)*(Pitch + (dPitch/deltat) * tpar_py)/
        (tpar_py * tpar_py * /*PowShoulder*/0.8);
    Force_Yaw = -(2 * /*InertionMoment*/3)*(Yaw + (dYaw/deltat) * tpar_py)/
        (tpar_py * tpar_py * /*PowShoulder*/0.8);
    Force_Roll = -(2 * /*InertionMoment*/3 * dRoll)/(tpar_roll * deltat *
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
        staba_fil[i] = staba_fil[i-1];
        stabb_fil[i] = stabb_fil[i-1];
        stabc_fil[i] = stabc_fil[i-1];
        sum_a += staba_fil[i];
        sum_b += stabb_fil[i];
        sum_c += stabc_fil[i];
    }
    staba_fil[0] = dPitch;
    stabb_fil[0] = dYaw;
    stabc_fil[0] = dRoll;
    sum_a += dPitch;
    sum_b += dYaw;
    sum_c += dRoll;
    dPitch = sum_a / DELTA_FILTER_QUEUE;
    dYaw = sum_b / DELTA_FILTER_QUEUE;
    dRoll = sum_c / DELTA_FILTER_QUEUE;
}
