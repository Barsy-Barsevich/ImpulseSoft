#define EN1     PC2
#define IN11    PC3
#define IN12    PC6
#define EN2     PC7
#define IN21    PC5
#define IN22    PC4
#define EN3     PA2
#define IN31    PA4
#define IN32    PA5
#define EN4     PA7
#define IN41    PA6
#define IN42    PA3

bool STAGE1_SENSORS(void);
void STAGE1_EXTRACT(void);
void STAGE1_ADD(void);
void STAGE1_STOP(void);
void STAGE2_IGNITION_COMMAND(void);
void STAGE2_STOP_IGNITION(void);
void STAGE2_OBT_OPEN(void);
void STAGE2_OBT_CLOSE(void);
void STAGE2_SSMS_OPEN(void);
void STAGE2_SSMS_CLOSE(void);
void STAGE2_SSMS_STOP(void);

void STAGE1_EXTRACT() {
    PORTC
}
