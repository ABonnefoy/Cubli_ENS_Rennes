/* DriverLib Includes */
//#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//#include <driverlib.h>
//#include <ti/msp/MSP432Ware_3_50_00_02/driverlib/driverlib/MSP432P4xx/driverlib.h>
//#include <ti/msp/MSP432Ware_3_50_00_02/driverlib/driverlib/MSP432P4xx/ccs/msp432p4xx_driverlib.lib>
//#include <ti/simplelink_msp432p4_sdk_2_10_00_14/source/ti/devices/msp432p4xx/driverlib/ccs/msp432p4xx_driverlib.lib>
#include "msp.h"
#include <msp432p401r.h>
#include <stdint.h>

/**
 * main.c
 */

void setPWM(float);
void setDir(int);

long pos_corps;
long vitesse_corps;
long prev_tic_corps;
long prev_pos_corps;
long pos_roue;
long prev_tic_roue;
long prev_pos_roue;
long vitesse_roue;
float K1,K2,K3; // coefficients du correcteur LQR
float Kc; // Constante de couple
float cons; // consigne de couple

volatile uint16_t uMax;
volatile uint16_t uMin;


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
//  pour une raison inconnue, tous les registres associes au TIMER32 ne sont pas reconnus
//    T32_CONTROL1 = BIT7 + BIT6 + BIT1;
//    TIMER32_CONTROL1 &=~ BIT5 + BIT3 + BIT2 + BIT0;
//    TIMER32_LOAD1 = 0xFFFFFFF;
//   MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, //TIMER32_BASE
//            TIMER32_PERIODIC_MODE);

    // TA0CTL0, configure le timer avec SMCLK, no division, up mode, and clear it now.
    TA0CTL = TASSEL_2 | ID_0 | MC_1 | TACLR | TAIE;
    // TimerA compte alors jusqu'à la valeur indiquée dans TA0CCR0 à une fréquence de 48MHz.
    // La fréquence de la PWM est alors réglée par TA0CCR0, interrupt value (max is 2^16);
    TA0CCR0 = 1200;  // fréquence de sortie de 2.5kHz
    uMax = TA0CCR0;
    uMin = uMax>>7;
    // rapport cyclique initial
    TA0CCR1 = uMax>>1;  // Default 50% duty cycle
    // autoriser les interruptions sur le timerA
    TA0CCTL0 = CCIE;
    TA0CCTL1 = CCIE;
    NVIC_EnableIRQ(TA0_0_IRQn);
    NVIC_EnableIRQ(TA0_N_IRQn);
    NVIC_SetPriority(TA0_0_IRQn,2);
    NVIC_SetPriority(TA0_N_IRQn,2);


    P1DIR = BIT0 + BIT6 + BIT7; // LED1 en sortie, 1.6 en sortie pour la PWM, 1.7 pour la direction, autres broches en entree
    P2DIR = BIT0 + BIT1 + BIT2; // LED RBG en sortie, autres broches en entree
    // entrees du codeur du corps sur le port 2
    // A sur 2.4 et 2.5, B sur 2.6 et 2.7
    P2IE |= BIT4 + BIT5 + BIT6 + BIT7; // interrupt enable sur 2.4, 2.5, 2.6 et 2.7
    P2IES |= BIT4 + BIT6; // interruption sur front descendant pour 2.4 et 2.6
    P2IES |=~ BIT5 + BIT7; // interruption sur front montant pour 2.5 et 2.7
    P2IFG &=~ BIT4 + BIT5 + BIT6 + BIT7; // clear des flags d'interruption
    NVIC_EnableIRQ(PORT2_IRQn); // parametrage du Nested Vector Interrupt Controller
    NVIC_SetPriority(PORT2_IRQn,0); // niveau le plus prioritaire
    pos_corps = 0;
    prev_tic_corps = 0;
    prev_pos_corps = 0;
    // entrees du codeur de la roue sur le port 4
    // A sur 4.4 et 4.5, B sur 4.6 et 4.7
    P4IE |= BIT4 + BIT5 + BIT6 + BIT7; // interrupt enable sur 4.4, 4.5, 4.6 et 4.7
    P4IES |= BIT4 + BIT6; // interruption sur front descendant pour 4.4 et 4.6
    P4IES |=~ BIT5 + BIT7; // interruption sur front montant pour 4.5 et 4.7
    P4IFG &=~ BIT4 + BIT5 + BIT6 + BIT7; // clear des flags d'interruption
    NVIC_EnableIRQ(PORT4_IRQn); // parametrage du Nested Vector Interrupt Controller
    NVIC_SetPriority(PORT4_IRQn,0);
    pos_roue = 0;
    prev_tic_roue = 0;
    prev_pos_roue = 0;
    __enable_interrupt();

    // valeurs des coefficients du correcteur LQR
    K1 = -19.057;
    K2 = 0.041;
    K3 = -0.002;
    Kc = 33.5;
    cons = (K1*pos_corps+K2*vitesse_corps+K3*vitesse_roue)/Kc;
    setPWM(cons);

    printf("initialisation reussie");
    while(1){}
}

void setPWM(float dutyCycle)
{
    // Le rapport cyclique est indique comme un pourcentage.
    // Conversion vers un nombre entier a indiquer dans TA0CCCR1
    int dutyCycle_int = dutyCycle*uMax/65535;
    if(dutyCycle_int < uMin) dutyCycle_int = uMin;
    if(dutyCycle_int > uMax) dutyCycle_int = uMax;
    TA0CCR1 = dutyCycle_int;
}

void TA0_0_IRQHandler()
{
    // Le timer a atteint sa valeur maximale TA0CCR0
    TA0CCTL0 &= ~CCIFG;  // Remise a 0 des flag
    TA0CTL   &= ~TAIFG;  //
    TA0CCTL1 &= ~CCIFG;
    P1OUT    |= BIT6;    // Broche de sortie a l'etat haut
    TA0CCTL1 |= CCIE;    // Lancement d'un nouveau cycle de comptage
}

void TA0_N_IRQHandler()
{   // Le timer a atteint sa valeur intermédiaire
    if(TA0CCTL1 & CCIFG)
    {
        TA0CCTL1 &= ~CCIFG; // Remise a 0 des flag
        P1OUT    &= ~BIT6;  // Broche de sortie a l'etat bas
    }
}

void setDIR(uint8_t dir)
{
    if(dir == 0)
    {
        P1OUT |= BIT7;
    }
    else
    {
        P1OUT &= ~BIT7;
    }
}

void PORT2_IRQHandler(void){ // handler d'une interruption sur le port2 : increment du codeur incremental du corps
    if (P2IFG & BIT6){ // front descendant sur B
        if (P2IN & BIT4){ // etat haut sur A
            pos_corps--;
        }
        else{ // etat bas sur A
            pos_corps++;
        }
        P2IFG &=~ BIT6;
    }
    if (P2IFG & BIT4){ // front descendant sur A
        if (P2IN & BIT6){ // etat haut sur B
            pos_corps++;
        }
        else{ // etat bas sur B
            pos_corps--;
        }
        P2IFG &=~ BIT4;
    }
    if (P2IFG & BIT7){ // front montant sur B
        if (P2IN & BIT4){ // etat haut sur A
            pos_corps++;
        }
        else{ // etat bas sur A
            pos_corps--;
        }
        P2IFG &=~ BIT7;
    }
    if (P2IFG & BIT5){ // front montant sur A
        if (P2IN & BIT6){ // etat haut sur B
            pos_corps--;
        }
        else{ // etat bas sur B
            pos_corps++;
        }
        P2IFG &=~ BIT5;
     }
    if(pos_corps > 3600)  pos_corps = pos_corps - 3600;
    if(pos_corps <    0)  pos_corps = pos_corps + 3600;
    long temps = TA0R; // valeur du timerA
    vitesse_corps = (pos_corps-prev_pos_corps)*(2*3.14/3600)/(temps - prev_tic_corps); // mutiplier par les caractéristiques de timerA
    pos_corps = pos_corps*2*3.14/3600;
    prev_tic_corps = TA0R;
    prev_pos_corps = pos_corps;
}

void PORT4_IRQHandler(void){ // handler d'une interruption sur le port3 : increment du codeur incremental de la roue
    if (P4IFG & BIT6){ // front descendant sur B
        if (P4IN & BIT4){ // etat haut sur A
            pos_roue--;
        }
        else{ // etat bas sur A
            pos_roue++;
        }
        P4IFG &=~ BIT6;
    }
    if (P4IFG & BIT4){ // front descendant sur A
        if (P4IN & BIT6){ // etat haut sur B
            pos_roue++;
        }
        else{ // etat bas sur B
            pos_roue--;
        }
        P4IFG &=~ BIT4;
    }
    if (P4IFG & BIT7){ // front montant sur B
        if (P4IN & BIT4){ // etat haut sur A
            pos_roue++;
        }
        else{ // etat bas sur A
            pos_roue--;
        }
        P4IFG &=~ BIT7;
    }
    if (P4IFG & BIT5){ // front montant sur A
        if (P4IN & BIT6){ // etat haut sur B
            pos_roue--;
        }
        else{ // etat bas sur B
            pos_roue++;
        }
        P4IFG &=~ BIT5;
     }
    if(pos_roue > 3600)  pos_roue = pos_roue - 3600;
    if(pos_roue <    0)  pos_roue = pos_roue + 3600;
    long temps = TA0R; // valeur du timerA
    vitesse_roue = (pos_roue-prev_pos_roue)*(2*3.14/3600)/(temps - prev_tic_roue); // mutiplier par les caractéristiques de timerA
    pos_roue = pos_roue*2*3.14/3600;
    prev_tic_roue = TA0R;
    prev_pos_roue = pos_roue;
}
