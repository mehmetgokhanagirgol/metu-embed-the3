


#include <xc.h>

uint8_t clockCounter = 0;
uint8_t digitNum = 0;
uint8_t customCharCount = 0;
uint8_t ledPositionColumn = 0;
uint8_t ledPositionRow = 0;

void __interrupt(high_priority) highPriorityISR(void) {
    if (INTCONbits.TMR0IF) tmr0_isr();
    
}
void __interrupt(low_priority) lowPriorityISR(void) {}

uint8_t changeDigit(uint8_t digitNum){
    switch(digitNum){
        case 1:
            PORTHbits.RH0 = 1;
            PORTHbits.RH1 = 0;
            PORTHbits.RH2 = 0;
            PORTHbits.RH3 = 0;
            return customCharCount;
        case 2:
            PORTHbits.RH0 = 0;
            PORTHbits.RH1 = 0;
            PORTHbits.RH2 = 1;
            PORTHbits.RH3 = 0;
            return ledPositionColumn;
        case 3:
            PORTHbits.RH0 = 0;
            PORTHbits.RH1 = 0;
            PORTHbits.RH2 = 0;
            PORTHbits.RH3 = 1;
            return ledPositionRow;
    }
}

void sevenSegment(){
    
    uint8_t value;
    
    // Traverse digits to remove flickering
    digitNum++;
    if(digitNum == 4)
        digitNum = 1;
    value = changeDigit(digitNum);
    

    switch(value){
        case 1:
            LATJ = 0x06;
            break;
        case 2:
            //**
            LATJ = 0x5B;
            break;
        case 3:
        //**
            LATJ = 0x4F;
            break;
        case 4:
        //**
            LATJ = 0x66;
            break;
        case 5:
        //**
            LATJ = 0x6D;
            break;
        case 6:
        //**
            LATJ = 0x7D;
            break;
        case 7:
        //**
            LATJ = 0x07;
            break;
        case 8:
        //**
            LATJ = 0x7F;
            break;
        case 9:
        //**
            LATJ = 0x6F;
            break;
        default:
            LATJ = 0x3F;
            break;
    }
}

void tmr0_isr(){
    INTCONbits.TMR0IF = 0;
    /********************/
    clockCounter ++;
    if(clockCounter % 2 == 0){
        sevenSegment();
    }
    
}

void main(void) {
    return;
}
