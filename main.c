#include <xc.h>

uint8_t clockCounter = 0;
uint8_t digitNum = 0;
uint8_t customCharCount = 0;
uint8_t ledPositionColumn = 0;
uint8_t ledPositionRow = 0;

uint8_t isRE0Pressed=0;
uint8_t isRE1Pressed=0;
uint8_t isRE2Pressed=0;
uint8_t isRE3Pressed=0;
uint8_t isRE4Pressed=0;
uint8_t isRE5Pressed=0;

uint8_t cursorIndex = 0;


void Pulse(void){
    PORTBbits.RB5 = 1;
    __delay_us(30);
    PORTBbits.RB5 = 0;
    __delay_us(30);
}

void SendBusContents(uint8_t data){
  PORTD = PORTD & 0x0F;           // Clear bus
  PORTD = PORTD | (data&0xF0);     // Put high 4 bits
  Pulse();                        
  PORTD = PORTD & 0x0F;           // Clear bus
  PORTD = PORTD | ((data<<4)&0xF0);// Put low 4 bits
  Pulse();
}
uint8_t customMap[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
};

uint8_t lcdRow[16] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
};

int characterIndexArray[16]={
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

volatile char PREDEFINED[] = " abcdefghijklmnopqrstuvwxyz0123456789";

void __interrupt(high_priority) highPriorityISR(void) {
    if (INTCONbits.TMR0IF) tmr0_isr();
    if (PIR1bits.ADIF) adc_isr();
    
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

void initPorts(){
     // Set ADC Inputs
  TRISH = 0x10; // AN12 input RH4
  TRISJ = 0x00;
  // Set LCD Outputs
  TRISB = 0x00; // LCD Control RB2/RB5
  TRISD = 0x00; // LCD Data  RD[4-7]
  // Configure ADC
  ADCON0 = 0x31; // Channel 12; Turn on AD Converter
  ADCON1 = 0x00; // All analog pins
  ADCON2 = 0xAA; // Right Align | 12 Tad | Fosc/32
  ADRESH = 0x00;
  ADRESL = 0x00;
  TRISA = 0x00;
  TRISB = 0x00;
  TRISC = 0x00;
  TRISD = 0x00;
  TRISE = 0x1F;
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

void adc_isr() {
    PIR1bits.ADIF = 0; 

    unsigned int result = (ADRESH << 8) + ADRESL;
    
    cursorIndex = result / 64; 
    PORTBbits.RB2 = 0;
    SendBusContents(0x80|(cursorIndex & 0xff));
    
    
}

void init_interrupt(){
    INTCONbits.TMR0IE = 1;
    PIE1bits.ADIE = 1;
    INTCONbits.GIE = 1;
}

uint8_t checkRE0Pressed(){
    
    if(isRE0Pressed && !PORTEbits.RE0){
        isRE0Pressed = 0;
        return 1;
    }
    if(PORTEbits.RE0){
        isRE0Pressed = 1;
    }
    return 0;
}
uint8_t checkRE1Pressed(){
    
    if(isRE1Pressed && !PORTEbits.RE1){
        isRE1Pressed = 0;
        return 1;
    }
    if(PORTEbits.RE1){
        isRE1Pressed = 1;
    }
    return 0;
}
uint8_t checkRE2Pressed(){
    
    if(isRE2Pressed && !PORTEbits.RE2){
        isRE2Pressed = 0;
        return 1;
    }
    if(PORTEbits.RE2){
        isRE2Pressed = 1;
    }
    return 0;
}
uint8_t checkRE3Pressed(){
    
    if(isRE3Pressed && !PORTEbits.RE3){
        isRE3Pressed = 0;
        return 1;
    }
    if(PORTEbits.RE3){
        isRE3Pressed = 1;
    }
    return 0;
}
uint8_t checkRE4Pressed(){
    
    if(isRE4Pressed && !PORTEbits.RE4){
        isRE4Pressed = 0;
        return 1;
    }
    if(PORTEbits.RE4){
        isRE4Pressed = 1;
    }
    return 0;
}
uint8_t checkRE5Pressed(){
    
    if(isRE5Pressed && !PORTEbits.RE5){
        isRE5Pressed = 0;
        return 1;
    }
    if(PORTEbits.RE5){
        isRE5Pressed = 1;
    }
    return 0;
}


void scrollInPredefinedChars(int backwardOrForward){ //0 if bacward, 1 if forward
    PORTBbits.RB2 = 1;
    if(backwardOrForward == 0){
        if(characterIndexArray[cursorIndex]==0){
            characterIndexArray[cursorIndex]=36;
        }
        characterIndexArray[cursorIndex] -- ;
    }else if(backwardOrForward == 1){
        if(characterIndexArray[cursorIndex]==36){
            characterIndexArray[cursorIndex]=0;
        }
        characterIndexArray[cursorIndex]++; 
    }
    
    SendBusContents(PREDEFINED[characterIndexArray[cursorIndex]]);
}

void scrollInCustomChars(int backwardOrForward){ //0 if bacward, 1 if forward
    
}


void main(void) {
    
    initPorts();
    init_interrupt();
    InitLCD();
    __delay_ms(30);
    // text entry mode
    // ADC position set
    // RE2 forward cycle
    // RE1 backward cycle
    // RE0 forward cycle custom
    // RE3 backward cycle custom
    
    // if RE4 pressed character define mode
    // if RE5 pressed text scroll mode
    
    int mode = 0; //0 -> text mode, 1 -> custom mode, 2 -> scroll mode
    while(1){
        if(mode == 0){ // TEM
            
            if(checkRE0Pressed()){
                
            }
            if(checkRE1Pressed()){
                scrollInPredefinedChars(0);
            }
            if(checkRE2Pressed()){
                scrollInPredefinedChars(1);
            }
            if(checkRE3Pressed()){
                
            }
            if(checkRE4Pressed()){
                mode = 1;
            }
            if(checkRE5Pressed()){
                mode = 2;
            }
            
            
        }else if(mode == 1){ //CCD
            
        }else if(mode == 2){ // SCROLL
            
        }
    }
    
    
    return;
}
