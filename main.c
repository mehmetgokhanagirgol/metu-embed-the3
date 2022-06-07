/*
 * We are using Timer 0 with 1:256 prescaler for seven segment display and
 * scroll mode.
 */

#include <xc.h>
#include "Includes.h"
#include "lcd.h"
#include <stdio.h>

uint8_t clockCounter1 = 0;
uint8_t clockCounter2 = 0;
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

int mode = 0; //0 -> text mode, 1 -> custom mode, 2 -> scroll mode

// Global value for cursor position in CDM
// TODO: This should be drawn constantly on seven segment display with customCharCount
int cursorPosition[2] = {0, 0};


void tmr0_isr();
void adc_isr();
void scrollText();
void shiftCharArray();

void __interrupt(high_priority) highPriorityISR(void) {
    if (INTCONbits.TMR0IF) tmr0_isr();
    if (PIR1bits.ADIF) adc_isr();
    
}
void __interrupt(low_priority) lowPriorityISR(void) {}

void SendBusContents(uint8_t data){
  PORTD = PORTD & 0x0F;           // Clear bus
  PORTD = PORTD | (data&0xF0);     // Put high 4 bits
  Pulse();                        
  PORTD = PORTD & 0x0F;           // Clear bus
  PORTD = PORTD | ((data<<4)&0xF0);// Put low 4 bits
  Pulse();
}
uint8_t customMap[8][8];

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

int customCharacterIndexArray[16]={
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

//volatile char PREDEFINED[] = " abcdefghijklmnopqrstuvwxyz0123456789";

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
    sevenSegment();
    if(mode == 2){
        clockCounter2++;
        if(clockCounter2 == 76){
            clockCounter2 = 0;
            scrollText();
        }
    }
}

void adc_isr() {
    
    PIR1bits.ADIF = 0; 
    unsigned int result = (ADRESH << 8) + ADRESL;
    if(mode!=2){
        cursorIndex = result / 64; 

    }
    
    GODONE=1;
}



void tmr_init() {
    T0CON = 0xC7;
    //tmr_preload();
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
  TRISE = 0x3F;
  
  // Clear all ports to use in CDM
  PORTA = 0x00;
  PORTB = 0x00;
  PORTC = 0x00;
  PORTD = 0x00;
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
    
    if(backwardOrForward == 0){
        if(characterIndexArray[cursorIndex]==0){
            characterIndexArray[cursorIndex]=37;
        }
        characterIndexArray[cursorIndex] -- ;
    }else if(backwardOrForward == 1){
        if(characterIndexArray[cursorIndex]==36){
            characterIndexArray[cursorIndex]=-1;
        }
        characterIndexArray[cursorIndex]++; 
    }
    PORTBbits.RB2 = 1;
    
    SendBusContents(PREDEFINED[characterIndexArray[cursorIndex]]);
}

void scrollInCustomChars(int backwardOrForward){ //0 if backward, 1 if forward
    
    if(backwardOrForward == 0){
        if(customCharacterIndexArray[cursorIndex]==0){
            customCharacterIndexArray[cursorIndex]=customCharCount+1;
        }
        customCharacterIndexArray[cursorIndex] -- ;
    }else if(backwardOrForward == 1){
        
        if(customCharacterIndexArray[cursorIndex]==customCharCount){
            customCharacterIndexArray[cursorIndex]=-1;
        }
        customCharacterIndexArray[cursorIndex]++; 
    }
    PORTBbits.RB2 = 1;
    if(customCharacterIndexArray[cursorIndex] == 0){
        SendBusContents(PREDEFINED[0]);
    } else {
        SendBusContents(customCharacterIndexArray[cursorIndex]-1);
    }

}


// Function to toggle led state of a bit of PORT A
void ledToggleA(int bitNumber) {
    switch(bitNumber) {
        case 0:
            LATAbits.LA0 = ~LATAbits.LA0;
            break;
        case 1:
            LATAbits.LA1 = ~LATAbits.LA1;
            break;
        case 2:
            LATAbits.LA2 = ~LATAbits.LA2;
            break;
        case 3:
            LATAbits.LA3 = ~LATAbits.LA3;
            break;
        case 4:
            LATAbits.LA4 = ~LATAbits.LA4;
            break;
        case 5:
            LATAbits.LA5 = ~LATAbits.LA5;
            break;
        case 6:
            // This bit's state won't change!
            LATAbits.LA6 = ~LATAbits.LA6;
            break;
        case 7:
            // This bit's state won't change!
            LATAbits.LA7 = ~LATAbits.LA7;
            break;
        default:
            break;
    }
}


// Function to toggle led state of a bit of PORT B
void ledToggleB(int bitNumber) {
    switch(bitNumber) {
        case 0:
            PORTBbits.RB0 = ~PORTBbits.RB0;
            break;
        case 1:
            PORTBbits.RB1 = ~PORTBbits.RB1;
            break;
        case 2:
            PORTBbits.RB2 = ~PORTBbits.RB2;
            break;
        case 3:
            PORTBbits.RB3 = ~PORTBbits.RB3;
            break;
        case 4:
            PORTBbits.RB4 = ~PORTBbits.RB4;
            break;
        case 5:
            PORTBbits.RB5 = ~PORTBbits.RB5;
            break;
        case 6:
            PORTBbits.RB6 = ~PORTBbits.RB6;
            break;
        case 7:
            PORTBbits.RB7 = ~PORTBbits.RB7;
            break;
        default:
            break;
    }
}


// Function to toggle led state of a bit of PORT C
void ledToggleC(int bitNumber) {
    switch(bitNumber) {
        case 0:
            PORTCbits.RC0 = ~PORTCbits.RC0;
            break;
        case 1:
            PORTCbits.RC1 = ~PORTCbits.RC1;
            break;
        case 2:
            PORTCbits.RC2 = ~PORTCbits.RC2;
            break;
        case 3:
            PORTCbits.RC3 = ~PORTCbits.RC3;
            break;
        case 4:
            PORTCbits.RC4 = ~PORTCbits.RC4;
            break;
        case 5:
            PORTCbits.RC5 = ~PORTCbits.RC5;
            break;
        case 6:
            PORTCbits.RC6 = ~PORTCbits.RC6;
            break;
        case 7:
            PORTCbits.RC7 = ~PORTCbits.RC7;
            break;
        default:
            break;
    }
}


// Function to toggle led state of a bit of PORT D
void ledToggleD(int bitNumber) {
    switch(bitNumber) {
        case 0:
            PORTDbits.RD0 = ~PORTDbits.RD0;
            break;
        case 1:
            PORTDbits.RD1 = ~PORTDbits.RD1;
            break;
        case 2:
            PORTDbits.RD2 = ~PORTDbits.RD2;
            break;
        case 3:
            PORTDbits.RD3 = ~PORTDbits.RD3;
            break;
        case 4:
            PORTDbits.RD4 = ~PORTDbits.RD4;
            break;
        case 5:
            PORTDbits.RD5 = ~PORTDbits.RD5;
            break;
        case 6:
            PORTDbits.RD6 = ~PORTDbits.RD6;
            break;
        case 7:
            PORTDbits.RD7 = ~PORTDbits.RD7;
            break;
        default:
            break;
    }
}


void customDefineMode() {
        if(checkRE0Pressed()){
            // Move cursor right
            if (cursorPosition[0] < 3) {
                cursorPosition[0]++;
                ledPositionColumn++;
            }
        }
        if(checkRE1Pressed()){
            // Move cursor down
            if (cursorPosition[1] < 7) {
                cursorPosition[1]++;
                ledPositionRow++;
            }
        }
        if(checkRE2Pressed()){
            // Move cursor up
            if (cursorPosition[1] > 0) {
                cursorPosition[1]--;
                ledPositionRow--;
            }
        }
        if(checkRE3Pressed()){
            // Move cursor left
            if (cursorPosition[0] > 0) {
                cursorPosition[0]--;
                ledPositionColumn--;
            }
        }
        if(checkRE4Pressed()){
            // Toggle LED state
            // cursorPosition[0]: PORTA, cursorPosition[1]: PORTB,
            // cursorPosition[2]: PORTC, cursorPosition[3]: PORTD
            if (cursorPosition[0] == 0) {
                // Cursor is on PORT A
                ledToggleA(cursorPosition[1]);

            } else if (cursorPosition[0] == 1) {
                // Cursor is on PORT B
                ledToggleB(cursorPosition[1]);

            } else if (cursorPosition[0] == 2) {
                // Cursor is on PORT C
                ledToggleC(cursorPosition[1]);

            } else if (cursorPosition[0] == 3) {
                // Cursor is on PORT D
                ledToggleD(cursorPosition[1]);

            }
        }
        if(checkRE5Pressed()){
            // Reset the cursor position
            cursorPosition[0] = 0;
            cursorPosition[1] = 0;
            // Save the custom character to the custom character array
            customMap[customCharCount][0] = ((LATA & 0x01) << 4) + ((PORTB & 0x01) << 3) + ((PORTC & 0x01) << 2) + ((PORTD & 0x01) << 1);
            customMap[customCharCount][1] = ((LATA & 0x02) << 3) + ((PORTB & 0x02) << 2) + ((PORTC & 0x02) << 1) + ((PORTD & 0x02));
            customMap[customCharCount][2] = ((LATA & 0x04) << 2) + ((PORTB & 0x04) << 1) + ((PORTC & 0x04)) + ((PORTD & 0x04) >> 1);
            customMap[customCharCount][3] = ((LATA & 0x08) << 1) + ((PORTB & 0x08)) + ((PORTC & 0x08) >> 1) + ((PORTD & 0x08) >> 2);
            customMap[customCharCount][4] = ((LATA & 0x10)) + ((PORTB & 0x10) >> 1) + ((PORTC & 0x10) >> 2) + ((PORTD & 0x10) >> 3);
            customMap[customCharCount][5] = ((LATA & 0x20) >> 1) + ((PORTB & 0x20) >> 2) + ((PORTC & 0x20) >> 3) + ((PORTD & 0x20) >> 4);
            customMap[customCharCount][6] = ((LATA & 0x40) >> 2) + ((PORTB & 0x40) >> 3) + ((PORTC & 0x40) >> 4) + ((PORTD & 0x40) >> 5);
            customMap[customCharCount][7] = ((LATA & 0x80) >> 3) + ((PORTB & 0x80) >> 4) + ((PORTC & 0x80) >> 5) + ((PORTD & 0x80) >> 6);

            ledPositionRow = 0;
            ledPositionColumn = 0;
            // TODO: Print the created custom character on LCD screen
            // TODO: If implemented, copy values back to the ports
            // Go to TEM
            mode = 0;
            
            uint8_t charmap[8] = {
              0x00,
              0x00,
              0x0A,
              0x1F,
              0x1F,
              0x0E,
              0x04,
              0x00,
            };

            // Define custom char LCD
            // Set CGRAM address 0 -> 0x40
            PORTBbits.RB2 = 0;
            SendBusContents(0x40 |(customCharCount << 3));

            // Start sending charmap
            for(int i=0; i<8; i++){
              PORTBbits.RB2 = 1; // Send Data
              SendBusContents(customMap[customCharCount][i]);
            }
            SendBusContents(0x02);
            PORTBbits.RB2 = 1;
            
            // Set DDRAM address to 0 (line 1 cell 1) -> 0x80
            PORTBbits.RB2 = 0;
            SendBusContents(0x80|(cursorIndex & 0xff));
            
            //unsigned int result = (ADRESH << 8) + ADRESL; // Get the result;

            // 4bytes for ADC Res + 1 byte for custom char + 1 byte null;
//            char buf[2];
//            sprintf(buf, "%04u", result);
//            buf[0]=0; // Address of custom char
//            buf[1]=0; // Null terminator

            // Write buf to LCD DDRAM
            PORTBbits.RB2 = 1;
            SendBusContents(customCharCount);
            
            // Increment custom character count by 1(max 8)
            customCharCount++;
            customCharacterIndexArray[cursorIndex] = customCharCount;
//            ADCON0 = 0x31; // Channel 12; Turn on AD Converter
//            ADCON1 = 0x00;
        }
}

void shiftCharArray(){
    int temp=characterIndexArray[0];
    for(int i=0;i<15;i++){
        characterIndexArray[i]=characterIndexArray[i+1];
    }
    characterIndexArray[15]=temp;
}

void scrollText(){
    
    //INTCONbits.GIE = 0;
    //shiftCharArray();
    // Write "finished"
    PORTBbits.RB2 = 0;
    SendBusContents(0x84);
    PORTBbits.RB2 = 1;
    SendBusContents(PREDEFINED[6]);
    PORTBbits.RB2 = 0;
    SendBusContents(0x85);
    PORTBbits.RB2 = 1;
    SendBusContents(PREDEFINED[9]);
    PORTBbits.RB2 = 0;
    SendBusContents(0x86);
    PORTBbits.RB2 = 1;
    SendBusContents(PREDEFINED[14]);
    PORTBbits.RB2 = 0;
    SendBusContents(0x87);
    PORTBbits.RB2 = 1;
    SendBusContents(PREDEFINED[9]);
    PORTBbits.RB2 = 0;
    SendBusContents(0x88);
    PORTBbits.RB2 = 1;
    SendBusContents(PREDEFINED[19]);
    PORTBbits.RB2 = 0;
    SendBusContents(0x89);
    PORTBbits.RB2 = 1;
    SendBusContents(PREDEFINED[8]);
    PORTBbits.RB2 = 0;
    SendBusContents(0x8a);
    PORTBbits.RB2 = 1;
    SendBusContents(PREDEFINED[5]);
    PORTBbits.RB2 = 0;
    SendBusContents(0x8b);
    PORTBbits.RB2 = 1;
    SendBusContents(PREDEFINED[4]);
    PORTBbits.RB2 = 0;
    SendBusContents(0x0C);
    
    // Shift character array
    int temp=characterIndexArray[0];
    for(int i=0;i<15;i++){
        characterIndexArray[i]=characterIndexArray[i+1];
    }
    characterIndexArray[15]=temp;

    // Shift custom character array
    temp=customCharacterIndexArray[0];
    for(int i=0;i<15;i++){
        customCharacterIndexArray[i]=customCharacterIndexArray[i+1];
    }
    customCharacterIndexArray[15]=temp;
    
    PORTBbits.RB2 = 0;
    
    //SendBusContents(0x01);
    for(int i=0;i<=15;i++){
        PORTBbits.RB2 = 0;
        SendBusContents(0xC0|(15-i & 0xff));
        
        PORTBbits.RB2 = 1;
        // Check if char is custom
        if(customCharacterIndexArray[15-i] == 0) {
            SendBusContents(PREDEFINED[characterIndexArray[15-i]]);
        } else {
            SendBusContents(customCharacterIndexArray[15-i] - 1);
        }

    }
    //INTCONbits.GIE = 1;
}


void main(void) {
    
    initPorts();
    tmr_init();
    init_interrupt();
    InitLCD();
    __delay_ms(30);
    PORTBbits.RB2 = 0;
    SendBusContents(0x2C); // 2LINE
    SendBusContents(0x0C);
    SendBusContents(0x0e); // Display on, cursor off, blink off.
    SendBusContents(0x01);
    GODONE=1;
    
    // text entry mode
    // ADC position set
    // RE2 forward cycle
    // RE1 backward cycle
    // RE0 forward cycle custom
    // RE3 backward cycle custom
    
    // if RE4 pressed character define mode
    // if RE5 pressed text scroll mode
    
    while(1){
        if(mode == 0){ // TEM
            PORTBbits.RB2 = 0;
            SendBusContents(0x80|(cursorIndex & 0xff));
            if(checkRE0Pressed()){
                scrollInCustomChars(1);
            }
            if(checkRE1Pressed()){
                scrollInPredefinedChars(0);
            }
            if(checkRE2Pressed()){
                scrollInPredefinedChars(1);
            }
            if(checkRE3Pressed()){
                scrollInCustomChars(0);
            }
            if(checkRE4Pressed()){
                // Clear all ports to use in CDM
                // Consider copying the values before resetting ports
                PORTA = 0x00;
                PORTB = 0x00;
                PORTC = 0x00;
                PORTD = 0x00;
                mode = 1;
            }
            if(checkRE5Pressed()){
                mode = 2;
                PIE1bits.ADIE = 0;
                GODONE = 0;
                PORTBbits.RB2 = 0;
                SendBusContents(0x01);
                SendBusContents(0x0C); // cursor? durdur??

            }
            
            
        }else if(mode == 1){ //CCD
            //ADCON0=0;
            //ADCON1= 0x06;
            customDefineMode();
            
        }else if(mode == 2){ // SCROLL

        }
    }
    
    
    return;
}