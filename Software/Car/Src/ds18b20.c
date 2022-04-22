#include "ds18b20.h"
#include <string.h>

/*
 * USART2(Tx=PA2 Rx=PA3)
 */

typedef struct {
  __IO uint8_t Reset;              //Communication Phase 1: Reset
  __IO uint8_t ROM_Command;        //Communication Phase 2: Rom command
  __IO uint8_t Function_Command;   //Communication Phase 3: DS18B20 function command
  __IO uint8_t *ROM_TxBuffer;
  __IO uint8_t *ROM_RxBuffer;
  __IO uint8_t ROM_TxCount;
  __IO uint8_t ROM_RxCount;
  __IO uint8_t *Function_TxBuffer;
  __IO uint8_t *Function_RxBuffer;
  __IO uint8_t Function_TxCount;
  __IO uint8_t Function_RxCount;
  __IO uint8_t ROM;
  __IO uint8_t Function;
} State;

State state;
uint8_t internal_Buffer[73];

typedef struct {
  void(*OnComplete)(void);
  void(*OnErr)(void);
}OneWire_Callback;

__IO OneWire_Callback onewire_callback;

void OneWire_SetCallback(void(*OnComplete)(void), void(*OnErr)(void))
{
  onewire_callback.OnErr = OnErr;
  onewire_callback.OnComplete = OnComplete;
}
void OneWire_Init(){
  OneWire_UARTInit(9600);
}

// Declare a USART_HandleTypeDef handle structure. 
void OneWire_UARTInit(uint32_t baudRate){
  // HAL_UART_DeInit(&OW_UART);
  OW_UART.Instance = USART2;
  OW_UART.Init.BaudRate = baudRate;
  OW_UART.Init.WordLength = UART_WORDLENGTH_8B;
  OW_UART.Init.StopBits = UART_STOPBITS_1;
  OW_UART.Init.Parity = UART_PARITY_NONE;
  OW_UART.Init.Mode = UART_MODE_TX_RX;
  OW_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  OW_UART.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&OW_UART);
  return ;
}

void OneWire_TxCpltCallback(){
}
 
void OneWire_RxCpltCallback(){
  StateMachine();
}
 /* OneWire_SendBytes & OneWire_ReadBytes */
 
void StateMachine(){
  switch (state.Reset){
    case 0: // start the reset produce;
      OneWire_UARTInit(9600);
      internal_Buffer[0]=0xf0;
      HAL_UART_Transmit_DMA(&OW_UART,&(internal_Buffer[0]),1);
      HAL_UART_Receive_DMA(&OW_UART,&(internal_Buffer[0]),1);
      state.Reset++;
      break;
    case 1: // to check if the device exist or not.
      if (internal_Buffer[0]==0xf0)
        {
        onewire_callback.OnErr();
          break;
        }
      state.Reset++;
    case 2:
      if (ROMStateMachine()==0)
        state.Reset++;
      else break;
    case 3:
      if (FunctionStateMachine()==0)
        state.Reset++;
      else break;
    case 4:
      if (state.Function_Command == 0xbe) {
        onewire_callback.OnComplete();
      }
      break;
  }
  return;
}

uint8_t ROMStateMachine(void){
  switch(state.ROM){
    case 0: // start the ROM command by sending the ROM_Command
      OneWire_UARTInit(115200);
      for (uint8_t i=0;i<8;i++)
        internal_Buffer[i]=((state.ROM_Command>>i)&0x01)?0xff:0x00;
      HAL_UART_Transmit_DMA(&OW_UART,&(internal_Buffer[0]),8);
      HAL_UART_Receive_DMA(&OW_UART,&(internal_Buffer[0]),8);
      state.ROM++;
      break;
    case 1: // continue by sending necessary Tx buffer
      if (state.ROM_TxCount!=0){
        for (uint8_t i=0;i<state.ROM_TxCount;i++)
          for (uint8_t j=0;j<8;j++)
            internal_Buffer[i*8+j]=((state.ROM_TxBuffer[i]>>j)&0x01)?0xff:0x00;
        HAL_UART_Transmit_DMA(&OW_UART,&(internal_Buffer[0]),state.ROM_TxCount*8);
        HAL_UART_Receive_DMA(&OW_UART,&(internal_Buffer[0]),state.ROM_TxCount*8);
        state.ROM++;
        break;              
      }
      if (state.ROM_RxCount!=0){
        for (uint8_t i=0;i<=state.ROM_RxCount*8;i++)
          internal_Buffer[i]=0xff;
        HAL_UART_Transmit_DMA(&OW_UART,&(internal_Buffer[0]),state.ROM_RxCount*8);
        HAL_UART_Receive_DMA(&OW_UART,&(internal_Buffer[0]),state.ROM_RxCount*8);
        state.ROM++;
        break;
      } 
      state.ROM++;
    case 2: 
      if (state.ROM_RxCount!=0){
        for (uint8_t i=0;i<state.ROM_RxCount;i++)
          for (uint8_t j=0;j<8;j++)
            state.ROM_RxBuffer[i]=(state.ROM_RxBuffer[i])+(((internal_Buffer[i*8+j]==0xff)?0x01:0x00)<<j);
      }
      state.ROM=0;
      break;
  }
  return state.ROM;
}

uint8_t FunctionStateMachine(void){
  switch(state.Function){
    case 0: 
      OneWire_UARTInit(115200);
      for (uint8_t i=0;i<8;i++)
        internal_Buffer[i]=((state.Function_Command>>i)&0x01)?0xff:0x00;
      HAL_UART_Transmit_DMA(&OW_UART,&(internal_Buffer[0]),8);
      HAL_UART_Receive_DMA(&OW_UART,&(internal_Buffer[0]),8);
      state.Function++;
      break;
    case 1: // continue by sending necessary Tx buffer
      if (state.Function_TxCount!=0){
        for (uint8_t i=0;i<state.Function_TxCount;i++)
          for (uint8_t j=0;j<8;j++)
            internal_Buffer[i*8+j]=((state.Function_TxBuffer[i]>>j)&0x01)?0xff:0x00;
        HAL_UART_Transmit_DMA(&OW_UART,&(internal_Buffer[0]),state.Function_TxCount*8);
        HAL_UART_Receive_DMA(&OW_UART,&(internal_Buffer[0]),state.Function_TxCount*8);
        state.Function++;
        break;
      }
      if (state.Function_RxCount!=0){
        for (uint8_t i=0;i<=state.Function_RxCount*8;i++)
          internal_Buffer[i]=0xff;
        HAL_UART_Transmit_DMA(&OW_UART,&(internal_Buffer[0]),state.Function_RxCount*8);
        HAL_UART_Receive_DMA(&OW_UART,&(internal_Buffer[0]),state.Function_RxCount*8);
        state.Function++;
        break;
      }
      state.Function++;
    case 2: 
      if (state.Function_RxCount!=0){
        for (uint8_t i=0;i<state.Function_RxCount;i++)
          for (uint8_t j=0;j<8;j++)
        state.Function_RxBuffer[i]=state.Function_RxBuffer[i]+(((internal_Buffer[i*8+j]==0xff)?0x01:0x00)<<j);
      }
      state.Function=0;
      break;
  }
  return state.Function;
}

void OneWire_Execute(uint8_t ROM_Command,uint8_t* ROM_Buffer,uint8_t Function_Command,uint8_t* Function_buffer){
  memset(&(state),0,sizeof(State));
  state.ROM_Command=ROM_Command;
  state.Function_Command=Function_Command;
  switch (ROM_Command){
    case 0x33:  // Read ROM
      state.ROM_RxBuffer=ROM_Buffer;
      state.ROM_RxCount=8; //8 byte
      break;
    case 0x55:  // Match ROM
      state.ROM_TxBuffer=ROM_Buffer;
      state.ROM_TxCount=8; 
      break;
    case 0xf0: break; // Search ROM it might be too hard to implement you might need to refer to Chapter "C.3. Search ROM Command" in the pdf here:http://pdfserv.maximintegrated.com/en/an/AN937.pdf
    case 0xec: break; // Alarm Search it might be too hard to implement refer to http://pdfserv.maximintegrated.com/en/an/AN937.pdf if in need.
    case 0xcc: break; // Skip Rom just send the 0xcc only since the code is implement one-slave need.  
  }
  switch (Function_Command){
    case 0x44: break; // Convert T need to transmit nothing or we can read a 0 while the temperature is in progress read a 1 while the temperature is done.
    case 0x4e:  // Write Scratchpad
      state.Function_TxBuffer=Function_buffer;
      state.Function_TxCount=3; 
      break;
    case 0x48: break; // Copy Scratchpad need to transmit nothing
    case 0xbe:  // Read Scratchpad
      state.Function_RxBuffer=Function_buffer;
      state.Function_RxCount=9; 
      break;
    case 0xb8: break; // Recall EEPROM return transmit status to master 0 for in progress and 1 is for done.
    case 0xb4: break; // read power supply only work for undetermined power supply status. so don't need to implement it 
  }
  StateMachine();
}
