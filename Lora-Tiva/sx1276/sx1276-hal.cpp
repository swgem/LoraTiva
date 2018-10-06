/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2014 Semtech

Description: -

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/
#include "sx1276-hal.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

//definicao de pinos

//mosi - SSI2TX - PB7
//miso - SSI2RX - PB6
//sclk - SSI2CLK - PB4
//nss - PE0
//reset - PE4
//dio0 - PC4
//dio1 - PC5
//dio2 - PC6
//dio3 - PC7
//dio4 - PD6
//dio5 - PD7
//                            antSwitch( antSwitch )



const RadioRegisters_t SX1276MB1xAS::RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

//ponteiros de funcao das interrupcoes de io
void (*g_irq_hdl[5])();

SX1276MB1xAS::SX1276MB1xAS( RadioEvents_t *events/*,
                            PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                            PinName dio0, PinName dio1, PinName dio2, PinName dio3, PinName dio4, PinName dio5,
                            PinName antSwitch*/ )
                            : SX1276( events/*, mosi, miso, sclk, nss, reset, dio0, dio1, dio2, dio3, dio4, dio5 ),
                            antSwitch( antSwitch ),
                            fake( A3 )*/)

{
    this->RadioEvents = events;

    boardConnected =  SX1276MB1LAS;

    //liga o clock da porta E (necessário para o reset)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);

    SpiInit();

    Reset( );
    
    RxChainCalibration( );
    
    IoInit( );
    
    SetOpMode( RF_OPMODE_SLEEP );
    
    IoIrqInit( dioIrq );
    
    RadioRegistersInit( );

    SetModem( MODEM_FSK );

    this->settings.State = RF_IDLE ;
}

//SX1276MB1xAS::SX1276MB1xAS( RadioEvents_t *events ) :   SX1276( events/*, D11, D12, D13, D10, A0, D2, D3, D4, D5, D8, D9 ), antSwitch( A4 ), fake( A3 )*/)
//{
//    this->RadioEvents = events;
//
//    Reset( );
//
//    boardConnected = UNKNOWN;
//
//    DetectBoardType( );
//
//    RxChainCalibration( );
//
//    IoInit( );
//
//    SetOpMode( RF_OPMODE_SLEEP );
//    IoIrqInit( dioIrq );
//
//    RadioRegistersInit( );
//
//    SetModem( MODEM_FSK );
//
//    this->settings.State = RF_IDLE ;
//}

//-------------------------------------------------------------------------
//                      Board relative functions
//-------------------------------------------------------------------------
uint8_t SX1276MB1xAS::DetectBoardType( void )
{
    if( boardConnected == UNKNOWN )
    {
//        antSwitch.input( );
//        wait_ms( 1 );
//        if( antSwitch == 1 )
//        {
            boardConnected = SX1276MB1LAS;
//        }
//        else
//        {
//			boardConnected = SX1276MB1MAS;
//        }
//        antSwitch.output( );
//        wait_ms( 1 );
    }
    return ( boardConnected );
}

void SX1276MB1xAS::IoInit( void )
{
    //AntSwInit( );

	//reset - PE4
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//
	// Enable the GPIO pin for SSEL
	//
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);



}

void SX1276MB1xAS::RadioRegistersInit( )
{
    uint8_t i = 0;
    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SetModem( RadioRegsInit[i].Modem );
        Write( RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }    
}

void SX1276MB1xAS::SpiInit( void )
{
	//mosi - SSI2TX - PB7
	//miso - SSI2RX - PB6
	//sclk - SSI2CLK - PB4
	//nss - PE0


	//nss = 1;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//
	// Enable the GPIO pin for SSEL
	//
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);


    //spi.format( 8,0 );
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

	GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);
    uint32_t frequencyToSet = 8000000;
    SSIConfigSetExpClk(SSI2_BASE/**/,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,500000,8);

    //SSIConfigSetExpClk(SSI2_BASE/**/,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, 2000000, 8);

    SSIEnable(SSI2_BASE);


    //debug ssi
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_3|GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 8000000, 16);
    SSIEnable(SSI0_BASE);



    //#if( defined ( TARGET_NUCLEO_L152RE ) ||  defined ( TARGET_LPC11U6X ) )
    //    spi.frequency( frequencyToSet );
    //#elif( defined ( TARGET_KL25Z ) ) //busclock frequency is halved -> double the spi frequency to compensate
    //    spi.frequency( frequencyToSet * 2 );
    //#else
    //    #warning "Check the board's SPI frequency"
    //#endif
    //wait(0.1);
    SysCtlDelay(SysCtlClockGet() / 10);
}

void portc_handler(void)
{
	uint32_t status = GPIOIntStatus(GPIO_PORTC_BASE,true);

	if(status & GPIO_INT_PIN_4)
	{
		(*g_irq_hdl[0])();
	}


	if(status & GPIO_INT_PIN_5)
	{
		(*g_irq_hdl[1])();
	}


	if(status & GPIO_INT_PIN_6)
	{
		(*g_irq_hdl[2])();
	}


	if(status & GPIO_INT_PIN_7)
	{
		(*g_irq_hdl[3])();
	}

	GPIOIntClear(GPIO_PORTC_BASE,GPIO_INT_PIN_4 | GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
}

void portd_handler()
{
	uint32_t status = GPIOIntStatus(GPIO_PORTD_BASE,true);

	if(status & GPIO_INT_PIN_6)
	{
		(*g_irq_hdl[4])();
	}

	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_6);

}

void SX1276MB1xAS::IoIrqInit( void (**irqHandlers)() )
{
//#if( defined ( TARGET_NUCLEO_L152RE ) ||  defined ( TARGET_LPC11U6X ) )
//    dio0.mode(PullDown);
//    dio1.mode(PullDown);
//    dio2.mode(PullDown);
//    dio3.mode(PullDown);
//    dio4.mode(PullDown);
//#endif
//    dio0.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[0] ) );
//    dio1.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[1] ) );
//    dio2.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[2] ) );
//    dio3.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[3] ) );
//    dio4.rise( this, static_cast< TriggerMB1xAS > ( irqHandlers[4] ) );

	//dio0 - PC4
	//dio1 - PC5
	//dio2 - PC6
	//dio3 - PC7
	//dio4 - PD6

	int i;

	for(i = 0;i<5;i++)
	{
		g_irq_hdl[i] = irqHandlers[i];
	}

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_RISING_EDGE);
	GPIOIntDisable(GPIO_PORTC_BASE,GPIO_INT_PIN_4 | GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
	GPIOIntClear(GPIO_PORTC_BASE,GPIO_INT_PIN_4 | GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);
	GPIOIntRegister(GPIO_PORTC_BASE, portc_handler);
	GPIOIntEnable(GPIO_PORTC_BASE,GPIO_INT_PIN_4 | GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7);


	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_6 );
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6 , GPIO_RISING_EDGE);
	GPIOIntDisable(GPIO_PORTD_BASE,GPIO_INT_PIN_6);
	GPIOIntClear(GPIO_PORTD_BASE,GPIO_INT_PIN_6);
	GPIOIntRegister(GPIO_PORTD_BASE, portd_handler);
	GPIOIntEnable(GPIO_PORTD_BASE,GPIO_INT_PIN_6);


}

void SX1276MB1xAS::IoDeInit( void )
{
    //nothing
}

uint8_t SX1276MB1xAS::GetPaSelect( uint32_t channel )
{
    if( channel > RF_MID_BAND_THRESH )
    {
        if( boardConnected == SX1276MB1LAS )
        {
            return RF_PACONFIG_PASELECT_PABOOST;
        }
        else
        {
            return RF_PACONFIG_PASELECT_RFO;
        }
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

//void SX1276MB1xAS::SetAntSwLowPower( bool status )
//{
//    if( isRadioActive != status )
//    {
//        isRadioActive = status;
//
//        if( status == false )
//        {
//            AntSwInit( );
//        }
//        else
//        {
//            AntSwDeInit( );
//        }
//    }
//}

//void SX1276MB1xAS::AntSwInit( void )
//{
//    antSwitch = 0;
//}

//void SX1276MB1xAS::AntSwDeInit( void )
//{
//    antSwitch = 0;
//}

//void SX1276MB1xAS::SetAntSw( uint8_t rxTx )
//{
//    if( this->rxTx == rxTx )
//    {
//        //no need to go further
//        return;
//    }
//
//    this->rxTx = rxTx;
//
//    if( rxTx != 0 )
//    {
//        antSwitch = 1;
//    }
//    else
//    {
//        antSwitch = 0;
//    }
//}

bool SX1276MB1xAS::CheckRfFrequency( uint32_t frequency )
{
    //TODO: Implement check, currently all frequencies are supported
    return true;
}


void SX1276MB1xAS::Reset( void )
{
    //reset.output();
    //reset = 0;
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);
    SysCtlDelay(SysCtlClockGet() / 1000);
    //reset.input();
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
    SysCtlDelay(6 * (SysCtlClockGet() / 1000));
}
    
void SX1276MB1xAS::Write( uint8_t addr, uint8_t data )
{
    Write( addr, &data, 1 );
}

uint8_t SX1276MB1xAS::Read( uint8_t addr )
{
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
}

void SX1276MB1xAS::Write( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    uint32_t dummy;

    //nss = 0;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);

    //spi.write( addr | 0x80 );
    SSIDataPut(SSI2_BASE/**/, addr | 0x80);
    while(SSIBusy(SSI2_BASE/**/));
    SSIDataGet(SSI2_BASE/**/,&dummy); //dummy read to empty fifo

    for( i = 0; i < size; i++ )
    {
        //spi.write( buffer[i] );
    	SSIDataPut(SSI2_BASE/**/, buffer[i]);
    	while(SSIBusy(SSI2_BASE/**/));
    	SSIDataGet(SSI2_BASE/**/,&dummy); //dummy read to empty fifo
    }
    //nss = 1;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

void SX1276MB1xAS::Read( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    uint32_t dummy;
    //uint32_t* SSI2_RIS = (uint32_t*)(SSI2_BASE/**/ + 0x18);

    //nss = 0;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);


    //spi.write( addr & 0x7F );
    SSIDataPut(SSI2_BASE/**/, addr & 0x7F);
    while(SSIBusy(SSI2_BASE/**/));
    SSIDataGet(SSI2_BASE/**/,&dummy); //dummy read to empty fifo


    for( i = 0; i < size; i++ )
    {
    	SSIDataPut(SSI2_BASE/**/, 0x00);
    	while(SSIBusy(SSI2_BASE/**/));
    	SSIDataGet(SSI2_BASE/**/,&dummy);
    	buffer[i] = (uint8_t)(dummy&0xFF);
    	//buffer[i] = spi.write( 0 );
    }

    //nss = 1;
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

void SX1276MB1xAS::WriteFifo( uint8_t *buffer, uint8_t size )
{
    Write( 0, buffer, size );
}

void SX1276MB1xAS::ReadFifo( uint8_t *buffer, uint8_t size )
{
    Read( 0, buffer, size );
}
