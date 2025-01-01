/* C Standard library */
#include <stdio.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

#include <ti/drivers/i2c/I2CCC26XX.h>


/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"

/* Task */
#define STACKSIZE 2048

Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

// **TILAKONEEN ESITTELY**
enum state { WAITING=1, DATA_READY };
enum state programState = WAITING;


// **MUUTTUJAT JA VAKIOT**

Task_Handle uartTaskHandle;   //koodi toimii, kun laittaa nämä tänne main funktion ulkopuolelle.
Task_Params uartTaskParams;
Task_Handle sensorTaskHandle;
Task_Params sensorTaskParams;


// Globaalit muuttujat
bool piste = false;   // false=0, arvo 0 meinaa pineissä low
bool viiva = false;

// Kiihtyvyys ja gyroskooppiarvot
float ax, ay, az, gx, gy, gz;

// Jos kiihtyvyys johonkin suuntaan ylittää jonkun näistä arvoista niin tehdään jotakin
float kynnysAx = 1.1;
float kynnysAy = 1.3;
float kynnysAz = 1.7;

// **NAPIT**

// RTOS:n globaalit muuttujat pinnien käyttöön
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;  u
static PIN_State ledState;

// Pinnien alustukset
PIN_Config buttonConfig[] = {
   Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE, //nappi 1
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE, //nappi 2
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};


// **MPU**

// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

// MPU power pin
static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU uses its own I2C interface (muut sensorit käyttävät toista väylää, meillä kuitenkin vain kiihtyvyssensori käytössä)
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {   // opettajan Githubista löytyi mpu jutut
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};


// **FUNKTIOT**

// Prototyypit
void buttonFxn(PIN_Handle handle, PIN_Id pinId);
void liikkeentunnistus(UART_Handle handle);
void uartTaskFxn(UArg arg0, UArg arg1);
void mpuTaskFxn(UArg arg0, UArg arg1);


void buttonFxn(PIN_Handle handle, PIN_Id pinId) {  // Keskeytyskäsittelijä
    if (pinId == Board_BUTTON1) {  // tarkistaa onko painike 1 painettu
        piste = true;  // Merkitään true, että pisteen painike on painettu
    } else if (pinId == Board_BUTTON0) { // tarkistaa onko painike 2 painettu
        viiva = true; //Merkitään true, että viivan painike on painettu
    }
}

void liikkeentunnistus(UART_Handle handle) {
    /* Vertailee kiihtyvyyttä annettuihin kynnysarvoihin.
    Jos kynnysarvo ylittyy, lähetetään viesti UART:lla.

    Kiihtyvyys z-akselilla suurin -> lähettää välilyönnin
    Kiihtyvyys x-akselilla suurin -> lähettää "sos"
    Kiihtyvyys y-akselilla suurin -> lähettää "hello"*/

    if (ax > ay && ax > az)
    {
        if (ax > kynnysAx)
        {
            UART_write(handle, "sos\r\n", strlen("sos\r\n"));

            // 1,5 s nukkuminen, jotta komennot eivät monistu
            Task_sleep(150000 / Clock_tickPeriod);
        }
    }

    else if (ay > ax && ay > az)
    {
        if (ay > kynnysAy)
        {
            UART_write(handle, "hello\r\n", strlen("hello\r\n"));

            // 1,5 s nukkuminen, jotta komennot eivät monistu
            Task_sleep(150000 / Clock_tickPeriod);
        }
    }

    else if (az > ax && az > ay)
    {
        if (az > kynnysAz)
        {
            UART_write(handle, " \r\n", strlen(" \r\n"));

            // 1,5 s nukkuminen, jotta komennot eivät monistu
            Task_sleep(150000 / Clock_tickPeriod);
        }
    }
}

/* Task Functions */
void uartTaskFxn(UArg arg0, UArg arg1) {

    // UART-kirjaston asetukset
    UART_Handle uart;
    UART_Params uartParams;

    // Alustetaan sarjaliikenne
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode=UART_MODE_BLOCKING;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    // Avataan yhteys laitteen sarjaporttiin vakiossa Board_UART0
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
       System_abort("Error opening the UART");
    }

    while (1) {  // loop

        if (programState == DATA_READY) { // kun tämä toteutuu, niin liikkeentunnistus kutsutaan
            liikkeentunnistus(uart);
            programState = WAITING;
        }

        if (piste) {            // Kirjoittaa pisteen ('.')
            piste = false;    // Nollataan lippu, jotta ei toisteta tapahtumaa
            char str[10];  // str:ään rakennetaan viestijono (10 merkin lista että viesti mahtuu)
            sprintf(str, "%c\r\n", '.'); //  str muuttuja, c kohtaan tulee piste ja rivinvaihto n ja r laittaa kursorin vasempaan reunaan (eli rakennetaan viesti)
            System_printf("%c\n", str);  // printtaa ylläolevan koodin
            System_flush(); // tyhjentää lähetettävän viestin muuten yhdistäisi kaksi viestiä
            UART_write(uart, str, strlen(str)); // uart antaa parametrit, str on lähetettävä viesti ja strlen meinaa viestinpituutta eli 10
                                                // (Lähetetään UART:n kautta)
        }

        if (viiva) {            // Kirjoittaa viivan ('-')
            viiva = false;
            char str[10];
            sprintf(str, "%c\r\n", '-'); //kirjoittaa tekstin merkkijonoon
            System_printf("%c\n", str);
            System_flush();
            UART_write(uart, str, strlen(str));
        }

        // Once per second, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

void mpuTaskFxn(UArg arg0, UArg arg1) {
    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    mpu9250_setup(&i2cMPU);

    while (1) {  //loop
        if (programState == WAITING) {    // programstate waiting tilassa
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);     //kerää dataa sensorilta ja tallentaa arvot globaaleihin muuttujiin
            programState = DATA_READY;     // kun dataa on kerätty tila vaihtuu data_readyksi ja sitten siirrytään uartTaskFxn- funktioon
        }
        Task_sleep(10000 / Clock_tickPeriod);
    }
}


Int main(void) {

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    // Initialize board
    Board_initGeneral();

    // Väylä mukaan ohjelmaan
    Board_initI2C();

    // Ota UART k�ytt��n ohjelmassa
    Board_initUART();

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
        }

      // Otetaan pinnit käyttöön ohjelmassa
      buttonHandle = PIN_open(&buttonState, buttonConfig);
      if(!buttonHandle) {
         System_abort("Error initializing button pins\n");
      }

      // funktio buttonFxn // Asetetaan käsittelijä buttonFxn pinnille
      if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
         System_abort("Error registering button callback function");
      }



//  Task
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(mpuTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

      Task_Params_init(&uartTaskParams);
      uartTaskParams.stackSize = STACKSIZE;
      uartTaskParams.stack = &uartTaskStack;
      uartTaskParams.priority=2;
      uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
      if (uartTaskHandle == NULL) {
          System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
