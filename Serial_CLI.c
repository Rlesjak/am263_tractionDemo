#include "Serial_CLI.h"
#include "Serial_Cmd_HAL.h"
#include "cli.h"
#include "FOC_loop.h"


UART_Handle UART_HANDLE;
UART_Transaction UART_TRANSACTION;
cli_t cli;

// Privatni prototipi
void serial_cli_println(char *string);

cli_status_t handler__print_help(int argc, char **argv);
cli_status_t handler__set_speed(int argc, char **argv);
cli_status_t handler__set_motor_state(int argc, char **argv);


// Definiranje tablice komandi
cmd_t cmd_tbl[] = {
    {
        .cmd = "help",
        .func = handler__print_help
    },
    {
         .cmd = "speed",
         .func = handler__set_speed
    },
    {
         .cmd = "motor",
         .func = handler__set_motor_state
    }
};


// Inicijalizacija
void serial_cli_init(void)
{
    UART_HANDLE = gUartHandle[CONFIG_UART_CONSOLE];
    UART_Transaction_init(&UART_TRANSACTION);

    // - Inicijalizacija CLI interpretera
    // Dodjela implementacije println metode
    cli.println = serial_cli_println;
    // Dodjela liste komandi
    cli.cmd_tbl = cmd_tbl;
    cli.cmd_cnt = sizeof(cmd_tbl)/sizeof(cmd_t);
    cli_init(&cli);
}

// Metoda koja se periodicno poziva
// triggera obradu do sad primljenog stringa
void serial_cli_service(void)
{
    cli_process(&cli);
}

// Metoda koju pozove Serial_Cmd_HAL za svaki dobiveni char
void serial_cli_consumeChar(char recevedChar)
{
    //SerialCmd_send(recevedChar);

    // serial_cli_println("Hello World!");

    cli_put(&cli, recevedChar);

}

// Implementacija println metode potrebne za cli library
void serial_cli_println(char *string)
{
    UART_TRANSACTION.buf = string;
    UART_TRANSACTION.count = (uint32_t)strlen(string);

    UART_write(UART_HANDLE, &UART_TRANSACTION);
}

// ##### Handleri komandi

cli_status_t handler__print_help(int argc, char **argv)
{
    cli.println("HELP function executed\r\n");
    return CLI_OK;
}


// Postavljanje reference brzine
cli_status_t handler__set_speed(int argc, char **argv)
{
    if(argc < 1) return CLI_E_INVALID_ARGS;

    // Setiranje brzine
    if(strcmp(argv[1], "-s") == 0)
    {
        float32_t parsedSpeed = atof(argv[2]);
        FOC_setSpeedRef(parsedSpeed);

        if (parsedSpeed == 0.0) {
            cli.println("Brzina postavljena na 0\r\n");
        }
        else {
            cli.println("Brzina postavljena\r\n");
        }

        return CLI_OK;
    }
    // Ispis brzine
    else if(strcmp(argv[1], "-p") == 0)
    {
        char buff[8];
        sprintf(buff, "%.3f\r\n", FOC_getSpeedRef());
        cli.println(buff);

        return CLI_OK;
    }

    return CLI_E_INVALID_ARGS;
}

// Postavljanje stanja run/stop
cli_status_t handler__set_motor_state(int argc, char **argv)
{
    if(argc < 1) {
        FOC_setMotorRunState(0);
        cli.println("STOP!\r\n");
        return CLI_OK;
    }

    if(strcmp(argv[1], "run") == 0)
    {
        FOC_setMotorRunState(1);
        cli.println("RUN!\r\n");
        return CLI_OK;
    }

    FOC_setMotorRunState(0);
    cli.println("STOP!\r\n");
    return CLI_OK;
}
