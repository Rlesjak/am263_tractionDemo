/*
 * Serial_CLI.h
 */

#ifndef SERIAL_CLI_H_
#define SERIAL_CLI_H_

#define HELP_STRING     "help            - Lista komandi\r\n\
\r\n\
speed           - Postavljanje i citanje referentne brzine\r\n\
   -p           Ispis trenutno postavljene brzine\r\n\
   -s  [val]    Postavljanje ref brzine\r\n\
\r\n\
motor           - postavljanje statusa na STOP!\r\n\
motor run       - postavljanje statusa na RUN!\r\n\
\r\n\
vtest           - Postavljanje i citanje tesnih napona d i q osi\r\n\
   -vd [val]    Postavljanje Vd napona\r\n\
   -vq [val]    Postavljanje Vq napona\r\n\
   -p           Ispis napona\r\n\r\n\
padc            Ispis ocitanja adc\r\n"

void serial_cli_init(void);
void serial_cli_consumeChar(char recevedChar);
void serial_cli_println(char *string);

#endif /* SERIAL_CLI_H_ */
