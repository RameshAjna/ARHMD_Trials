#ifndef __CLI_APP_H
#define __CLI_APP_H

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "sh2.h"
#include "commands.h"

#define MAX_TOKENS			5
#define MAX_SIZE 			25
#define PROC_BUFFER_SIZE    100

#define SUCCESS 0
#define FAILURE 1

int data_flag_bit;

/*
 * Parse_line function definition
 */
int parse_line(unsigned char *buf);
/*
 * find_cmd function definition
 */
int find_cmd(CLI_CMD_T *clicommands, char *command);
/*
 * command_process function definition
 */
int command_process(unsigned char *cmd_phrase);
/*
 * app_mgr() function
 */
int app_mgr();

#endif // __APP_H
