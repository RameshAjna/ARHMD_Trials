
#include "app.h"
#include "commands.h"

int api_turn = 0;
char arr[MAX_TOKENS][MAX_SIZE];

extern unsigned char proc_buffer[PROC_BUFFER_SIZE];

/*
 * Parse_line function definition
 */
int parse_line(unsigned char *buf)
{
    char* result;
    int i = 0;
    memset(arr,0,sizeof(arr));
    result = strtok((char *)buf, " ");
    while(result != NULL && i < (MAX_TOKENS - 1)) {
        strncpy(arr[i], result, MAX_SIZE);
        result = strtok(NULL, " ");
        i++;
    }
    strcpy(arr[i],"NULL");
    return 0;
}

/*
 * find_cmd function definition
 */
int find_cmd(CLI_CMD_T *clicommands, char *command)
{
    int index = 0;
    while ( index < MAX_NUM_CMDS ) {
        if (strcmp(clicommands->cmd_name, command) == 0) {
            return ( index );
        }
        index++;
        clicommands++;
    }
    return -1;
}

/*
 * command_process function definition
 */
int command_process(unsigned char *cmd_phrase)
{
    int index = 0, i = 0;
    api_turn = 0;

    CLI_CMD_T *cmd_list = clicommands;

    /*Parsing the commands*/
    parse_line(cmd_phrase);

    /*Find the command in the commands structure*/
    index = find_cmd ( cmd_list, arr[0] );
    if (index == -1) {
        return 3;
    }

    /*increment the pointer to the command*/
    while ( i!= index) {
        cmd_list ++;
        i++;
    }

    if((strcmp(arr[0],"NULL") != 0 ))
    {
        /*call the command handler function for commands with no arguments*/
        ( cmd_list->cmd_handler ) (arr[1],arr[2],arr[3]);
    }

    return 0;
}

/*
 * app_mgr() function
 */
int app_mgr()
{
    command_process(proc_buffer);

    return 0;
}

