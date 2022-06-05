#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include <mysql/mysql.h>

static char *host = "localhost";
static char *user = "root";
static char *pass = "kcci";
static char *dbname = "project";

char device[] = "/dev/ttyACM0";
int fd;
unsigned long baud = 115200;

int main() {
	MYSQL *conn;
	conn = mysql_init(NULL);
	int sql_index, flag = 0;
	char in_sql[200] = {0};
	int res = 0;
	int cnt = 0;
	
	if(!(mysql_real_connect(conn, host, user, pass, dbname, 0, NULL, 0))) {
		fprintf(stderr, "error : %s[%d]\n", mysql_error(conn), mysql_errno(conn));
		exit(1);
	}
	else printf("Connection Successful!\n");
	
	char col_buff[20] = {0};
	char tra_buff[20] = {0};
	int index = 0, red, green, blue, clear, str_len;
	char *pArray[4] = {0};
	char *pToken;
	printf("Raspberry StartUp\n");
	fflush(stdout);

	if((fd = serialOpen(device, baud)) < 0) {
		fprintf(stderr, "Unable %s\n", strerror(errno));
		exit(1);
	}
	if(wiringPiSetup() == -1) return 1;

	if(serialDataAvail(fd)){
		while(cnt < 20) {
			tra_buff[cnt++] = serialGetchar(fd);
			printf("trash_buff : %s\n", col_buff);
		}
	}

	while(1) {
		if(serialDataAvail(fd)) {
			col_buff[index++] = serialGetchar(fd);
			if(col_buff[index - 1] == 'L') {
				flag = 1;
				col_buff[index - 1] = '\0';
				str_len = strlen(col_buff);
				printf("col_buff = %s\n", col_buff);
				pToken = strtok(col_buff, ":");

				int i = 0;
				while(pToken != NULL) {
					pArray[i] = pToken;
					if(++i > 3) break;
					pToken = strtok(NULL, ":");
				}
				red = atoi(pArray[0]);
				green = atoi(pArray[1]);
				blue = atoi(pArray[2]);
				clear = atoi(pArray[3]);
				printf("RED = %d, GREEN = %d, BLUE = %d, CLEAR = %d\n", red, green, blue, clear);
				for(int i = 0; i <= str_len; i++) col_buff[i] = 0;
				index = 0;
			}
			if(red < 100 && green < 100 && blue < 100) {
				if(flag == 1) {
					sprintf(in_sql, "insert into color(ID, DATE, TIME, RED, GREEN, BLUE, CLEAR) values (null, curdate(), curtime(), %d, %d, %d, %d)", red, green, blue, clear);
					res = mysql_query(conn, in_sql);
					if(!res) printf("inserted %lu rows\n", (unsigned long)mysql_affected_rows(conn));
					else {
						fprintf(stderr, "error : %s[%d]\n", mysql_error(conn), mysql_errno(conn));
						exit(1);
					}
				}
			}
		}
		flag = 0;
		fflush(stdout);
	}
	mysql_close(conn);
	return EXIT_SUCCESS;
}
