#include "stdio.h"
#include "mysql.h"

int main2()
{
	MYSQL * con; //= mysql_init((MYSQL*) 0); 
	MYSQL_RES *res;
	MYSQL_ROW row;
	char tmp[400];
	//database configuartion
	char dbuser[30] = "root";
	char dbpasswd[30] = "admin"; // it must be    changed
	char dbip[30] = "localhost";
	char dbname[50] = "picdetect";
	char tablename[4][20] = { "infodata1" ,"infodata2","infodata3","infodata4"};
	char *query = NULL;

	int x;
	int y;
	int rt;//return value  
	unsigned int t;

	int count = 0;

	con = mysql_init((MYSQL*)0);

	if (con != NULL && mysql_real_connect(con, dbip, dbuser, dbpasswd, dbname, 3306, NULL, 0)) {
		if (!mysql_select_db(con, dbname)) {
			printf("Select successfully the database!\n");
			con->reconnect = 1;
			query = "set names \'GBK\'";
			rt = mysql_real_query(con, query, strlen(query));
			if (rt) {
				printf("Error making query: %s !!!\n", mysql_error(con));
			}
			else {
				printf("query %s succeed!\n", query);
			}
		}
	}
	else {
		MessageBoxA(NULL, "Unable to connect the database,check your configuration!", "", NULL);
	}

	//sprintf(tmp, "insert into %s values(%s,%d,%d)", tablename, "null", x, y); //ע���������������ֶε����ݿ��в����¼
	/*sprintf(tmp, "insert into bl values(null,'x','x','x','x')");


	rt = mysql_real_query(con, tmp, strlen(tmp));
	if (rt)
	{
		printf("Error making query: %s !!!\n", mysql_error(con));
	}
	else
	{
		printf("%s executed!!!\n", tmp);
	}*/

	sprintf(tmp, "select * from %s", tablename[0]);
	rt = mysql_real_query(con, tmp, strlen(tmp));
	if (rt)
	{
		printf("Error making query: %s !!!\n", mysql_error(con));
	}
	else
	{
		printf("%s executed!!!\n", tmp);
	}
	res = mysql_store_result(con);//�����������res�ṹ����

	while (row = mysql_fetch_row(res)) {
		//for (t = 0; t<mysql_num_fields(res); t++) {
			printf("%s  ", row[2]);
		//}
		printf(".............\n");
		count++;
	}

	printf("number of rows %d\n", count);
	printf("mysql_free_result...\n");
	mysql_free_result(res);
	mysql_close(con);
	system("pause");
	return 0;
}