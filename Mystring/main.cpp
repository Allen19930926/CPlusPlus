#include "MyString.h"

int main()
{
	Mystring str1;
	Mystring str2("hello world!");
	Mystring str3(str2);

	cout << str1 << endl;
	cout << str2 << endl;
	cout << str3 << endl;

	str1.copy(str2);
	str2.copy("glory of king");
//	str3 = str3 + "hello china!";

	cout << str1 << endl;
	cout << str2 << endl;
	cout << str3 << endl;
}



