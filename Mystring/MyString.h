#ifndef MYSTRING_H_
#define MYSTRING_H_

#include <iostream>

using namespace std;

class Mystring
{
public:
	Mystring(): p(NULL) {}
	Mystring(const Mystring& s);
	Mystring(const char* s);
	~Mystring();
	const size_t size();
	char operator[](size_t n);
	Mystring& operator+(const char* s);
	Mystring& copy(const Mystring& s);
	Mystring& copy(const char* s);
	ostream& operator<<(ostream& os);
	friend ostream& operator<<(ostream& os, Mystring& str);

private:
	char* p;
};

#endif /* MYSTRING_H_ */
