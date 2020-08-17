#include "Mystring.h"
#include <cstring>

Mystring::Mystring(const Mystring& s)
{
	if (s.p)
	{
		p = new char[strlen(s.p)+1];
		strcpy(p,s.p);
	}
	else
	{
		p = NULL;
	}
}

Mystring::Mystring(const char* s)
{
	if (s)
	{
		p = new char[strlen(s)+1];
		strcpy(p,s);
	}
	else
	{
		p = NULL;
	}
}

Mystring::~Mystring()
{
	if (p)
	{
		delete [] p;
		p = NULL;
	}
}

const size_t Mystring::size()
{
	if (p)
	{
		return strlen(p);
	}
	else
	{
		return 0;
	}
}


char Mystring::operator[](size_t n)
{
	if (n > strlen(p))
		return '\0';
	return p[n];
}

Mystring& Mystring::operator+(const char* s)
{
	if (!s)
		return *this;
	if (!p)
	{
		p = new char[strlen(s)+1];
				strcpy(p,s);
		return *this;
	}

	char* tmpPtr = new char[strlen(p)+1];
	strcpy(tmpPtr,p);
	delete [] p;
	p = new char[strlen(tmpPtr) + strlen(s) + 1];
	strcpy(p, tmpPtr);
	strcpy(p+strlen(tmpPtr),s);
}

ostream& Mystring::operator<<(ostream& os)
{
	if (p)
	{
		os << p;
	}
	else
	{
		os << "Null ptr!";
	}
	return os;
}

Mystring& Mystring::copy(const Mystring& s)
{
	delete p;
	if (!s.p)
	{
		p = NULL;
	}
	p = new char[strlen(s.p)+1];
	strcpy(p, s.p);
	return *this;
}

Mystring& Mystring::copy(const char* s)
{
	delete p;
	if (!s)
	{
		p = NULL;
	}
	p = new char[strlen(s)+1];
	strcpy(p, s);
	return *this;
}


ostream& operator<<(ostream& os, Mystring& str)
{
	if (str.p)
	{
		os << str.p;
	}
	else
	{
		os << "Null ptr!";
	}
	return os;
}
