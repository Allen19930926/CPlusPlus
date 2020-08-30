#include <iostream>

using namespace std;

int GCD(int a, int b);
int LCM(int a, int b);

int main()
{
    int lcm = 2520;
    for (int i=10; i<21; ++i)
    {
        lcm = LCM(lcm, i);
    }
    cout << lcm << endl;
    return 0;
}

int LCM(int a, int b)
{
    return a * (b / GCD(a,b));
}

int GCD(int a, int b)
{
    return b==0 ? a : GCD(b, a%b);
}