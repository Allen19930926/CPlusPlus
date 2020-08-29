#include <iostream>
#include <string>

using namespace std;

const int MAXIMUM = 1000*1000;

int MaxPalindromiNumber(int n);
bool IsPalindromiNumber(int n);
bool IsHundredFactor(int n);

int main()
{
    cout << MaxPalindromiNumber(MAXIMUM) << endl;
    return 0;
}

int MaxPalindromiNumber(int n)
{
    while(n--)
    {
        if (IsPalindromiNumber(n) && IsHundredFactor(n))
            return n;
    }
    return -1;
}

bool IsPalindromiNumber(int n)
{
    string forward = to_string(n);
    string backward(forward.rbegin(), forward.rend());
    return forward == backward;
}

bool IsHundredFactor(int n)
{
    for (int i=1000; i>99; i--)
    {
        if ((n%i == 0) && (n/i < 1000))
        {
            cout << i << " " << (n/i) << endl;
            return true;
        }
    }
    return false;
}
