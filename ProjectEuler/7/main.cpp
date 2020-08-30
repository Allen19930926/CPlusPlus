#include <iostream>
#include <cmath>

using namespace std;

bool IsPrime(int n);

int main()
{
    int count = 0;
    int i = 2;
    while(count < 10001)
    {
        if (IsPrime(i))
        {
            count++;
            cout << count << '\t' << i << endl;
        }
        i++;
    }
    cout << i-1 << endl;
    return 0;
}

bool IsPrime(int n)
{
    for (int i=2; i<=sqrt(n); i++)
    {
        if (n % i == 0)
            return false;
    }
    return true;
}
