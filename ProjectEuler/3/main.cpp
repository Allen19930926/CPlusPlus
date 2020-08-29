#include <iostream>
#include <cmath>

using namespace std;

long long MaxPrime(long long n);

int main()
{
    long long n = 600851475143;
    cout << MaxPrime(n) << endl;
    return 0;
}

long long MaxPrime(long long n)
{
    for (long long i=2; i<sqrt(n); i++)
    {
        if (n / i == 1)
        {
            break;
        }
        if (n % i == 0)
        {
            n /= i;
        }
    }
    return n;
}
