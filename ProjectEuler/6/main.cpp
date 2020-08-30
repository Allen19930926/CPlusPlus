#include <iostream>

using namespace std;

int diff(int n);

int main()
{
    cout << diff(100) << endl;
    return 0;
}

int diff(int n)
{
    int sum = 0;
    int powerSum = 0;
    for (int i=1, j=n; i<=j; i++, j--)
    {
        sum += i + j;
        powerSum += i*i + j*j;
    }
    cout << sum << powerSum << endl;
    return sum * sum - powerSum;
}