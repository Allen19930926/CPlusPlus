#include <iostream>

using namespace std;

int GetSum(int n);
int GetSum(int n, int factor);

int main()
{
    int num;
    cin >> num;
    while(num--)
    {
        int t;
        cin >> t;
        cout << GetSum(t) << endl;
    }
    return 0;
}

int GetSum(int n)
{
    return GetSum(n, 3) + GetSum(n, 5) - GetSum(n, 15);
}

int GetSum(int n, int factor)
{
    int sum = 0;
    for (int i=factor; i<n; i+=factor)
    {
        sum += i;
    }
    return sum;
}