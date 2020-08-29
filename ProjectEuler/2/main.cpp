#include <iostream>
#include <vector>

using namespace std;

const int FOUR_MILLION = 4000000;

int Fab(int n);
int FabSum();

int main()
{
    cout << FabSum() << endl;
    return 0;
}

int FabSum()
{
    vector<int> fabArr;
    for (int i=0; i<2000; i++)
    {
        int value = Fab(i);
        if (value > FOUR_MILLION)
        {
            break;
        }
        fabArr.push_back(value);
    }

    int sum = 0;
    for (const auto& i: fabArr)
    {
        if (i % 2 == 0)
            sum += i;
    }
    return sum;
}

int Fab(int n)
{
    if (n == 0 || n == 1)
        return n;
    return Fab(n-1) + Fab(n-2);
}
