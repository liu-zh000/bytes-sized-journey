
#include <string>
#include <iostream>
#include <vector>
using namespace std;
class Solution {
public:
    string multiply(string num1, string num2) {
        int a = num1.size();
        int b = num2.size();
        int cheng10 = 1;
        int cheng = 0;
        int numn1i = 0 ; int numn2i = 0;
        vector<int> res ;
        //string res = "";
        int m = 0 ;int jinwei = 0;
        for(int i = a-1 ;i>=0 ; i--)//123
        {
            
            for(int j = b-1 ;j >=0 ; j--)
            {
                cheng = (num1[i] - '0') * (num2[j] - '0');
                cheng = cheng * cheng10;
                cheng10 = 10 * cheng10;
                //res[m] = cheng%10 + jinwei;
                //jinwei = cheng/10;  
                //m++;
            }
            res.push_back(cheng);
            cheng10 = 1;
        }

        int sum = 0 ;
        for(int l = res.size() -1  ; l >=0; l-- )
        {
            sum  = sum + res[l]*cheng10;
             cheng10 = 10 * cheng10;
        }

        
        return to_string(sum);
    }
};

int main()
{
    Solution a;
    string result = a.multiply("123","456");
    std::cout << "结果是："<<result << std::endl;
}

// class Solution {
// public:
//     string multiply(string num1, string num2) {
//         int a = num1.size();
//         int b = num2.size();
//         int numn1i = 0 ; int numn2i = 0;
//         for(int i = 0 ;i<=a-1 ; i++)//123
//         {
//             numn1i = numn1i * 10 + int(num1[i] - '0') ;
//         }

//         for(int j = 0 ;j <=b-1 ; j++)
//         {
//             numn2i = numn2i * 10 + int(num2[j] - '0' );
//         }

//         return to_string(numn1i * numn2i);
//     }
// };